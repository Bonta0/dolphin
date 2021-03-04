// Copyright 2021 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#define PYCPARSE  // Remove static functions from the header
#include <mgba/core/interface.h>
#undef PYCPARSE
#include <mgba/core/blip_buf.h>
#include <mgba/core/timing.h>
#include <mgba/internal/gba/gba.h>
#include <mgba-util/vfs.h>
#include <unzip.h>

#include "AudioCommon/AudioCommon.h"
#include "Common/ChunkFile.h"
#include "Common/CommonTypes.h"
#include "Common/CommonPaths.h"
#include "Common/Config/Config.h"
#include "Common/FileUtil.h"
#include "Common/MinizipUtil.h"
#include "Common/Thread.h"
#include "Core/Config/MainSettings.h"
#include "Core/Core.h"
#include "Core/HW/GBACore.h"
#include "Core/HW/GBAPad.h"
#include "Core/HW/GBAFrontend.h"
#include "Core/HW/SI/SI_DeviceGCController.h"
#include "Core/HW/SystemTimers.h"
#include "Core/Movie.h"
#include "Core/NetPlayProto.h"

namespace HW::GBA
{
constexpr auto SAMPLES = 512;
constexpr auto SAMPLE_RATE = 48000;

static FrontendFactory s_frontend_factory = [](int, const char*, u32, u32) {
  return std::make_unique<FrontendInterface>();
};

void SetFrontendFactory(FrontendFactory factory)
{
  s_frontend_factory = factory;
}

Core::Core(int device_number, u64 gc_ticks)
    : m_device_number(device_number), m_last_gc_ticks(gc_ticks), m_gc_ticks_remainder(0),
      m_link_enabled(false)
{
  m_rom_path = Pad::GetGBARomPath(m_device_number);

  m_core = mCoreCreate(mPlatform::mPLATFORM_GBA);
  m_core->init(m_core);

  mCoreInitConfig(m_core, nullptr);
  mCoreConfigSetValue(&m_core->config, "idleOptimization", "detect");
  mCoreConfigSetIntValue(&m_core->config, "useBios", 1);
  mCoreConfigSetIntValue(&m_core->config, "skipBios", 0);

  LoadBIOS();
  LoadROM();
  SetSIODriver();
  SetCallbacks();

  u32 width, height;
  m_core->desiredVideoDimensions(m_core, &width, &height);
  m_video_buffer.resize(width * height);
  m_core->setVideoBuffer(m_core, m_video_buffer.data(), width);

  m_core->setAudioBufferSize(m_core, SAMPLES);
  blip_set_rates(m_core->getAudioChannel(m_core, 0), GBA_ARM7TDMI_FREQUENCY, SAMPLE_RATE);
  blip_set_rates(m_core->getAudioChannel(m_core, 1), GBA_ARM7TDMI_FREQUENCY, SAMPLE_RATE);
  g_sound_stream->GetMixer()->SetGBAInputSampleRates(m_device_number, SAMPLE_RATE);

  SetAVStream();
  SetupEvent();

  std::string game_title(256, '\0');
  m_core->getGameTitle(m_core, game_title.data());
  m_frontend = s_frontend_factory(m_device_number, game_title.data(), width, height);

  m_core->reset(m_core);

  if (Config::Get(Config::MAIN_GBA_THREADS) && !Movie::IsRecordingInput() && !Movie::IsPlayingInput())
  {
    m_idle = true;
    m_exit_loop = false;
    m_thread = std::make_unique<std::thread>([this] { ThreadLoop(); });
  }
}

Core::~Core()
{
  if (m_thread)
  {
    Flush();
    m_exit_loop = true;
    {
      std::lock_guard<std::mutex> lock(m_queue_mutex);
      m_command_cv.notify_one();
    }
    m_thread->join();
    m_thread.reset();
  }
  m_frontend->Stop();
  mCoreConfigDeinit(&m_core->config);
  m_core->deinit(m_core);
}

void Core::LoadBIOS()
{
  std::string bios_path = File::GetUserPath(F_GBABIOS_IDX);
  VFile* vf = VFileOpen(bios_path.c_str(), O_RDONLY);
  if (!vf)
  {
    PanicAlertFmtT("Error: GBA{0} failed to open the BIOS in {1}", m_device_number + 1, bios_path);
    return;
  }

  if (!m_core->loadBIOS(m_core, vf, 0))
  {
    PanicAlertFmtT("Error: GBA{0} failed to load the BIOS in {1}", m_device_number + 1, bios_path);
    vf->close(vf);
    return;
  }
}

static VFile* LoadROM_Archive(const char* path)
{
  VFile* vf{};
  VDir* archive = VDirOpenArchive(path);
  if (archive)
  {
    VFile* vf_archive =
        VDirFindFirst(archive, [](VFile* vf) { return mCoreIsCompatible(vf) == mPLATFORM_GBA; });
    if (vf_archive)
    {
      size_t size = static_cast<size_t>(vf_archive->size(vf_archive));

      std::vector<u8> buffer(size);
      vf_archive->seek(vf_archive, 0, SEEK_SET);
      vf_archive->read(vf_archive, buffer.data(), size);
      vf_archive->close(vf_archive);

      vf = VFileMemChunk(buffer.data(), size);
    }
    archive->close(archive);
  }
  return vf;
}

static VFile* LoadROM_Zip(const char* path)
{
  VFile* vf{};
  unzFile zip = unzOpen(path);
  if (zip)
  {
    do
    {
      unz_file_info info{};
      if (unzGetCurrentFileInfo(zip, &info, nullptr, 0, nullptr, 0, nullptr, 0) != UNZ_OK ||
          !info.uncompressed_size)
        continue;

      std::vector<u8> buffer(info.uncompressed_size);
      if (!Common::ReadFileFromZip(zip, &buffer))
        continue;

      vf = VFileMemChunk(buffer.data(), info.uncompressed_size);
      if (mCoreIsCompatible(vf) == mPLATFORM_GBA)
      {
        vf->seek(vf, 0, SEEK_SET);
        break;
      }

      vf->close(vf);
      vf = nullptr;
    } while (unzGoToNextFile(zip) == UNZ_OK);
    unzClose(zip);
  }
  return vf;
}

void Core::LoadROM()
{
  if (m_rom_path.empty())
    return;

  VFile* vf{};

  vf = LoadROM_Archive(m_rom_path.c_str());
  if (!vf)
    vf = LoadROM_Zip(m_rom_path.c_str());
  if (!vf)
  {
    vf = VFileOpen(m_rom_path.c_str(), O_RDONLY);
    if (vf)
    {
      if (mCoreIsCompatible(vf) != mPLATFORM_GBA)
      {
        vf->close(vf);
        vf = nullptr;
      }
      else
      {
        vf->seek(vf, 0, SEEK_SET);
      }
    }
  }

  if (!vf)
  {
    PanicAlertFmtT("Error: GBA{0} failed to open the ROM in {1}", m_device_number + 1, m_rom_path);
    return;
  }

  if (!m_core->loadROM(m_core, vf))
  {
    PanicAlertFmtT("Error: GBA{0} failed to load the ROM in {1}", m_device_number + 1, m_rom_path);
    vf->close(vf);
    return;
  }

  LoadSave();
}

void Core::LoadSave()
{
  std::string save_path = fmt::format(
      "{}_{}.sav", m_rom_path.substr(0, m_rom_path.find_last_of('.')), m_device_number + 1);

  if (!Config::Get(Config::MAIN_GBA_SAVES_IN_ROM_PATH))
  {
    save_path =
        File::GetUserPath(D_GBASAVES_IDX) + save_path.substr(save_path.find_last_of("\\/") + 1);
  }

  VFile* vf = VFileOpen(save_path.c_str(), O_CREAT | O_RDWR);
  if (!vf)
  {
    PanicAlertFmtT("Error: GBA{0} failed to open the save in {1}", m_device_number + 1, save_path);
    return;
  }

  if (!m_core->loadSave(m_core, vf))
  {
    PanicAlertFmtT("Error: GBA{0} failed to load the save in {1}", m_device_number + 1, save_path);
    vf->close(vf);
    return;
  }
}

void Core::SetSIODriver()
{
  GBASIOJOYCreate(&m_sio_driver);
  GBASIOSetDriver(&static_cast<::GBA*>(m_core->board)->sio, &m_sio_driver, SIO_JOYBUS);

  m_sio_driver.core = this;
  m_sio_driver.load = [](GBASIODriver* driver) {
    static_cast<SIODriver*>(driver)->core->m_link_enabled = true;
    return true;
  };
  m_sio_driver.unload = [](GBASIODriver* driver) {
    static_cast<SIODriver*>(driver)->core->m_link_enabled = false;
    return true;
  };
}

void Core::SetCallbacks()
{
  mCoreCallbacks callbacks{};
  callbacks.context = this;
  callbacks.keysRead = [](void* context) { reinterpret_cast<Core*>(context)->OnKeysRead(); };
  callbacks.videoFrameEnded = [](void* context) {
    auto core = reinterpret_cast<Core*>(context);
    core->m_frontend->FrameEnded(core->m_video_buffer);
  };
  m_core->addCoreCallbacks(m_core, &callbacks);
}

void Core::SetAVStream()
{
  m_stream.core = this;
  m_stream.videoDimensionsChanged = nullptr;
  m_stream.postVideoFrame = nullptr;
  m_stream.postAudioFrame = nullptr;
  m_stream.postAudioBuffer = [](struct mAVStream* stream, struct blip_t* left,
                                struct blip_t* right) {
    auto device = static_cast<AVStream*>(stream)->core;
    std::vector<s16> buffer(SAMPLES * 2);
    blip_read_samples(left, &buffer[0], SAMPLES, 1);
    blip_read_samples(right, &buffer[1], SAMPLES, 1);
    g_sound_stream->GetMixer()->PushGBASamples(device->m_device_number, &buffer[0], SAMPLES);
  };
  m_core->setAVStream(m_core, &m_stream);
}

void Core::SetupEvent()
{
  m_event.context = m_core->board;
  m_event.name = "Dolphin Sync";
  m_event.callback = [](mTiming* timing, void* context, u32 cycles_late) {
    static_cast<::GBA*>(context)->earlyExit = true;
  };
  m_event.priority = 0x80;
}

void Core::SendJoybusCommand(u64 gc_ticks, u8* buffer)
{
  Command command{};
  command.ticks = gc_ticks;
  command.sync_only = buffer == nullptr;
  if (buffer)
    std::copy(buffer, buffer + 5, command.buffer.begin());

  if (m_thread)
  {
    std::lock_guard<std::mutex> lock(m_queue_mutex);
    m_command_queue.push(command);
    m_idle = false;
    m_command_cv.notify_one();
  }
  else
  {
    RunCommand(command);
  }
}

std::vector<u8> Core::GetJoybusResponse()
{
  if (m_thread)
  {
    std::unique_lock<std::mutex> lock(m_response_mutex);
    m_response_cv.wait(lock, [&] { return m_response_ready; });
  }
  m_response_ready = false;
  return m_response;
}

void Core::Flush()
{
  if (!m_thread)
    return;
  std::unique_lock<std::mutex> lock(m_queue_mutex);
  m_response_cv.wait(lock, [&] { return m_idle; });
}

void Core::ThreadLoop()
{
  Common::SetCurrentThreadName(fmt::format("GBA{}", m_device_number + 1).c_str());
  std::unique_lock<std::mutex> queue_lock(m_queue_mutex);
  while (true)
  {
    m_command_cv.wait(queue_lock, [&] { return !m_command_queue.empty() || m_exit_loop; });
    if (m_exit_loop)
      break;
    Command command{m_command_queue.front()};
    m_command_queue.pop();
    queue_lock.unlock();

    RunCommand(command);

    queue_lock.lock();
    if (m_command_queue.empty())
      m_idle = true;
    m_response_cv.notify_one();
  }
}

void Core::RunCommand(Command& command)
{
  RunUntil(command.ticks);
  if (!command.sync_only)
  {
    u64 run_ahead = 0;
    m_response.clear();
    if (m_link_enabled)
    {
      int recvd = GBASIOJOYSendCommand(
          &m_sio_driver, static_cast<GBASIOJOYCommand>(command.buffer[0]), &command.buffer[1]);
      std::copy(command.buffer.begin() + 1, command.buffer.begin() + 1 + recvd,
                std::back_inserter(m_response));
      run_ahead = m_response.size() * SystemTimers::GetTicksPerSecond() / BYTES_PER_SECOND;
    }
    if (m_thread && !m_response_ready)
    {
      std::lock_guard<std::mutex> response_lock(m_response_mutex);
      m_response_ready = true;
      m_response_cv.notify_one();
    }
    else
      m_response_ready = true;
    if (run_ahead)
      RunFor(run_ahead);
  }
}

void Core::RunUntil(u64 gc_ticks)
{
  bool scheduled = false;
  u64 gc_frequency = SystemTimers::GetTicksPerSecond();
  while (static_cast<s64>(gc_ticks - m_last_gc_ticks) > 0)
  {
    if (!scheduled && gc_ticks - m_last_gc_ticks < gc_frequency)
    {
      scheduled = true;
      mTimingSchedule(
          m_core->timing, &m_event,
          static_cast<s32>((gc_ticks - m_last_gc_ticks) * GBA_ARM7TDMI_FREQUENCY / gc_frequency));
    }
    u32 start_time = mTimingCurrentTime(m_core->timing);
    m_core->runLoop(m_core);
    u64 run_cycles = static_cast<u32>(mTimingCurrentTime(m_core->timing)) - start_time;

    u64 d = (run_cycles * gc_frequency) + m_gc_ticks_remainder;
    m_last_gc_ticks += d / GBA_ARM7TDMI_FREQUENCY;
    m_gc_ticks_remainder = d % GBA_ARM7TDMI_FREQUENCY;
  }
  mTimingDeschedule(m_core->timing, &m_event);
}

void Core::RunFor(u64 gc_ticks)
{
  RunUntil(m_last_gc_ticks + gc_ticks);
}

void Core::DoState(PointerWrap& p)
{
  Flush();

  p.Do(m_video_buffer);
  p.Do(m_last_gc_ticks);
  p.Do(m_gc_ticks_remainder);
  p.Do(m_link_enabled);
  p.Do(m_response_ready);
  p.Do(m_response);

  std::vector<u8> core_state(m_core->stateSize(m_core));
  if (p.GetMode() == PointerWrap::MODE_WRITE || p.GetMode() == PointerWrap::MODE_VERIFY)
    m_core->saveState(m_core, core_state.data());
  p.Do(core_state);
  if (p.GetMode() == PointerWrap::MODE_READ)
  {
    m_core->loadState(m_core, core_state.data());
    m_frontend->FrameEnded(m_video_buffer);
  }
}

void Core::OnKeysRead()
{
  m_core->setKeys(m_core, GetPadStatus());
}

u16 Core::GetPadStatus()
{
  GCPadStatus pad_status{};

  if (!NetPlay::IsNetPlayRunning())
    pad_status = Pad::GetGBAStatus(m_device_number);

  if (m_thread && (Movie::IsRecordingInput() || Movie::IsPlayingInput()))
  {
    PanicAlertFmtT("Warning: Movies are disabled when using multithreaded GBAs");
    SerialInterface::CSIDevice_GCController::NetPlay_GetInput(m_device_number, &pad_status);
  }
  else
  {
    SerialInterface::CSIDevice_GCController::HandleMoviePadStatus(m_device_number, &pad_status);
  }

  static constexpr std::array buttons_map = {
      PadButton::PAD_BUTTON_A,      // A
      PadButton::PAD_BUTTON_B,      // B
      PadButton::PAD_TRIGGER_Z,     // Select
      PadButton::PAD_BUTTON_START,  // Start
      PadButton::PAD_BUTTON_RIGHT,  // Right
      PadButton::PAD_BUTTON_LEFT,   // Left
      PadButton::PAD_BUTTON_UP,     // Up
      PadButton::PAD_BUTTON_DOWN,   // Down
      PadButton::PAD_TRIGGER_R,     // R
      PadButton::PAD_TRIGGER_L,     // L
  };

  u16 ret{};
  for (size_t i = 0; i < buttons_map.size(); ++i)
    ret |= static_cast<std::uint16_t>(static_cast<bool>((pad_status.button & buttons_map[i]))) << i;

  return ret;
}
}  // namespace HW::GBA
