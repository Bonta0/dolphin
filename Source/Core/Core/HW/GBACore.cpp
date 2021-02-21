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

#include "AudioCommon/AudioCommon.h"
#include "Common/ChunkFile.h"
#include "Common/CommonTypes.h"
#include "Common/FileUtil.h"
#include "Core/HW/GBACore.h"
#include "Core/HW/GBAFrontend.h"
#include "Core/HW/SI/SI_DeviceGCController.h"
#include "Core/HW/SystemTimers.h"
#include "Core/NetPlayProto.h"

namespace HW::GBA
{
constexpr auto SAMPLES = 512;
constexpr auto SAMPLE_RATE = 48000;

static std::unique_ptr<FrontendInterface> CreateDummyFrontend(int device_number, u32 width,
                                                              u32 height)
{
  return std::make_unique<FrontendInterface>();
}
std::unique_ptr<FrontendInterface> (*s_create_frontend)(int, u32, u32) = CreateDummyFrontend;

Core::Core(int device_number) : m_device_number(device_number)
{
}

Core::~Core()
{
}

void Core::Init(u64 gc_ticks)
{
  m_last_gc_ticks = gc_ticks;
  m_gc_ticks_remainder = 0;
  m_link_enabled = false;

  m_core = mCoreCreate(mPlatform::mPLATFORM_GBA);
  m_core->init(m_core);

  mCoreInitConfig(m_core, nullptr);
  mCoreConfigSetValue(&m_core->config, "idleOptimization", "detect");
  mCoreConfigSetIntValue(&m_core->config, "useBios", 1);
  mCoreConfigSetIntValue(&m_core->config, "skipBios", 0);

  // TODO: tell the user if it's missing
  std::string bios_file = File::GetUserPath(D_GCUSER_IDX) + "gba_bios.bin";
  VFile* vf = VFileOpen(bios_file.c_str(), O_RDONLY);
  m_core->loadBIOS(m_core, vf, 0);

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

  mCoreCallbacks callbacks{};
  callbacks.context = this;
  callbacks.keysRead = [](void* context) {
    reinterpret_cast<Core*>(context)->OnKeysRead();
  };
  callbacks.videoFrameEnded = [](void* context) {
    auto core = reinterpret_cast<Core*>(context);
    core->m_frontend->FrameEnded(core->m_video_buffer);
  };
  m_core->addCoreCallbacks(m_core, &callbacks);

  u32 width, height;
  m_core->desiredVideoDimensions(m_core, &width, &height);
  m_video_buffer.resize(width * height);
  m_core->setVideoBuffer(m_core, m_video_buffer.data(), width);

  m_core->setAudioBufferSize(m_core, SAMPLES);
  blip_set_rates(m_core->getAudioChannel(m_core, 0), GBA_ARM7TDMI_FREQUENCY, SAMPLE_RATE);
  blip_set_rates(m_core->getAudioChannel(m_core, 1), GBA_ARM7TDMI_FREQUENCY, SAMPLE_RATE);
  g_sound_stream->GetMixer()->SetGBAInputSampleRates(m_device_number, SAMPLE_RATE);

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

  m_event.context = m_core->board;
  m_event.name = "Dolphin Sync";
  m_event.callback = [](mTiming* timing, void* context, u32 cycles_late) {
    static_cast<::GBA*>(context)->earlyExit = true;
  };
  m_event.priority = 0x80;

  m_frontend = s_create_frontend(m_device_number, width, height);

  m_core->reset(m_core);
}

void Core::Deinit()
{
  Flush();
  m_frontend->Stop();
  mCoreConfigDeinit(&m_core->config);
  m_core->deinit(m_core);
}

void Core::SendJoybusCommand(u64 gc_ticks, u8* buffer)
{
  std::copy(buffer, buffer + 5, m_command_buffer.begin());
  RunUntil(gc_ticks);
}

std::vector<u8> Core::GetJoybusResponse()
{
  std::vector<u8> response_buffer;
  if (m_link_enabled)
  {
    int recvd = GBASIOJOYSendCommand(
        &m_sio_driver, static_cast<GBASIOJOYCommand>(m_command_buffer[0]), &m_command_buffer[1]);
    std::copy(m_command_buffer.begin() + 1, m_command_buffer.begin() + 1 + recvd,
              std::back_inserter(response_buffer));
    RunFor(response_buffer.size() * SystemTimers::GetTicksPerSecond() / BYTES_PER_SECOND);
  }
  return response_buffer;
}

void Core::Flush()
{
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
  p.Do(m_command_buffer);
  p.Do(m_last_gc_ticks);
  p.Do(m_gc_ticks_remainder);
  p.Do(m_link_enabled);

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
  //TODO: get from own config and threadsafety
  GCPadStatus pad_status{};

  if (!NetPlay::IsNetPlayRunning())
    pad_status = Pad::GetStatus(m_device_number);

  SerialInterface::CSIDevice_GCController::HandleMoviePadStatus(m_device_number, &pad_status);

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
