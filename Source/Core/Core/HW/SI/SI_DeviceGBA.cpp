// Copyright 2009 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include <utility>
#include <vector>

#include "Common/ChunkFile.h"
#include "Common/CommonTypes.h"
#include "Common/Logging/Log.h"
#include "Common/Swap.h"
#include "Core/ConfigManager.h"
#include "Core/Core.h"
#include "Core/CoreTiming.h"
#include "Core/HW/GBACore.h"
#include "Core/HW/SI/SI.h"
#include "Core/HW/SI/SI_DeviceGBA.h"
#include "Core/HW/SystemTimers.h"

namespace SerialInterface
{
namespace
{
int s_num_connected = 0;
} // namespace

// --- GameBoy Advance "Link Cable" ---

static std::pair<int, int> GetTransferSize(u8 cmd)
{
  switch (cmd)
  {
  case GBASIOJOYCommand::JOY_RESET:
  case GBASIOJOYCommand::JOY_POLL:
    return {1, 3};
  case GBASIOJOYCommand::JOY_TRANS:
    return {1, 5};
  case GBASIOJOYCommand::JOY_RECV:
    return {5, 1};
  }
  return {1, 0};
}

static int GetSendTime(u8 cmd)
{
  return static_cast<int>(GetTransferSize(cmd).first * SystemTimers::GetTicksPerSecond() /
                          (s_num_connected * HW::GBA::BYTES_PER_SECOND));
}

static int GetResponseTime(u8 cmd)
{
  return static_cast<int>(GetTransferSize(cmd).second * SystemTimers::GetTicksPerSecond() /
                          HW::GBA::BYTES_PER_SECOND);
}

static int GetTransferTime(u8 cmd)
{
  return GetSendTime(cmd) + GetResponseTime(cmd);
}

static s64 GetSyncInterval()
{
  return SystemTimers::GetTicksPerSecond() / 1000;
}

CSIDevice_GBA::CSIDevice_GBA(SIDevices device, int device_number)
    : ISIDevice(device, device_number), m_core(device_number, SConfig::GetInstance().bGBAThreads),
      m_init(false)
{
  if (Core::IsRunningAndStarted())
    PostInit();
  ++s_num_connected;
}

CSIDevice_GBA::~CSIDevice_GBA()
{
  RemoveEvent(m_device_number);

  if (m_init)
    PreShutdown();
  --s_num_connected;
}

void CSIDevice_GBA::PostInit()
{
  m_core.Init(CoreTiming::GetTicks());
  m_init = true;
}

void CSIDevice_GBA::PreShutdown()
{
  m_init = false;
  m_core.Deinit();
}

int CSIDevice_GBA::RunBuffer(u8* buffer, int request_length)
{
  RemoveEvent(m_device_number);
  switch (m_next_action)
  {
  case NextAction::SendCommand:
  {
#ifdef _DEBUG
    NOTICE_LOG_FMT(SERIALINTERFACE, "{} cmd {:02x} [> {:02x}{:02x}{:02x}{:02x}]", m_device_number,
                   buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
#endif
    m_last_cmd = buffer[0];
    m_timestamp_sent = CoreTiming::GetTicks();
    m_core.SendJoybusCommand(m_timestamp_sent, buffer);

    m_next_action = NextAction::WaitTransferTime;
    [[fallthrough]];
  }

  case NextAction::WaitTransferTime:
  {
    int elapsed_time = static_cast<int>(CoreTiming::GetTicks() - m_timestamp_sent);
    // Tell SI to ask again after TransferInterval() cycles
    if (GetTransferTime(m_last_cmd) > elapsed_time)
      return 0;
    m_next_action = NextAction::ReceiveResponse;
    [[fallthrough]];
  }

  case NextAction::ReceiveResponse:
  {
    std::vector<u8> response = m_core.GetJoybusResponse();
    m_next_action = NextAction::SendCommand;

    if (response.empty())
    {
      u32 reply = Common::swap32(SI_ERROR_NO_RESPONSE);
      std::memcpy(buffer, &reply, sizeof(reply));
      ScheduleEvent(m_device_number, GetSyncInterval());
      return sizeof(reply);
    }
    std::copy(response.begin(), response.end(), buffer);

#ifdef _DEBUG
    const Common::Log::LOG_LEVELS log_level =
        (m_last_cmd == GBASIOJOYCommand::JOY_POLL || m_last_cmd == GBASIOJOYCommand::JOY_RESET) ?
            Common::Log::LERROR :
            Common::Log::LWARNING;
    GENERIC_LOG_FMT(Common::Log::SERIALINTERFACE, log_level,
                    "{}                              [< {:02x}{:02x}{:02x}{:02x}{:02x}] ({})",
                    m_device_number, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4],
                    response.size());
#endif

    ScheduleEvent(m_device_number, GetSyncInterval());
    return static_cast<int>(response.size());
  }
  }

  // This should never happen, but appease MSVC which thinks it might.
  ERROR_LOG_FMT(SERIALINTERFACE, "Unknown state {}\n", m_next_action);
  return 0;
}

int CSIDevice_GBA::TransferInterval()
{
  return GetTransferTime(m_last_cmd);
}

bool CSIDevice_GBA::GetData(u32& hi, u32& low)
{
  return false;
}

void CSIDevice_GBA::SendCommand(u32 command, u8 poll)
{
}

void CSIDevice_GBA::DoState(PointerWrap& p)
{
  p.Do(m_next_action);
  p.Do(m_last_cmd);
  p.Do(m_timestamp_sent);
  m_core.DoState(p);
}

void CSIDevice_GBA::OnEvent(s64 cycles_late)
{
  m_core.SendJoybusCommand(CoreTiming::GetTicks(), nullptr, true);
  ScheduleEvent(m_device_number, GetSyncInterval());
}

}  // namespace SerialInterface
