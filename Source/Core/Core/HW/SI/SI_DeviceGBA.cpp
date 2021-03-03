// Copyright 2009 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include <vector>

#include "Common/ChunkFile.h"
#include "Common/CommonTypes.h"
#include "Common/Logging/Log.h"
#include "Common/Swap.h"
#include "Core/Core.h"
#include "Core/CoreTiming.h"
#include "Core/HW/GBACore.h"
#include "Core/HW/SI/SI.h"
#include "Core/HW/SI/SI_DeviceGBA.h"
#include "Core/HW/SystemTimers.h"

namespace SerialInterface
{
// --- GameBoy Advance "Link Cable" ---

static int GetTransferTime(u8 cmd)
{
  u64 gc_bits_transferred = 8;
  u64 gba_bits_transferred = 0;

  switch (cmd)
  {
  case GBASIOJOYCommand::JOY_RESET:
  case GBASIOJOYCommand::JOY_POLL:
  {
    gba_bits_transferred = 24;
    break;
  }
  case GBASIOJOYCommand::JOY_TRANS:
  {
    gba_bits_transferred = 40;
    break;
  }
  case GBASIOJOYCommand::JOY_RECV:
  {
    gc_bits_transferred += 32;
    gba_bits_transferred += 8;
    break;
  }
  default:
  {
    break;
  }
  }

  u64 cycles =
      (gba_bits_transferred * SystemTimers::GetTicksPerSecond() / HW::GBA::GBA_BITS_PER_SECOND) +
      (gc_bits_transferred * SystemTimers::GetTicksPerSecond() / HW::GBA::GC_BITS_PER_SECOND) +
      ((HW::GBA::GC_STOP_BIT_NS + HW::GBA::GBA_STOP_BIT_NS) * SystemTimers::GetTicksPerSecond() /
       1000000000LL);
  return static_cast<int>(cycles);
}

static s64 GetSyncInterval()
{
  return SystemTimers::GetTicksPerSecond() / 1000;
}

CSIDevice_GBA::CSIDevice_GBA(SIDevices device, int device_number) : ISIDevice(device, device_number)
{
  ResetCore();
}

CSIDevice_GBA::~CSIDevice_GBA()
{
  RemoveEvent(m_device_number);
}

void CSIDevice_GBA::ResetCore()
{
  m_core = std::make_unique<HW::GBA::Core>(m_device_number, CoreTiming::GetTicks());
  m_next_action = NextAction::SendCommand;
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
    m_core->SendJoybusCommand(m_timestamp_sent, buffer);

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
    std::vector<u8> response = m_core->GetJoybusResponse();
    m_next_action = NextAction::SendCommand;

    if (response.empty())
    {
      ScheduleEvent(m_device_number, GetSyncInterval());
      return -1;
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
  m_core->DoState(p);
}

void CSIDevice_GBA::OnEvent(s64 cycles_late)
{
  m_core->SendJoybusCommand(CoreTiming::GetTicks(), nullptr);
  ScheduleEvent(m_device_number, GetSyncInterval());
}

}  // namespace SerialInterface
