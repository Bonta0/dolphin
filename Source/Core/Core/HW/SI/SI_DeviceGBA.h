// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <memory>

#include "Common/CommonTypes.h"
#include "Core/HW/GBACore.h"
#include "Core/HW/SI/SI_Device.h"

// GameBoy Advance "Link Cable"

namespace SerialInterface
{
class CSIDevice_GBA : public ISIDevice
{
public:
  CSIDevice_GBA(SIDevices device, int device_number);
  ~CSIDevice_GBA();

  void ResetCore();

  int RunBuffer(u8* buffer, int request_length) override;
  int TransferInterval() override;
  bool GetData(u32& hi, u32& low) override;
  void SendCommand(u32 command, u8 poll) override;
  void DoState(PointerWrap& p) override;
  void OnEvent(s64 cycles_late) override;

private:

  enum class NextAction
  {
    SendCommand,
    WaitTransferTime,
    ReceiveResponse
  };

  NextAction m_next_action = NextAction::SendCommand;
  u8 m_last_cmd{};
  u64 m_timestamp_sent{};

  std::unique_ptr<HW::GBA::Core> m_core;
};
}  // namespace SerialInterface
