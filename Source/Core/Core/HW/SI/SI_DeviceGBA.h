// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <array>
#include <memory>

#include <SFML/Network.hpp>

#include "Common/CommonTypes.h"
#include "Core/HW/SI/SI_Device.h"

// GameBoy Advance "Link Cable"

namespace SerialInterface
{
void GBAConnectionWaiter_Shutdown();

class GBASockServer
{
public:
  GBASockServer() = default;
  ~GBASockServer() = default;

  bool Send(const u8* si_buffer, s32 ticks, s32 waiting, u16 pad);
  int Receive(u8* si_buffer);

  bool IsConnected() const;

private:
  sf::TcpSocket m_client;

  bool m_connected = false;
  bool m_disconnected = false;
};

class CSIDevice_GBA : public ISIDevice
{
public:
  CSIDevice_GBA(SIDevices device, int device_number);
  ~CSIDevice_GBA();

  int RunBuffer(u8* buffer, int request_length) override;
  int TransferInterval() override;
  bool GetData(u32& hi, u32& low) override;
  void SendCommand(u32 command, u8 poll) override;

private:
  enum class NextAction
  {
    SendCommand,
    WaitTransferTime,
    ReceiveResponse
  };

  u16 GetGBAPad();

  GBASockServer m_sock_server;
  NextAction m_next_action = NextAction::SendCommand;
  u64 m_timestamp_sent = 0;
  u8 m_last_cmd = 0;
};
}  // namespace SerialInterface
