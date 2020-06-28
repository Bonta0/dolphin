// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <array>
#include <memory>
#include <vector>

#include <SFML/Network.hpp>

#include "Common/CommonTypes.h"
#include "Core/HW/SI/SI_Device.h"
#include "Core/NetPlayProto.h"

// GameBoy Advance "Link Cable"

namespace SerialInterface
{
void GBAConnectionWaiter_Shutdown();

class GBASockServer
{
public:
  GBASockServer(int device_number) : m_device_number(device_number) {}
  ~GBASockServer() = default;

  bool Send(const u8* si_buffer, u64 ticks, u16 pad, bool movie = false);
  bool Receive(NetPlay::GBAStatus* status);
  void SendMovieUpdate(const u8* si_buffer, u64 ticks, u16 pad);

  void Disconnect();
  bool IsConnected() const;

private:
  sf::TcpSocket m_client;

  int m_device_number = 0;
  bool m_disconnected = false;

  struct MovieUpdate
  {
    std::vector<u8> data;
    u64 ticks;
    u16 pad;
  };
  std::vector<MovieUpdate> m_movie_queue;
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

  bool NetPlay_GetData(NetPlay::GBAStatus* status);
  bool NetPlay_SendData(int pad_num, NetPlay::GBAStatus* status);
  bool NetPlay_IsClient();

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
  NetPlay::GBAStatus m_last_status{};

  u16 m_last_padstatus = 0;
  u64 m_next_pad_update = 0;
};
}  // namespace SerialInterface
