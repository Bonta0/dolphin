// Copyright 2009 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/HW/SI/SI_DeviceGBA.h"

#include <cstddef>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include <SFML/Network.hpp>

#include "Common/CommonTypes.h"
#include "Common/Flag.h"
#include "Common/Logging/Log.h"
#include "Common/Swap.h"
#include "Common/Thread.h"
#include "Core/CoreTiming.h"
#include "Core/HW/SI/SI_Device.h"
#include "Core/HW/SI/SI_DeviceGCController.h"
#include "Core/HW/SystemTimers.h"
#include "Core/NetPlayClient.h"

namespace SerialInterface
{
namespace
{
sf::TcpListener s_server;
}  // namespace

enum EJoybusCmds
{
  CMD_RESET = 0xff,
  CMD_STATUS = 0x00,
  CMD_READ = 0x14,
  CMD_WRITE = 0x15
};

constexpr auto BITS_PER_SECOND = 115200;
constexpr auto BYTES_PER_SECOND = BITS_PER_SECOND / 8;
constexpr auto SEND_MAX_SIZE = 5, RECV_MAX_SIZE = 5;

// --- GameBoy Advance "Link Cable" ---

static int GetSendTransferTime(u8 cmd)
{
  u64 bytes_transferred = cmd == CMD_WRITE ? 5 : 1;
  return static_cast<int>(bytes_transferred * SystemTimers::GetTicksPerSecond() / BYTES_PER_SECOND);
}

static int GetReceiveTransferTime(u8 cmd)
{
  u64 bytes_transferred = 0;

  switch (cmd)
  {
  case CMD_RESET:
  case CMD_STATUS:
  {
    bytes_transferred = 3;
    break;
  }
  case CMD_READ:
  {
    bytes_transferred = 5;
    break;
  }
  case CMD_WRITE:
  {
    bytes_transferred = 1;
    break;
  }
  default:
  {
    bytes_transferred = 1;
    break;
  }
  }
  return static_cast<int>(bytes_transferred * SystemTimers::GetTicksPerSecond() / BYTES_PER_SECOND);
}

static void GBAStartListening()
{
  if (s_server.getLocalPort() != 0)
    return;

  // "dolphin gba"
  if (s_server.listen(0xd6ba) != sf::Socket::Done)
    return;

  s_server.setBlocking(false);
}

void GBAConnectionWaiter_Shutdown()
{
  s_server.close();
}

bool GBASockServer::Send(const u8* si_buffer, s32 ticks, s32 waiting, u16 pad)
{
  GBAStartListening();

  if (!m_connected)
  {
    if (!m_disconnected)
    {
      sf::SocketSelector selector;
      selector.add(s_server);
      selector.wait();
    }

    if (s_server.accept(m_client) != sf::Socket::Done)
      return false;

    m_client.setBlocking(false);
    m_connected = true;
  }

  std::array<u8, SEND_MAX_SIZE> data;
  std::memcpy(data.data(), si_buffer, si_buffer[0] == CMD_WRITE ? SEND_MAX_SIZE : 1);

  sf::Packet packet;
  packet << data[0] << data[1] << data[2] << data[3] << data[4] << ticks << waiting << pad;

  if (m_client.send(packet) != sf::Socket::Done)
  {
    m_client.disconnect();
    m_connected = false;
    m_disconnected = true;
  }

  return m_connected;
}

int GBASockServer::Receive(u8* si_buffer)
{
  sf::SocketSelector selector;
  selector.add(m_client);
  selector.wait();

  size_t num_received = 0;
  std::array<u8, RECV_MAX_SIZE + 1> recv_data;
  sf::Socket::Status recv_stat =
      m_client.receive(recv_data.data(), recv_data.size(), num_received);

  if (recv_stat != sf::Socket::Done)
  {
    m_client.disconnect();
    m_connected = false;
    m_disconnected = true;
    return 0;
  }

  if (!recv_data[0])
  {
    return 0;
  }

  std::memcpy(si_buffer, &recv_data[1], num_received - 1);
  return static_cast<int>(num_received);
}

bool GBASockServer::IsConnected() const
{
  return m_connected;
}

CSIDevice_GBA::CSIDevice_GBA(SIDevices device, int device_number) : ISIDevice(device, device_number)
{
}

CSIDevice_GBA::~CSIDevice_GBA()
{
}

int CSIDevice_GBA::RunBuffer(u8* buffer, int request_length)
{
  switch (m_next_action)
  {
  case NextAction::SendCommand:
  {
    const u64 send_time = CoreTiming::GetTicks() + GetSendTransferTime(buffer[0]);

    if (!m_sock_server.IsConnected())
      m_timestamp_sent = CoreTiming::GetTicks();

    const s32 gba_ticks = static_cast<s32>((send_time - m_timestamp_sent) * 16777216 /
                                           SystemTimers::GetTicksPerSecond());

    const s32 gba_wait = static_cast<s32>(static_cast<u64>(GetReceiveTransferTime(buffer[0])) * 16777216 /
                                          SystemTimers::GetTicksPerSecond());

    m_timestamp_sent = send_time;
    m_last_cmd = buffer[0];

    if (!m_sock_server.Send(buffer, gba_ticks, gba_wait, GetGBAPad()))
    {
      u32 reply = Common::swap32(SI_ERROR_NO_RESPONSE);
      std::memcpy(buffer, &reply, sizeof(reply));
      return sizeof(reply);
    }

#ifdef _DEBUG
    NOTICE_LOG(SERIALINTERFACE, "%01d cmd %02x [> %02x%02x%02x%02x]", m_device_number, buffer[0],
               buffer[1], buffer[2], buffer[3], buffer[4]);
#endif
    m_next_action = NextAction::WaitTransferTime;
    [[fallthrough]];
  }

  case NextAction::WaitTransferTime:
  {
    // Tell SI to ask again after TransferInterval() cycles
    if (CoreTiming::GetTicks() < (m_timestamp_sent + GetReceiveTransferTime(m_last_cmd)))
      return 0;
    m_next_action = NextAction::ReceiveResponse;
    [[fallthrough]];
  }

  case NextAction::ReceiveResponse:
  {
    int num_data_received = m_sock_server.Receive(buffer);
    m_next_action = NextAction::SendCommand;
    if (num_data_received == 0)
    {
      u32 reply = Common::swap32(SI_ERROR_NO_RESPONSE);
      std::memcpy(buffer, &reply, sizeof(reply));
      return sizeof(reply);
    }
#ifdef _DEBUG
    const Common::Log::LOG_LEVELS log_level =
        (m_last_cmd == CMD_STATUS || m_last_cmd == CMD_RESET) ? Common::Log::LERROR :
                                                                Common::Log::LWARNING;
    GENERIC_LOG(Common::Log::SERIALINTERFACE, log_level,
                "%01d                              [< %02x%02x%02x%02x%02x] (%i)", m_device_number,
                buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], num_data_received);
#endif
    return num_data_received;
  }
  }

  // This should never happen, but appease MSVC which thinks it might.
  ERROR_LOG(SERIALINTERFACE, "Unknown state %i\n", static_cast<int>(m_next_action));
  return 0;
}

int CSIDevice_GBA::TransferInterval()
{
  return m_timestamp_sent - CoreTiming::GetTicks() + GetReceiveTransferTime(m_last_cmd);
}

bool CSIDevice_GBA::GetData(u32& hi, u32& low)
{
  return false;
}

void CSIDevice_GBA::SendCommand(u32 command, u8 poll)
{
}

u16 CSIDevice_GBA::GetGBAPad()
{
  GCPadStatus pad_status = {};
  if (!NetPlay::IsNetPlayRunning())
    pad_status = Pad::GetStatus(m_device_number);
  else
    SerialInterface::CSIDevice_GCController::NetPlay_GetInput(m_device_number, &pad_status);

  std::map<PadButton, u16> buttons_map = {
      {PadButton::PAD_BUTTON_A, 0x0001},      // A
      {PadButton::PAD_BUTTON_B, 0x0002},      // B
      {PadButton::PAD_TRIGGER_Z, 0x0004},     // Select
      {PadButton::PAD_BUTTON_START, 0x0008},  // Start
      {PadButton::PAD_BUTTON_RIGHT, 0x0010},  // Right
      {PadButton::PAD_BUTTON_LEFT, 0x0020},   // Left
      {PadButton::PAD_BUTTON_UP, 0x0040},     // Up
      {PadButton::PAD_BUTTON_DOWN, 0x0080},   // Down
      {PadButton::PAD_TRIGGER_R, 0x0100},     // R
      {PadButton::PAD_TRIGGER_L, 0x0200},     // L
  };

  u16 pad = 0;
  for (const auto& pair : buttons_map)
  {
    if (pad_status.button & pair.first)
      pad |= pair.second;
  }

  return pad;
}
}  // namespace SerialInterface
