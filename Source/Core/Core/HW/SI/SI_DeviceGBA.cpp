// Copyright 2009 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/HW/SI/SI_DeviceGBA.h"

#include <array>
#include <cstddef>
#include <cstring>
#include <memory>

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

std::array<bool, 4> is_gba{};
std::array<bool, 4> gba_connected{};

// --- GameBoy Advance "Link Cable" ---

static int GetSendTransferTime(u8 cmd)
{
  return 0;
  /*u64 bytes_transferred = cmd == CMD_WRITE ? 5 : 1;
  return static_cast<int>(bytes_transferred * SystemTimers::GetTicksPerSecond() / BYTES_PER_SECOND);*/
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
  s_server.listen(0xd6ba);

  s_server.setBlocking(false);
}

void GBAConnectionWaiter_Shutdown()
{
  s_server.close();
}

static void AddToPacket(sf::Packet& packet, const u8* si_buffer, u64 ticks, u16 pad)
{
  std::array<u8, SEND_MAX_SIZE> data;
  std::memcpy(data.data(), si_buffer, si_buffer[0] == CMD_WRITE ? SEND_MAX_SIZE : 1);

  packet << data[0] << data[1] << data[2] << data[3] << data[4] << ticks << pad;
}

bool GBASockServer::Send(const u8* si_buffer, u64 ticks, u16 pad, bool movie)
{
  GBAStartListening();

  sf::Packet packet;

  if (!gba_connected[m_device_number])
  {
    if (!movie && !m_disconnected)
    {
      sf::SocketSelector selector;
      selector.add(s_server);
      selector.wait();
    }

    if (s_server.accept(m_client) != sf::Socket::Done)
      return false;

    m_client.setBlocking(false);
    gba_connected[m_device_number] = true;

    packet << movie;
    if (movie)
    {
      for (auto& update : m_movie_queue)
        AddToPacket(packet, update.data.data(), update.ticks, update.pad);
      m_movie_queue.clear();
    }
  }

  AddToPacket(packet, si_buffer, ticks, pad);

  if (m_client.send(packet) != sf::Socket::Done)
    Disconnect();

  return gba_connected[m_device_number];
}

bool GBASockServer::Receive(NetPlay::GBAStatus* status)
{
  sf::SocketSelector selector;
  selector.add(m_client);
  selector.wait();

  sf::Packet packet;
  if (m_client.receive(packet) != sf::Socket::Done)
  {
    Disconnect();
    return false;
  }

  u8 recv_len = 0;
  packet >> recv_len;

  status->resp.resize(recv_len);
  for (u8 i = 0; i < recv_len; ++i)
    packet >> status->resp[i];

  packet >> status->ticks;

  return true;
}

void GBASockServer::SendMovieUpdate(const u8* si_buffer, u64 ticks, u16 pad)
{
  if (m_movie_queue.size() > 2 * 1024 * 1024)
    Disconnect();
  if (m_disconnected)
    return;

  int next_device = -1;
  int next_local_device = -1;
  if (gba_connected[m_device_number])
    next_local_device = m_device_number;
  else
  {
    for (int i = 0; i < 4; ++i)
    {
      if (!is_gba[i] || gba_connected[i])
        continue;
      if (CSIDevice_GCController::NetPlay_InGamePadToLocalPad(i) != 4)
      {
        next_local_device = i;
        break;
      }
      if (next_device == -1)
        next_device = i;
    }
  }

  bool dont_send = next_local_device != m_device_number &&
                   (next_local_device != -1 || next_device != m_device_number);
  if (dont_send || (!Send(si_buffer, ticks, pad, true) && !m_disconnected))
  {
    MovieUpdate update;
    update.data.resize(si_buffer[0] == CMD_WRITE ? SEND_MAX_SIZE : 1);
    std::memcpy(update.data.data(), si_buffer, update.data.size());
    update.ticks = ticks;
    update.pad = pad;
    m_movie_queue.emplace_back(std::move(update));
  }
}

void GBASockServer::Disconnect()
{
  m_client.disconnect();
  gba_connected[m_device_number] = false;
  m_movie_queue.clear();
  m_disconnected = true;
}

bool GBASockServer::IsConnected() const
{
  return gba_connected[m_device_number];
}

CSIDevice_GBA::CSIDevice_GBA(SIDevices device, int device_number)
    : ISIDevice(device, device_number), m_sock_server(device_number)
{
  is_gba[m_device_number] = true;
  gba_connected[m_device_number] = false;
}

CSIDevice_GBA::~CSIDevice_GBA()
{
  is_gba[m_device_number] = false;
  gba_connected[m_device_number] = false;
}

int CSIDevice_GBA::RunBuffer(u8* buffer, int request_length)
{
  bool is_netplay_client = NetPlay_IsClient();

  switch (m_next_action)
  {
  case NextAction::SendCommand:
  {
    const u16 gba_pad_status = GetGBAPad();

    const u64 send_time = CoreTiming::GetTicks() + GetSendTransferTime(buffer[0]);

    if (!is_netplay_client && !m_sock_server.IsConnected())
      m_timestamp_sent = CoreTiming::GetTicks();

    const s32 gba_ticks = static_cast<s32>((send_time - m_timestamp_sent) * 16777216 /
                                           SystemTimers::GetTicksPerSecond());

    const s32 gba_wait = static_cast<s32>(static_cast<u64>(GetReceiveTransferTime(buffer[0])) * 16777216 /
                                          SystemTimers::GetTicksPerSecond());

    m_timestamp_sent = send_time;
    m_last_cmd = buffer[0];

    bool error = false;
    if (is_netplay_client)
    {
      if (NetPlay_GetData(&m_last_status))
      {
        error = !m_last_status.sent;
        if (!error)
          m_sock_server.SendMovieUpdate(buffer, m_last_status.ticks, gba_pad_status);
      }
      else
        error = true;
    }
    else
    {
      error = !m_sock_server.Send(buffer, gba_ticks, gba_pad_status);
      m_last_status.sent = !error;
      if (!m_last_status.sent)
        NetPlay_SendData(m_device_number, &m_last_status);
    }

    if (error)
    {
      u32 reply = Common::swap32(SI_ERROR_NO_RESPONSE);
      std::memcpy(buffer, &reply, sizeof(reply));
      return sizeof(reply);
    }

//#ifdef _DEBUG
    NOTICE_LOG(SERIALINTERFACE, "%01d cmd %02x [> %02x%02x%02x%02x]", m_device_number, buffer[0],
               buffer[1], buffer[2], buffer[3], buffer[4]);
//#endif
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
    int num_data_received = 0;
    if (is_netplay_client)
      num_data_received = static_cast<int>(m_last_status.resp.size());
    else
    {
      if (!m_sock_server.Receive(&m_last_status))
        num_data_received = 0;
      else
      {
        num_data_received = static_cast<int>(m_last_status.resp.size());
        NetPlay_SendData(m_device_number, &m_last_status);
      }
    }
    std::memcpy(buffer, m_last_status.resp.data(), num_data_received);

    m_next_action = NextAction::SendCommand;
    if (num_data_received == 0)
    {
      u32 reply = Common::swap32(SI_ERROR_NO_RESPONSE);
      std::memcpy(buffer, &reply, sizeof(reply));
      return sizeof(reply);
    }
//#ifdef _DEBUG
    const Common::Log::LOG_LEVELS log_level =
        (m_last_cmd == CMD_STATUS || m_last_cmd == CMD_RESET) ? Common::Log::LERROR :
                                                                Common::Log::LWARNING;
    GENERIC_LOG(Common::Log::SERIALINTERFACE, log_level,
                "%01d                              [< %02x%02x%02x%02x%02x] (%i)", m_device_number,
                buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], num_data_received);
//#endif
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
  if (static_cast<int64_t>(m_next_pad_update - CoreTiming::GetTicks()) > 0)
    return m_last_padstatus;

  GCPadStatus pad_status = {};
  if (!NetPlay::IsNetPlayRunning())
    pad_status = Pad::GetStatus(m_device_number);
  else
    CSIDevice_GCController::NetPlay_GetInput(m_device_number, &pad_status);

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

  m_last_padstatus = 0;
  for (size_t i = 0; i < buttons_map.size(); ++i)
    m_last_padstatus |=
        static_cast<std::uint16_t>(static_cast<bool>((pad_status.button & buttons_map[i]))) << i;

  m_next_pad_update = CoreTiming::GetTicks() + (SystemTimers::GetTicksPerSecond() / 500);
  return m_last_padstatus;
}
}  // namespace SerialInterface
