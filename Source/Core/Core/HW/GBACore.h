// Copyright 2021 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <array>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#define PYCPARSE  // Remove static functions from the header
#include <mgba/core/interface.h>
#undef PYCPARSE
#include <mgba/core/core.h>
#include <mgba/gba/interface.h>

#include "Common/CommonTypes.h"

namespace HW::GBA
{
static constexpr auto BITS_PER_SECOND = 115200;
static constexpr auto BYTES_PER_SECOND = BITS_PER_SECOND / 8;

class FrontendInterface;

class Core;
struct SIODriver : GBASIODriver
{
  Core* core;
};
struct AVStream : mAVStream
{
  Core* core;
};

class Core
{
public:
  Core(int device_number, u64 gc_ticks);
  ~Core();

  void SendJoybusCommand(u64 gc_ticks, u8* buffer);
  std::vector<u8> GetJoybusResponse();

  void DoState(PointerWrap& p);

private:
  void ThreadLoop();
  void RunUntil(u64 gc_ticks);
  void RunFor(u64 gc_ticks);
  void Flush();

  struct Command
  {
    u64 ticks;
    bool sync_only;
    std::array<u8, 6> buffer;
  };
  void RunCommand(Command& command);

  void LoadBIOS();
  void LoadROM();
  void LoadSave();
  void SetSIODriver();
  void SetCallbacks();
  void SetAVStream();
  void SetupEvent();

  void OnKeysRead();
  u16 GetPadStatus();

  int m_device_number;
  std::string m_rom_path;

  mCore* m_core{};
  mTimingEvent m_event{};
  SIODriver m_sio_driver{};
  AVStream m_stream{};
  std::vector<u32> m_video_buffer;

  u64 m_last_gc_ticks{};
  u64 m_gc_ticks_remainder{};
  bool m_link_enabled{};

  std::unique_ptr<FrontendInterface> m_frontend;

  std::unique_ptr<std::thread> m_thread;
  bool m_exit_loop{};
  bool m_idle{};
  std::mutex m_queue_mutex;
  std::condition_variable m_command_cv;
  std::queue<Command> m_command_queue;

  std::mutex m_response_mutex;
  std::condition_variable m_response_cv;
  bool m_response_ready{};
  std::vector<u8> m_response;
};
}  // namespace HW::GBA
