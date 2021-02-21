// Copyright 2021 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <array>
#include <memory>
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
  Core(int device_number);
  ~Core();

  void Init(u64 gc_ticks);
  void Deinit();

  void SendJoybusCommand(u64 gc_ticks, u8* buffer);
  std::vector<u8> GetJoybusResponse();
  void Flush();

  void DoState(PointerWrap& p);

private:
  void RunUntil(u64 gc_ticks);
  void RunFor(u64 gc_ticks);
  void OnKeysRead();
  u16 GetPadStatus();

  int m_device_number;

  mCore* m_core{};
  mTimingEvent m_event{};
  SIODriver m_sio_driver{};
  AVStream m_stream{};
  std::vector<u32> m_video_buffer;
  std::array<u8, 6> m_command_buffer;

  u64 m_last_gc_ticks{};
  u64 m_gc_ticks_remainder{};
  bool m_link_enabled{};

  std::unique_ptr<FrontendInterface> m_frontend;
};
}  // namespace HW::GBA
