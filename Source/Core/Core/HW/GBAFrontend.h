// Copyright 2021 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <memory>
#include <vector>

#include "Common/CommonTypes.h"

class PointerWrap;

namespace HW::GBA
{
class FrontendInterface
{
public:
  FrontendInterface() = default;
  virtual ~FrontendInterface() = default;

  virtual void FrameEnded(const std::vector<u32>& video_buffer){};
  virtual void Stop(){};
};

extern std::unique_ptr<FrontendInterface> (*s_create_frontend)(int device_number, const char* title,
                                                               u32 width, u32 height);
}  // namespace HW::GBA
