// Copyright 2021 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <string_view>

#include "InputCommon/ControllerEmu/ControlGroup/ControlGroup.h"
#include "InputCommon/ControllerEmu/ControllerEmu.h"

struct GCPadStatus;

namespace ControllerEmu
{
class Buttons;
}  // namespace ControllerEmu

enum class GBAPadGroup
{
  DPad,
  Buttons,
  ROM
};

class GBAPad : public ControllerEmu::EmulatedController
{
public:
  explicit GBAPad(unsigned int index);
  GCPadStatus GetInput() const;

  std::string GetName() const override;

  ControllerEmu::ControlGroup* GetGroup(GBAPadGroup group);

  void LoadDefaults(const ControllerInterface& ciface) override;

  std::string GetRomPath();
  void SetRomPath(std::string_view path);

private:
  ControllerEmu::Buttons* m_buttons;
  ControllerEmu::Buttons* m_dpad;
  ControllerEmu::ControlGroup* m_rom;

  const unsigned int m_index;
};
