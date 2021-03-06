// Copyright 2021 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Core/HW/GBAPad.h"

#include "Common/Common.h"
#include "Core/HW/GBAPadEmu.h"
#include "InputCommon/ControllerEmu/ControlGroup/ControlGroup.h"
#include "InputCommon/ControllerInterface/ControllerInterface.h"
#include "InputCommon/GCPadStatus.h"
#include "InputCommon/InputConfig.h"

namespace Pad
{
static InputConfig s_config("GBA", _trans("Pad"), "GBA");
InputConfig* GetGBAConfig()
{
  return &s_config;
}

void ShutdownGBA()
{
  s_config.UnregisterHotplugCallback();

  s_config.ClearControllers();
}

void InitializeGBA()
{
  if (s_config.ControllersNeedToBeCreated())
  {
    for (unsigned int i = 0; i < 4; ++i)
      s_config.CreateController<GBAPad>(i);
  }

  s_config.RegisterHotplugCallback();

  // Load the saved controller config
  s_config.LoadConfig(InputConfig::InputType::GBA);
}

void LoadGBAConfig()
{
  s_config.LoadConfig(InputConfig::InputType::GBA);
}

bool IsGBAInitialized()
{
  return !s_config.ControllersNeedToBeCreated();
}

GCPadStatus GetGBAStatus(int pad_num)
{
  return static_cast<GBAPad*>(s_config.GetController(pad_num))->GetInput();
}

std::string GetGBARomPath(int pad_num)
{
  return static_cast<GBAPad*>(s_config.GetController(pad_num))->GetRomPath();
}

void SetGBARomPath(int pad_num, std::string_view path)
{
  static_cast<GBAPad*>(s_config.GetController(pad_num))->SetRomPath(path);
}

ControllerEmu::ControlGroup* GetGBAGroup(int pad_num, GBAPadGroup group)
{
  return static_cast<GBAPad*>(s_config.GetController(pad_num))->GetGroup(group);
}
}  // namespace Pad
