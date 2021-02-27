// Copyright 2021 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "DolphinQt/Config/Mapping/HotkeyGBA.h"

#include <QGroupBox>
#include <QHBoxLayout>

#include "Core/HotkeyManager.h"

HotkeyGBA::HotkeyGBA(MappingWindow* window) : MappingWidget(window)
{
  CreateMainLayout();
}

void HotkeyGBA::CreateMainLayout()
{
  m_main_layout = new QHBoxLayout();

  m_main_layout->addWidget(
      CreateGroupBox(tr("GameBoy Advance"), HotkeyManagerEmu::GetHotkeyGroup(HKGP_GBA)));

  setLayout(m_main_layout);
}

InputConfig* HotkeyGBA::GetConfig()
{
  return HotkeyManagerEmu::GetConfig();
}

void HotkeyGBA::LoadSettings()
{
  HotkeyManagerEmu::LoadConfig();
}

void HotkeyGBA::SaveSettings()
{
  HotkeyManagerEmu::GetConfig()->SaveConfig();
}
