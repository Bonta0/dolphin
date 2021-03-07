// Copyright 2021 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include "DolphinQt/Config/Mapping/MappingWidget.h"

class QLineEdit;

class GBAPadEmu final : public MappingWidget
{
  Q_OBJECT
public:
  explicit GBAPadEmu(MappingWindow* window);

  InputConfig* GetConfig() override;

private:
  void RomChanged();
  void BrowseRom();
  void LoadSettings() override;
  void SaveSettings() override;
  void CreateMainLayout();
  void ConfigChanged();

  // Main
  QLineEdit* m_rom_edit;
};
