// Copyright 2021 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "DolphinQt/Config/Mapping/GBAPadEmu.h"

#include <QFileDialog>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QToolButton>

#include "Core/HW/GBAPad.h"
#include "Core/HW/GBAPadEmu.h"
#include "DolphinQt/Config/Mapping/MappingWindow.h"
#include "InputCommon/ControllerEmu/Setting/NumericSetting.h"
#include "InputCommon/InputConfig.h"

GBAPadEmu::GBAPadEmu(MappingWindow* window) : MappingWidget(window)
{
  CreateMainLayout();
}

void GBAPadEmu::CreateMainLayout()
{
  auto* layout = new QGridLayout;

  auto* rom_box = new QGroupBox(tr("ROM"));
  auto* rom_layout = new QHBoxLayout();
  rom_box->setLayout(rom_layout);

  m_rom_edit = new QLineEdit();
  rom_layout->addWidget(m_rom_edit);

  QToolButton* rom_button = new QToolButton();
  rom_button->setText(QStringLiteral("..."));
  rom_layout->addWidget(rom_button);

  rom_box->setFixedHeight(rom_box->minimumSizeHint().height());

  layout->addWidget(rom_box, 0, 0);
  layout->addWidget(
      CreateControlsBox(tr("D-Pad"), Pad::GetGBAGroup(GetPort(), GBAPadGroup::DPad), 2), 1, 0, -1, 1);
  layout->addWidget(
      CreateControlsBox(tr("Buttons"), Pad::GetGBAGroup(GetPort(), GBAPadGroup::Buttons), 2), 0, 1, -1, 1);

  setLayout(layout);

  connect(m_rom_edit, &QLineEdit::editingFinished, this, &GBAPadEmu::RomChanged);
  connect(rom_button, &QToolButton::pressed, this, &GBAPadEmu::BrowseRom);
  connect(this, &MappingWidget::ConfigChanged, this, &GBAPadEmu::ConfigChanged);
}

void GBAPadEmu::RomChanged()
{
  Pad::SetGBARomPath(GetPort(), m_rom_edit->text().toStdString());
  SaveSettings();
}

void GBAPadEmu::BrowseRom()
{
  QString rom_file = QDir::toNativeSeparators(QFileDialog::getOpenFileName(
      this, tr("Select the ROM File"), QString::fromStdString(Pad::GetGBARomPath(GetPort())),
      tr("Game Boy Advance ROMs (*.gba *.7z *.zip *.agb *.mb *.rom *.bin)")));

  if (rom_file.isEmpty())
    return;

  Pad::SetGBARomPath(GetPort(), rom_file.toStdString());
  ConfigChanged();
}

void GBAPadEmu::LoadSettings()
{
  Pad::LoadGBAConfig();
}

void GBAPadEmu::SaveSettings()
{
  Pad::GetGBAConfig()->SaveConfig();
}

void GBAPadEmu::ConfigChanged()
{
  m_rom_edit->setText(QString::fromStdString(Pad::GetGBARomPath(GetPort())));
}

InputConfig* GBAPadEmu::GetConfig()
{
  return Pad::GetGBAConfig();
}
