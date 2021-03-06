// Copyright 2018 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <QWidget>

class QCheckBox;
class QComboBox;
class QLineEdit;
class QPushButton;

class GameCubePane : public QWidget
{
  Q_OBJECT
public:
  explicit GameCubePane();

private:
  void CreateWidgets();
  void ConnectWidgets();

  void LoadSettings();
  void SaveSettings();

  void UpdateButton(int slot);
  void OnConfigPressed(int slot);

  void BrowseGBABios();
  void SaveRomPathChanged();
  void BrowseGBASaves();

  QCheckBox* m_skip_main_menu;
  QComboBox* m_language_combo;

  QPushButton* m_slot_buttons[3];
  QComboBox* m_slot_combos[3];

  QCheckBox* m_gba_threads;
  QCheckBox* m_gba_save_rom_path;
  QPushButton* m_gba_browse_bios;
  QLineEdit* m_gba_bios_edit;
  QPushButton* m_gba_browse_saves;
  QLineEdit* m_gba_saves_edit;
};
