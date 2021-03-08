// Copyright 2021 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <string_view>
#include <vector>

#include <QWidget>

#include "Common/CommonTypes.h"

class QCloseEvent;
class QContextMenuEvent;
class QPaintEvent;

class GBAWidget : public QWidget
{
  Q_OBJECT
public:
  GBAWidget(int device_number, std::string_view current_rom, std::string_view title, u32 width,
            u32 height, QWidget* parent = nullptr, Qt::WindowFlags flags = {});
  ~GBAWidget();

  void SetVideoBuffer(std::vector<u32> video_buffer);
  void UpdateTitle();

  void SetVolume(int volume);
  void VolumeDown();
  void VolumeUp();
  bool IsMuted();
  void ToggleMute();

  void LoadROM();
  void UnloadROM();
  void ResetCore();
  void Resize(int scale);

private:
  bool CanResetCore();
  void SendReset(std::string_view rom_path);

  void closeEvent(QCloseEvent* event) override;
  void contextMenuEvent(QContextMenuEvent* event) override;
  void paintEvent(QPaintEvent* event) override;

  std::vector<u32> m_video_buffer;
  int m_device_number;
  int m_geometry_slot;
  std::string m_current_rom;
  std::string m_netplayer_name;
  std::string m_game_title;
  u32 m_width;
  u32 m_height;
  int m_volume;
  bool m_muted;
};

void EnableGBAFrontend();
