// Copyright 2021 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <vector>

#include <QWidget>

#include "Common/CommonTypes.h"

class GBAWidget : public QWidget
{
  Q_OBJECT
public:
  GBAWidget(int device_number, u32 width, u32 height, QWidget* parent = nullptr,
            Qt::WindowFlags flags = {});

  void SetVideoBuffer(std::vector<u32> video_buffer);
  void UpdateTitle();
  void UpdateVolume();

private:
  void closeEvent(QCloseEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void paintEvent(QPaintEvent* event) override;

  std::vector<u32> m_video_buffer;
  int m_device_number;
  u32 m_width;
  u32 m_height;
  int m_volume;
  bool m_muted;
};

void EnableGBAFrontend();
