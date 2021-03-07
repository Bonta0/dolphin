// Copyright 2021 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "DolphinQt/GBAWidget.h"

#include <algorithm>
#include <memory>

#include <fmt/format.h>

#include <QAction>
#include <QApplication>
#include <QCloseEvent>
#include <QContextMenuEvent>
#include <QFileDialog>
#include <QIcon>
#include <QImage>
#include <QMenu>
#include <QPainter>

#include "AudioCommon/AudioCommon.h"
#include "Core/Core.h"
#include "Core/HW/GBAFrontend.h"
#include "Core/HW/GBAPad.h"
#include "Core/HW/SI/SI.h"
#include "Core/Movie.h"
#include "Core/NetPlayClient.h"
#include "DolphinQt/Resources.h"
#include "DolphinQt/Settings.h"

GBAWidget::GBAWidget(int device_number, std::string_view current_rom, std::string_view title,
                     u32 width, u32 height, QWidget* parent, Qt::WindowFlags flags)
    : QWidget(parent, flags), m_device_number(device_number), m_current_rom(current_rom),
      m_game_title(title), m_width(width), m_height(height), m_volume(0), m_muted(false)
{
  setWindowIcon(Resources::GetAppIcon());
  resize(width, height);
  show();

  QSettings& settings = Settings::GetQSettings();
  auto key = QString::fromStdString(fmt::format("gbawidget/geometry{}", m_device_number + 1));
  if (settings.contains(key))
    restoreGeometry(settings.value(key).toByteArray());
  else
    move(static_cast<int>(x() - frameGeometry().width() / (device_number & 1 ? -2.f : 2.f)),
         static_cast<int>(y() - frameGeometry().height() / (device_number & 2 ? -2.f : 2.f)));

  SetVolume(100);

  if (NetPlay::IsNetPlayRunning())
  {
    auto client = Settings::Instance().GetNetPlayClient();
    auto pid = client->GetPadMapping()[device_number];
    for (const auto& player : client->GetPlayers())
    {
      if (player->pid == pid)
        m_netplayer_name = player->name;
    }
    if (!client->IsLocalPlayer(pid) && !IsMuted())
      ToggleMute();
  }

  UpdateTitle();
}

GBAWidget::~GBAWidget()
{
  QSettings& settings = Settings::GetQSettings();
  auto key = QString::fromStdString(fmt::format("gbawidget/geometry{}", m_device_number + 1));
  settings.setValue(key, saveGeometry());
}

void GBAWidget::SetVideoBuffer(std::vector<u32> video_buffer)
{
  m_video_buffer = std::move(video_buffer);
  update();
}

void GBAWidget::UpdateTitle()
{
  std::string title = fmt::format("GBA{}", m_device_number + 1);
  if (!m_netplayer_name.empty())
    title += " " + m_netplayer_name;

  title += " | " + m_game_title;

  if (m_muted)
    title += " | Muted";
  else
    title += fmt::format(" | Volume {}%", m_volume);

  setWindowTitle(QString::fromStdString(title));
}

void GBAWidget::SetVolume(int volume)
{
  m_muted = false;
  m_volume = std::clamp(volume, 0, 100);

  g_sound_stream->GetMixer()->SetGBAVolume(m_device_number, m_volume * 256 / 100,
                                           m_volume * 256 / 100);

  UpdateTitle();
}

void GBAWidget::VolumeDown()
{
  SetVolume(m_volume - 10);
}

void GBAWidget::VolumeUp()
{
  SetVolume(m_volume + 10);
}

bool GBAWidget::IsMuted()
{
  return m_muted;
}

void GBAWidget::ToggleMute()
{
  m_muted = !m_muted;

  g_sound_stream->GetMixer()->SetGBAVolume(m_device_number, 0, 0);

  UpdateTitle();
}

void GBAWidget::LoadROM()
{
  if (!CanResetCore())
    return;

  QString rom_file = QDir::toNativeSeparators(QFileDialog::getOpenFileName(
      this, tr("Select the ROM File"),
      QString::fromStdString(!m_current_rom.empty() ? m_current_rom :
                                                      Pad::GetGBARomPath(m_device_number)),
      tr("Game Boy Advance ROMs (*.gba *.7z *.zip *.agb *.mb *.rom *.bin)")));

  if (!rom_file.isEmpty())
    SendReset(rom_file.toStdString());
}

void GBAWidget::UnloadROM()
{
  if (m_current_rom.empty())
    return;

  SendReset("");
}

void GBAWidget::ResetCore()
{
  SendReset(m_current_rom);
}

void GBAWidget::Resize(int scale)
{
  resize(m_width * scale, m_height * scale);
}

bool GBAWidget::CanResetCore()
{
  return !Movie::IsPlayingInput() && !Movie::IsRecordingInput() && !NetPlay::IsNetPlayRunning();
}

void GBAWidget::SendReset(std::string_view rom_path)
{
  if (!CanResetCore())
    return;

  Core::RunOnCPUThread(
      [device_number = m_device_number, rom_path{std::string(rom_path)}] {
        std::string rom_backup = Pad::GetGBARomPath(device_number);
        Pad::SetGBARomPath(device_number, rom_path);
        SerialInterface::ResetGBACore(device_number);
        Pad::SetGBARomPath(device_number, rom_backup);
      },
      false);
}

void GBAWidget::closeEvent(QCloseEvent* event)
{
  event->ignore();
}

void GBAWidget::contextMenuEvent(QContextMenuEvent* event)
{
  QMenu* menu = new QMenu(this);
  connect(menu, &QMenu::triggered, menu, &QMenu::deleteLater);

  QAction* load_action = new QAction(tr("L&oad ROM"), menu);
  load_action->setEnabled(CanResetCore());
  connect(load_action, &QAction::triggered, this, &GBAWidget::LoadROM);

  QAction* unload_action = new QAction(tr("&Unload ROM"), menu);
  unload_action->setEnabled(CanResetCore() && !m_current_rom.empty());
  connect(unload_action, &QAction::triggered, this, &GBAWidget::UnloadROM);

  QAction* reset_action = new QAction(tr("&Reset"), menu);
  reset_action->setEnabled(CanResetCore());
  connect(reset_action, &QAction::triggered, this, &GBAWidget::ResetCore);

  QAction* mute_action = new QAction(tr("&Mute"), menu);
  mute_action->setCheckable(true);
  mute_action->setChecked(m_muted);
  connect(mute_action, &QAction::triggered, this, &GBAWidget::ToggleMute);

  QMenu* size_menu = new QMenu(tr("Window Size"), menu);

  QAction* x1_action = new QAction(tr("&1x"), size_menu);
  connect(x1_action, &QAction::triggered, this, [this] { Resize(1); });
  QAction* x2_action = new QAction(tr("&2x"), size_menu);
  connect(x2_action, &QAction::triggered, this, [this] { Resize(2); });
  QAction* x3_action = new QAction(tr("&3x"), size_menu);
  connect(x3_action, &QAction::triggered, this, [this] { Resize(3); });
  QAction* x4_action = new QAction(tr("&4x"), size_menu);
  connect(x4_action, &QAction::triggered, this, [this] { Resize(4); });

  size_menu->addAction(x1_action);
  size_menu->addAction(x2_action);
  size_menu->addAction(x3_action);
  size_menu->addAction(x4_action);

  menu->addAction(load_action);
  menu->addAction(unload_action);
  menu->addAction(reset_action);
  menu->addSeparator();
  menu->addAction(mute_action);
  menu->addSeparator();
  menu->addMenu(size_menu);

  menu->move(event->globalPos());
  menu->show();
}

void GBAWidget::paintEvent(QPaintEvent* event)
{
  QPainter painter(this);
  painter.fillRect(QRect(QPoint(), size()), Qt::black);

  if (m_video_buffer.size() == m_width * m_height)
  {
    QImage image(reinterpret_cast<const uchar*>(m_video_buffer.data()), m_width, m_height,
                 QImage::Format_ARGB32);
    image = image.convertToFormat(QImage::Format_RGB32);
    image = image.rgbSwapped();

    QSize widget_size = size();
    if (widget_size == QSize(m_width, m_height))
    {
      painter.drawImage(QPoint(), image, QRect(0, 0, m_width, m_height));
    }
    else if (static_cast<float>(m_width) / m_height >
             static_cast<float>(widget_size.width()) / widget_size.height())
    {
      u32 new_height = widget_size.width() * m_height / m_width;
      painter.drawImage(
          QRect(0, (widget_size.height() - new_height) / 2, widget_size.width(), new_height), image,
          QRect(0, 0, m_width, m_height));
    }
    else
    {
      u32 new_width = widget_size.height() * m_width / m_height;
      painter.drawImage(
          QRect((widget_size.width() - new_width) / 2, 0, new_width, widget_size.height()), image,
          QRect(0, 0, m_width, m_height));
    }
  }
}

class GBAFrontend : public HW::GBA::FrontendInterface
{
public:
  GBAFrontend(int device_number, const char* title, u32 width, u32 height,
              QWidget* parent = nullptr, Qt::WindowFlags flags = {})
  {
    m_widget = std::make_shared<GBAWidget*>();
    QMetaObject::invokeMethod(qApp, [=, widget{m_widget},
                                     current_rom{Pad::GetGBARomPath(device_number)},
                                     game_title{std::string(title)}] {
      *widget = new GBAWidget(device_number, current_rom, game_title, width, height, parent, flags);
    });
  }

  ~GBAFrontend() {}

  void FrameEnded(const std::vector<u32>& video_buffer) override
  {
    QMetaObject::invokeMethod(qApp, [widget{m_widget}, buffer{video_buffer}]() mutable {
      (*widget)->SetVideoBuffer(std::move(buffer));
    });
  }

  void Stop() override
  {
    QMetaObject::invokeMethod(qApp, [widget{m_widget}] { (*widget)->deleteLater(); });
  }

private:
  std::shared_ptr<GBAWidget*> m_widget;
};

void EnableGBAFrontend()
{
  HW::GBA::SetFrontendFactory([](int device_number, const char* title, u32 width,
                                 u32 height) -> std::unique_ptr<HW::GBA::FrontendInterface> {
    return std::make_unique<GBAFrontend>(device_number, title, width, height);
  });
}
