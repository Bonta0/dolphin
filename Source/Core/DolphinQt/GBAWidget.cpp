// Copyright 2021 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include <memory>

#include <fmt/format.h>

#include <QApplication>
#include <QCloseEvent>
#include <QIcon>
#include <QImage>
#include <QKeyEvent>
#include <QPainter>

#include "AudioCommon/AudioCommon.h"
#include "Core/HW/GBAFrontend.h"
#include "Core/NetPlayClient.h"
#include "DolphinQt/GBAWidget.h"
#include "DolphinQt/Resources.h"
#include "DolphinQt/Settings.h"

GBAWidget::GBAWidget(int device_number, std::string title, u32 width, u32 height, QWidget* parent,
                     Qt::WindowFlags flags)
    : QWidget(parent, flags), m_device_number(device_number), m_title(title), m_width(width), m_height(height),
      m_volume(100), m_muted(false)
{
  if (NetPlay::IsNetPlayRunning())
  {
    auto client = Settings::Instance().GetNetPlayClient();
    m_muted = !client->IsLocalPlayer(client->GetPadMapping()[device_number]);
  }
  setWindowIcon(Resources::GetAppIcon());
  resize(width, height);
  show();
  move(static_cast<int>(x() - frameGeometry().width() / (device_number & 1 ? -2.f : 2.f)),
       static_cast<int>(y() - frameGeometry().height() / (device_number & 2 ? -2.f : 2.f)));
  UpdateTitle();
  UpdateVolume();
}

void GBAWidget::SetVideoBuffer(std::vector<u32> video_buffer)
{
  m_video_buffer = std::move(video_buffer);
  update();
}

void GBAWidget::UpdateTitle()
{
  std::string title = fmt::format("GBA{} {} {}", m_device_number + 1, m_title,
                                  m_muted ? "Muted" : fmt::format("Volume {}%", m_volume));
  setWindowTitle(QString::fromStdString(title));
}

void GBAWidget::UpdateVolume()
{
  unsigned int volume = static_cast<unsigned int>(m_muted ? 0 : m_volume * 256 / 100);
  g_sound_stream->GetMixer()->SetGBAVolume(m_device_number, volume, volume);
}

void GBAWidget::closeEvent(QCloseEvent* event)
{
  event->ignore();
}

// TODO: proper config and connections
void GBAWidget::keyPressEvent(QKeyEvent* event)
{
  int scale = 0;
  if (event->key() == Qt::Key_1)
    scale = 1;
  if (event->key() == Qt::Key_2)
    scale = 2;
  if (event->key() == Qt::Key_3)
    scale = 3;
  if (event->key() == Qt::Key_4)
    scale = 4;
  if (scale && event->modifiers() & Qt::KeyboardModifier::KeypadModifier)
    resize(m_width * scale, m_height * scale);

  if (event->key() == Qt::Key_M)
  {
    m_muted = !m_muted;
    UpdateVolume();
    UpdateTitle();
  }

  int volume_change = 0;
  if (event->key() == Qt::Key_Plus && event->modifiers() & Qt::KeyboardModifier::KeypadModifier)
    volume_change = 2;
  if (event->key() == Qt::Key_Minus && event->modifiers() & Qt::KeyboardModifier::KeypadModifier)
    volume_change = -2;
  if (volume_change)
  {
    m_muted = false;
    m_volume = std::clamp(m_volume + volume_change, 0, 100);
    UpdateVolume();
    UpdateTitle();
  }
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
    std::string game_title(title);
    m_widget = std::make_shared<GBAWidget*>();
    QMetaObject::invokeMethod(qApp, [=, widget{m_widget}] {
      *widget = new GBAWidget(device_number, game_title, width, height, parent, flags);
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

static std::unique_ptr<HW::GBA::FrontendInterface>
CreateGBAFrontend(int device_number, const char* title, u32 width, u32 height)
{
  return std::make_unique<GBAFrontend>(device_number, title, width, height);
}

void EnableGBAFrontend()
{
  HW::GBA::s_create_frontend = CreateGBAFrontend;
}
