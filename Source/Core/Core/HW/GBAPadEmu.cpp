// Copyright 2021 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include <mutex>

#include "Common/Common.h"
#include "Common/CommonTypes.h"
#include "Common/IniFile.h"
#include "Core/HW/GBAPadEmu.h"
#include "InputCommon/ControllerEmu/Control/Input.h"
#include "InputCommon/ControllerEmu/ControlGroup/Buttons.h"
#include "InputCommon/GCPadStatus.h"

static const u16 dpad_bitmasks[] = {PAD_BUTTON_UP, PAD_BUTTON_DOWN, PAD_BUTTON_LEFT,
                                    PAD_BUTTON_RIGHT};

static const u16 button_bitmasks[] = {PAD_BUTTON_B,  PAD_BUTTON_A,  PAD_TRIGGER_L,
                                      PAD_TRIGGER_R, PAD_TRIGGER_Z, PAD_BUTTON_START};

static const char* const named_buttons[] = {"B", "A", "L", "R", "Select", "Start"};

class PathGroup final : public ControllerEmu::ControlGroup
{
public:
  explicit PathGroup(std::string name) : ControllerEmu::ControlGroup(name) {}

  void LoadConfig(IniFile::Section* sec, const std::string& defdev,
                  const std::string& base) override
  {
    std::lock_guard lock(m_mutex);
    if (!sec->Get(base + name, &m_path))
      m_path.clear();
  }
  void SaveConfig(IniFile::Section* sec, const std::string& defdev,
                  const std::string& base) override
  {
    std::lock_guard lock(m_mutex);
    sec->Set(base + name, m_path);
  }

  std::string GetPath()
  {
    std::lock_guard lock(m_mutex);
    return m_path;
  }

  void SetPath(std::string_view path)
  {
    std::lock_guard lock(m_mutex);
    m_path = path;
  }

private:
  std::mutex m_mutex;
  std::string m_path;
};

GBAPad::GBAPad(const unsigned int index) : m_index(index)
{
  // Buttons
  groups.emplace_back(m_buttons = new ControllerEmu::Buttons(_trans("Buttons")));
  for (const char* named_button : named_buttons)
  {
    ControllerEmu::Translatability translate = ControllerEmu::DoNotTranslate;
    std::string ui_name = named_button;

    if (named_button == std::string("Start"))
    {
      translate = ControllerEmu::Translate;
      ui_name = _trans("START");
    }
    if (named_button == std::string("Select"))
    {
      translate = ControllerEmu::Translate;
      ui_name = _trans("SELECT");
    }

    m_buttons->AddInput(translate, named_button, std::move(ui_name));
  }

  // DPad
  groups.emplace_back(m_dpad = new ControllerEmu::Buttons(_trans("D-Pad")));
  for (const char* named_direction : named_directions)
  {
    m_dpad->AddInput(ControllerEmu::Translate, named_direction);
  }

  // ROM
  groups.emplace_back(m_rom = new PathGroup(_trans("ROM")));
}

std::string GBAPad::GetName() const
{
  return std::string("GBA") + char('1' + m_index);
}

ControllerEmu::ControlGroup* GBAPad::GetGroup(GBAPadGroup group)
{
  switch (group)
  {
  case GBAPadGroup::Buttons:
    return m_buttons;
  case GBAPadGroup::DPad:
    return m_dpad;
  case GBAPadGroup::ROM:
    return m_rom;
  default:
    return nullptr;
  }
}

GCPadStatus GBAPad::GetInput() const
{
  const auto lock = GetStateLock();
  GCPadStatus pad = {};

  // Buttons
  m_buttons->GetState(&pad.button, button_bitmasks);

  // DPad
  m_dpad->GetState(&pad.button, dpad_bitmasks);

  return pad;
}

void GBAPad::LoadDefaults(const ControllerInterface& ciface)
{
  EmulatedController::LoadDefaults(ciface);

  // Buttons
  m_buttons->SetControlExpression(0, "Z");  // B
  m_buttons->SetControlExpression(1, "X");  // A
  m_buttons->SetControlExpression(2, "Q");  // L
  m_buttons->SetControlExpression(3, "W");  // R
#ifdef _WIN32
  m_buttons->SetControlExpression(4, "BACK");    // Select
  m_buttons->SetControlExpression(5, "RETURN");  // Start
#else
  // OS X/Linux
  // Start
  m_buttons->SetControlExpression(4, "Backspace");
  m_buttons->SetControlExpression(5, "Return");
#endif

  // D-Pad
  m_dpad->SetControlExpression(0, "T");  // Up
  m_dpad->SetControlExpression(1, "G");  // Down
  m_dpad->SetControlExpression(2, "F");  // Left
  m_dpad->SetControlExpression(3, "H");  // Right

  // ROM
  SetRomPath("");
}

std::string GBAPad::GetRomPath()
{
  return static_cast<PathGroup*>(m_rom)->GetPath();
}

void GBAPad::SetRomPath(std::string_view path)
{
  static_cast<PathGroup*>(m_rom)->SetPath(path);
}
