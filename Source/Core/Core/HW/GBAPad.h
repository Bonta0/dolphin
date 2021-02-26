// Copyright 2021 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <string>
#include <string_view>

class InputConfig;
enum class GBAPadGroup;
struct GCPadStatus;

namespace ControllerEmu
{
class ControlGroup;
}  // namespace ControllerEmu

namespace Pad
{
void ShutdownGBA();
void InitializeGBA();
void LoadGBAConfig();
bool IsGBAInitialized();

InputConfig* GetGBAConfig();

GCPadStatus GetGBAStatus(int pad_num);
std::string GetGBARomPath(int pad_num);
void SetGBARomPath(int pad_num, std::string_view path);

ControllerEmu::ControlGroup* GetGBAGroup(int pad_num, GBAPadGroup group);
}  // namespace Pad
