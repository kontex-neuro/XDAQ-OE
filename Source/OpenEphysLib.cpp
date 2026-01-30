/*
------------------------------------------------------------------

This file is part of the Open Ephys GUI
Copyright (C) 2013 Open Ephys

------------------------------------------------------------------

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <PluginInfo.h>

#include <string>

#include "ThorVision.h"
#include "XDAQ.h"

#ifdef WIN32
#include <Windows.h>
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif

using namespace Plugin;
#define NUM_PLUGINS 2


extern "C" EXPORT void getLibInfo(Plugin::LibraryInfo *info)
{
    const static auto version =
        (std::to_string(PLUGIN_VERSION_MAJOR) + "." + std::to_string(PLUGIN_VERSION_MINOR) + "." +
         std::to_string(PLUGIN_VERSION_PATCH));
    info->apiVersion = PLUGIN_API_VER;
    info->name = "XDAQ";
    info->libVersion = version.c_str();
    info->numPlugins = NUM_PLUGINS;
}

extern "C" EXPORT int getPluginInfo(int index, Plugin::PluginInfo *info)
{
    switch (index) {
    case 0:
        info->type = Plugin::Type::DATA_THREAD;
        info->dataThread.name = "XDAQ";
        info->dataThread.creator = &createDataThread<RhythmNode::XDAQSystem>;
        break;
    case 1:
        info->type = Plugin::Type::PROCESSOR;
        info->processor.name = "ThorVision";
        info->processor.type = Plugin::Processor::UTILITY;
        info->processor.creator = &createProcessor<ThorVision>;
        break;
    default: return -1; break;
    }
    return 0;
}

#ifdef WIN32
BOOL WINAPI DllMain(IN HINSTANCE hDllHandle, IN DWORD nReason, IN LPVOID Reserved) { return TRUE; }

#endif