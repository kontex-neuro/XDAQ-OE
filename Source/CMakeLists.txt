#plugin build file
cmake_minimum_required(VERSION 3.5.0)

#include common rules
include(../PluginRules.cmake)

#add sources, not including OpenEphysLib.cpp
add_sources(${PLUGIN_NAME}
	rhythm-api/intan_chip.h
	rhythm-api/ports.h
	rhythm-api/rhd2000datablock.h
	rhythm-api/rhd2000evalboard.cpp
	rhythm-api/rhd2000evalboard.h
	rhythm-api/rhd2000registers.cpp
	rhythm-api/rhd2000registers.h
	rhythm-api/utils.h
	UI/ChannelComponent.h
	UI/ChannelComponent.cpp
	UI/ChannelList.h
	UI/ChannelList.cpp
	UI/ChannelCanvas.h
	UI/ChannelCanvas.cpp
	DeviceThread.cpp
	DeviceThread.h
	DeviceEditor.cpp
	DeviceEditor.h
	Headstage.h
	Headstage.cpp
	ImpedanceMeter.h
	ImpedanceMeter.cpp
	XDAQ.h
	ThorVisionTriggerEditor.h
	ThorVisionTriggerEditor.cpp
	ThorVisionTrigger.h
	ThorVisionTrigger.cpp
)


#optional: create IDE groups
plugin_create_filters()
