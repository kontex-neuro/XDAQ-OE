/*
    ------------------------------------------------------------------

    This file is part of the Open Ephys GUI
    Copyright (C) 2021 Open Ephys

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


#ifndef __HEADSTAGE_H_2C4CBD67__
#define __HEADSTAGE_H_2C4CBD67__

#include <DataThreadHeaders.h>

#include <array>
#include <atomic>
#include <string>
#include <vector>

#include "rhythm-api/okFrontPanelDLL.h"
#include "rhythm-api/rhd2000datablock.h"
#include "rhythm-api/rhd2000evalboard.h"
#include "rhythm-api/rhd2000registers.h"

namespace RhythmNode
{


enum ChannelNamingScheme { GLOBAL_INDEX = 1, STREAM_INDEX = 2 };

/**
    A headstage object represents a data source containing
    one or more Intan chips.

    Each headstage can send 1 or 2 data streams, each with
    up to 64 channels.

    A headstage can be identified in the following ways:
    - dataSource   : port (A1-D2Ddr) that the headstage is connected to
    - streamIndex  : location in the array of active streams
    - startChannel : index of the first channel acquired by this headstage,
                     out of all actively acquired neural data channels

*/
class Headstage
{
public:
    const std::string prefix;
    const std::string port;
    /** Constructor */
    Headstage(std::string port, std::string prefix, int max_streams);

    /** Destructor*/
    ~Headstage() = default;

    /** Sets the index of this headstage's first neural data channel*/
    void setFirstChannel(int channelIndex);

    /** Returns the number of channels this headstage sends*/
    int getNumChannels() const { return channelsPerStream * numStreams; }

    /** Sets the number of channels per stream*/
    void setChannelsPerStream(int nchan);
    int getNumStreams() const { return streams; }
    /** Sets the number of streams for this headstage (1 or 2)*/
    void setNumStreams(int num);

    /** Returns the number of actively acquired neural data channels*/
    int getNumActiveChannels() const { return getNumChannels();}

    /** Returns the name of a channel at a given index*/
    String getChannelName(int ch) const;


    const std::vector<std::string> &get_channel_names();

    /** Sets the name of a channel at a given index*/
    void setChannelName(std::string name, int ch);

    /** Sets the channel naming scheme*/
    void setNamingScheme(ChannelNamingScheme scheme);

    /** Returns true if the headstage is connected*/
    bool isConnected() const { return (numStreams > 0); }

    /** Auto-generates the channel names, based on the naming scheme*/
    void generateChannelNames();

    /** Sets impedance values after measurement*/
    void setImpedances(std::vector<float> magnitude, std::vector<float> phase)
    {
        impedanceMagnitudes = magnitude;
        impedancePhases = phase;
    }

    /** Returns the impedance magnitude for a channel (if it exists)*/
    float getImpedanceMagnitude(int channel) const
    {
        return channel < impedanceMagnitudes.size() ? impedanceMagnitudes[channel] : 0.0f;
    }

    /** Returns the impedance phase for a channel (if it exists)*/
    float getImpedancePhase(int channel) const
    {
        return channel < impedancePhases.size() ? impedancePhases[channel] : 0.0f;
    }

    /** Returns true if impedance has been measured*/
    bool hasImpedanceData() const { return impedanceMagnitudes.size() > 0; }

private:
    int streams;
    int numStreams;
    int channelsPerStream;
    int firstChannelIndex;

    int MAX_NUM_HEADSTAGES;

    ChannelNamingScheme namingScheme;

    std::vector<std::string> channelNames;

    std::vector<float> impedanceMagnitudes;
    std::vector<float> impedancePhases;
};


}  // namespace RhythmNode
#endif  // __HEADSTAGE_H_2C4CBD67__
