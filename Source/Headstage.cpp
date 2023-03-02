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

#include "Headstage.h"

using namespace RhythmNode;

Headstage::Headstage(std::string port, std::string prefix, int max_streams)
    : port(port),
      prefix(prefix)
{
}

void Headstage::setNumStreams(int num)
{
    LOGD("Headstage ", prefix, " setting num streams to ", num);

    if (numStreams != num) {
        numStreams = num;

        generateChannelNames();
    }
}

void Headstage::setChannelsPerStream(int nchan)
{
    LOGD("Headstage ", prefix, " setting channels per stream to ", nchan);

    if (channelsPerStream != nchan) {
        channelsPerStream = nchan;

        generateChannelNames();
    }
}

void Headstage::setFirstChannel(int channelIndex)
{
    LOGD("Headstage ", prefix, " setting first channel to ", channelIndex);

    if (firstChannelIndex != channelIndex) {
        firstChannelIndex = channelIndex;

        if (namingScheme == GLOBAL_INDEX) generateChannelNames();
    }
}


String Headstage::getChannelName(int ch) const
{
    String name;

    if (ch > -1 && ch < channelNames.size())
        name = channelNames[ch];
    else
        name = " ";

    if (ch == 0) LOGD("Headstage ", prefix, " channel ", ch, " name: ", name);

    return name;
}

void Headstage::setChannelName(std::string name, int ch)
{
    if (ch > -1 && ch < channelNames.size()) channelNames[ch] = name;
}

void Headstage::setNamingScheme(ChannelNamingScheme scheme)
{
    if (namingScheme != scheme) {
        namingScheme = scheme;

        generateChannelNames();
    }
}

void Headstage::generateChannelNames()
{
    channelNames.clear();

    switch (namingScheme) {
    case GLOBAL_INDEX:
        for (int i = 0; i < getNumActiveChannels(); i++) {
            channelNames.push_back("CH" + std::to_string(firstChannelIndex + i + 1));
        }
        break;
    case STREAM_INDEX:
        for (int i = 0; i < getNumActiveChannels(); i++) {
            channelNames.push_back(prefix + "_CH" + std::to_string(i + 1));
        }
    }
}
