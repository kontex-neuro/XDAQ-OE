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

Headstage::Headstage(std::string port, std::string prefix, int chip_idx, int max_streams)
    : port(port),
      max_streams(max_streams),
      prefix(prefix),
      channelsPerStream(32),
      halfChannels(false),
      streamIndex(-1),
      firstChannelIndex(0),
      namingScheme(GLOBAL_INDEX)
{
}

void Headstage::setNumStreams(int num)
{
    LOGD("Headstage ", prefix, " setting num streams to ", num);

    if (num == 2) halfChannels = false;

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

void Headstage::setFirstStreamIndex(int streamIndex_) { streamIndex = streamIndex_; }

void Headstage::setFirstChannel(int channelIndex)
{
    LOGD("Headstage ", prefix, " setting first channel to ", channelIndex);

    if (firstChannelIndex != channelIndex) {
        firstChannelIndex = channelIndex;

        if (namingScheme == GLOBAL_INDEX) generateChannelNames();
    }
}

int Headstage::getStreamIndex(int offset) const { return streamIndex + offset; }

int Headstage::getNumChannels() const { return channelsPerStream * numStreams; }

void Headstage::setHalfChannels(bool half)
{
    if (getNumChannels() == 64) return;

    if (halfChannels != half) {
        halfChannels = half;

        generateChannelNames();
    }
}

int Headstage::getNumActiveChannels() const
{
    return (int) (getNumChannels() / (halfChannels ? 2 : 1));
}

bool Headstage::isConnected() const { return (numStreams > 0); }

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

void Headstage::setImpedances(Impedances &impedances)
{
    impedanceMagnitudes.clear();
    impedancePhases.clear();

    for (int i = 0; i < impedances.streams.size(); i++) {
        if (impedances.streams[i] == streamIndex) {
            impedanceMagnitudes.add(impedances.magnitudes[i]);
            impedancePhases.add(impedances.phases[i]);
        }

        if (numStreams == 2 && impedances.streams[i] == streamIndex + 1) {
            impedanceMagnitudes.add(impedances.magnitudes[i]);
            impedancePhases.add(impedances.phases[i]);
        }
    }
}

float Headstage::getImpedanceMagnitude(int channel) const
{
    if (channel < impedanceMagnitudes.size()) return impedanceMagnitudes[channel];

    return 0.0f;
}

float Headstage::getImpedancePhase(int channel) const
{
    if (channel < impedancePhases.size()) return impedancePhases[channel];

    return 0.0f;
}