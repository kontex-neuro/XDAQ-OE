/*
------------------------------------------------------------------

This file is part of the Open Ephys GUI
Copyright (C) 2022 Open Ephys

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

#include "ThorVision.h"

#include <fmt/core.h>

#include "ThorVisionEditor.h"


ThorVision::ThorVision(const std::string &ip, const int port)
    : GenericProcessor("ThorVision"),
      _ip(ip),
      _port(port),
      _lastConnected(false),
      _lastRecordingState(RecordingState::NotReady),
      _httpClient(_ip, _port, [this](bool connected, RecordingState recordingState) {
          MessageManager::callAsync([this, connected, recordingState] {
              if (connected != _lastConnected || recordingState != _lastRecordingState) {
                  _lastConnected = connected;
                  _lastRecordingState = recordingState;
                  sendChangeMessage();
              }
          });
      })
{
    _httpClient.startThread();
}


ThorVision::~ThorVision() { _httpClient.stopThread(5000); }


AudioProcessorEditor *ThorVision::createEditor()
{
    editor = std::make_unique<ThorVisionEditor>(this);
    return editor.get();
}


void ThorVision::registerParameters()
{
    // Register parameters here, if any
}


void ThorVision::updateSettings()
{
    // create and add a TTL channel to the first data stream
}


void ThorVision::process(AudioBuffer<float> &buffer) { checkForEvents(); }


void ThorVision::handleTTLEvent(TTLEventPtr event) {}


void ThorVision::handleSpike(SpikePtr spike) {}


void ThorVision::handleBroadcastMessage(const String &msg, const int64 messageTimeMilliseconds) {}


void ThorVision::saveCustomParametersToXml(XmlElement *parentElement) {}


void ThorVision::loadCustomParametersFromXml(XmlElement *parentElement) {}


void ThorVision::startRecording()
{
    const auto &endpoint = fmt::format("http://{}:{}/start", _ip, _port);
    sendPutRequest(endpoint);
}

void ThorVision::stopRecording()
{
    const auto &endpoint = fmt::format("http://{}:{}/stop", _ip, _port);
    sendPutRequest(endpoint);
}

void ThorVision::sendPutRequest(const String &endpoint, const String &body)
{
    Thread::launch([endpoint, body] {
        URL request(endpoint);
        fmt::println("ThorVision::sendPutRequest {}", endpoint.toStdString());

        const auto &options = URL::InputStreamOptions(URL::ParameterHandling::inPostData)
                                  .withExtraHeaders("Content-Type: text/plain")
                                  .withConnectionTimeoutMs(2000)
                                  .withHttpRequestCmd("PUT");

        if (auto stream = request.withPOSTData(body).createInputStream(options)) {
            const auto &response = stream->readEntireStreamAsString();
            fmt::println("ThorVision server ({}) response: {}", endpoint.toStdString(),
                         response.toStdString());
        }
    });
}
