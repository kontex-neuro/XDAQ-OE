#pragma once

#ifndef THORVISIONHTTPCLIENT_H_DEFINED
#define THORVISIONHTTPCLIENT_H_DEFINED

#include <JuceHeader.h>
#include <fmt/core.h>

class ThorVisionHttpClient final : public juce::Thread
{
public:
    explicit ThorVisionHttpClient(const std::string &ip = "127.0.0.1", const int port = 8000)
        : juce::Thread("HttpClient"), _ip(ip), _port(port), _lastConnected(false)
    {
    }

    enum class RecordingState { Unknown, NotReady, Ready, Recording };

    std::function<void(bool, RecordingState)> onStatusChanged;

    void run() override
    {
        while (!threadShouldExit()) {
            pingHost();
            wait(1000);
        }
    }

    bool isConnected() const { return _lastConnected.load(); }

private:
    const std::string _ip;
    const int _port;
    std::atomic<bool> _lastConnected;
    RecordingState _lastRecordingState;

    void pingHost()
    {
        const auto &endpoint = fmt::format("http://{}:{}/status", _ip, _port);
        URL request(endpoint);

        const auto &body = "Open Ephys";
        const auto &options = URL::InputStreamOptions(URL::ParameterHandling::inPostData)
                                  .withExtraHeaders("Content-Type: text/plain")
                                  .withConnectionTimeoutMs(1000)
                                  .withHttpRequestCmd("PUT");

        auto connected = false;
        auto recordingState = RecordingState::Unknown;

        if (auto stream = request.withPOSTData(body).createInputStream(options)) {
            connected = true;

            auto response = stream->readEntireStreamAsString();
            const auto &json = juce::JSON::parse(response);

            if (json.isVoid() || !json.isObject()) {
                DBG("Invalid JSON response");
                return;
            }

            const auto &obj = json.getDynamicObject();
            const auto &status = obj->getProperty("Status").toString();

            if (status.equalsIgnoreCase("Recording")) {
                recordingState = RecordingState::Recording;
            } else if (status.equalsIgnoreCase("Ready")) {
                recordingState = RecordingState::Ready;
            } else if (status.equalsIgnoreCase("Not Ready")) {
                recordingState = RecordingState::NotReady;
            } else {
                DBG("Unknown Status: " + status);
            }
        }

        if (_lastConnected != connected || _lastRecordingState != recordingState) {
            _lastConnected = connected;
            _lastRecordingState = recordingState;

            if (onStatusChanged) onStatusChanged(connected, recordingState);
        }
    }
};

#endif  // THORVISIONHTTPCLIENT_H_DEFINED