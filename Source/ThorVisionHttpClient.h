#pragma once

#ifndef THORVISIONHTTPCLIENT_H_DEFINED
#define THORVISIONHTTPCLIENT_H_DEFINED

#include <JuceHeader.h>
#include <fmt/core.h>

class ThorVisionHttpClient final : public juce::Thread
{
public:
    explicit ThorVisionHttpClient(const std::string &ip = "127.0.0.1", const int port = 8000)
        : juce::Thread("HttpClient"), _ip(ip), _port(port)
    {
    }

    void run() override
    {
        while (!threadShouldExit()) {
            pingHost();
            wait(1000);
        }
    }

private:
    const std::string _ip;
    const int _port;

    void pingHost()
    {
        const auto &endpoint = fmt::format("http://{}:{}/ping", _ip, _port);
        URL request(endpoint);

        const auto &body = "Open Ephys";
        const auto &options = URL::InputStreamOptions(URL::ParameterHandling::inPostData)
                                  .withExtraHeaders("Content-Type: text/plain")
                                  .withConnectionTimeoutMs(1000)
                                  .withHttpRequestCmd("PUT");

        if (auto stream = request.withPOSTData(body).createInputStream(options)) {
            const auto &response = stream->readEntireStreamAsString();
        }
    }
};

#endif  // THORVISIONHTTPCLIENT_H_DEFINED