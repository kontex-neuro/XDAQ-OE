#include "DeviceSelector.h"

#include <CoreServicesHeader.h>
#include <fmt/format.h>
#include <xdaq/device_plugin.h>

#include <filesystem>
#include <ranges>


#if defined(_WIN32)
#include <windows.h>
#endif



namespace fs = std::filesystem;

std::vector<info> get_device_options()
{
    auto sharedDir =
        fs::path(CoreServices::getSavedStateDirectory().getFullPathName().toStdString());
    if (fs::exists(sharedDir / "shared"))
        sharedDir = sharedDir / "shared";
    else {
        sharedDir = sharedDir / ("shared-api" + std::to_string(PLUGIN_API_VER));
        if (!fs::exists(sharedDir)) throw std::runtime_error("Shared directory not found");
    }

    auto xdaq_dir = sharedDir / "xdaq";
#if defined(_WIN32)
    constexpr auto shared_extension = ".dll";
    SetDllDirectoryA((xdaq_dir / "plugin").generic_string().c_str());
#elif defined(__APPLE__)
    constexpr auto shared_extension = ".dylib";
#elif defined(__linux__)
    constexpr auto shared_extension = ".so";
#else
    static_assert(false, "Unsupported platform");
#endif
    std::vector<info> device_options;
    auto plugin_paths_range =
        fs::directory_iterator(xdaq_dir / "plugin") |
        std::views::filter([=](const fs::directory_entry &entry) {
            fmt::print("Checking: {}\n", entry.path().generic_string());
            if (fs::is_directory(entry)) return false;
            // C++23: std::basic_string<CharT,Traits,Allocator>::contains
            if (entry.path().filename().generic_string().find("device_plugin") == std::string::npos)
                return false;
            return entry.path().extension() == shared_extension;
        }) |
        std::views::transform([](auto ent) { return fs::canonical(fs::path(ent)); });
    // C++23: std::ranges::to ref: https://stackoverflow.com/a/60971856
    // | std::ranges::to<std::vector>();
    std::vector<fs::path> plugin_paths;
    for (auto path : plugin_paths_range) plugin_paths.push_back(path);
    // remove duplicates
    std::ranges::sort(plugin_paths);
    plugin_paths.erase(std::unique(plugin_paths.begin(), plugin_paths.end()), plugin_paths.end());

    for (auto &path : plugin_paths) {
        fmt::print("Loading plugin: {}\n", path.generic_string());
        auto plugin = xdaq::get_plugin(path);
        auto plugin_info = json::parse(plugin->info());
        fmt::print("Plugin: {}\n", plugin_info.dump(4));
        for (auto &device_config : json::parse(plugin->list_devices())) {
            device_config["mode"] = "rhd";
            fmt::print("Device: {}\n", device_config.dump(4));
            auto device = plugin->create_device(device_config.dump());
            const auto info_str = device->get_info();
            fmt::print("Info: {}\n", info_str.value_or("N/A"));
            device_options.push_back({
                .plugin_path = path.generic_string(),
                .plugin_display_name = plugin_info["name"].get<std::string>(),
                .device_config = device_config,
                .device_info = info_str.has_value() ? json::parse(info_str.value())
                                                    : json{{"Serial Number", "N/A"}},
            });
        }
    }
    return device_options;
};

DeviceSelector::DeviceSelector(std::vector<info> &options) : options(options)
{
    table.getHeader().addColumn("Name", 1, 150);
    table.getHeader().addColumn("Serial Number", 2, 300);
    table.setColour(juce::ListBox::outlineColourId, juce::Colours::grey);
    table.setOutlineThickness(1);
    table.setBounds(10, 10, 452, 200);
    table.addKeyListener(this);
    addAndMakeVisible(table);

    refreshButton.setBounds(10, 220, 80, 20);
    refreshButton.addListener(this);
    addAndMakeVisible(refreshButton);

    acceptButton.setBounds(370, 220, 80, 20);
    acceptButton.addListener(this);
    addAndMakeVisible(acceptButton);
}

void accept(auto table, auto dw)
{
    if (table->getSelectedRow() == -1) {
        AlertWindow::showMessageBox(AlertWindow::WarningIcon, "No device selected",
                                    "Please select a device to launch", "OK");
        return;
    }
    if (dw) dw->exitModalState(table->getSelectedRow() + 1);
}

bool DeviceSelector::keyPressed(const KeyPress &key, Component *originatingComponent)
{
    if (key == KeyPress::returnKey) {
        accept(&table, findParentComponentOfClass<DialogWindow>());
        return true;
    }
    return false;
}

void DeviceSelector::paintCell(juce::Graphics &g, int rowNumber, int columnId, int width,
                               int height, bool rowIsSelected)
{
    g.setColour(rowIsSelected ? juce::Colours::darkblue
                              : getLookAndFeel().findColour(juce::ListBox::textColourId));
    const auto &d = options[rowNumber];
    if (columnId == 1)
        g.drawText(d.device_info.contains("XDAQ Model")
                       ? fmt::format("{}", d.device_info["XDAQ Model"].get<std::string>())
                       : "N/A",
                   2, 0, width - 4, height, juce::Justification::centredLeft, true);
    else if (columnId == 2)
        g.drawText(d.device_info.contains("Serial Number")
                       ? fmt::format("{}", d.device_info["Serial Number"].get<std::string>())
                       : "N/A",
                   2, 0, width - 4, height, juce::Justification::centredLeft, true);

    g.setColour(getLookAndFeel().findColour(juce::ListBox::backgroundColourId));
    g.fillRect(width - 1, 0, 1, height);
}

void DeviceSelector::paintRowBackground(juce::Graphics &g, int rowNumber, int /*width*/,
                                        int /*height*/, bool rowIsSelected)
{
    auto alternateColour =
        getLookAndFeel()
            .findColour(juce::ListBox::backgroundColourId)
            .interpolatedWith(getLookAndFeel().findColour(juce::ListBox::textColourId), 0.03f);
    if (rowIsSelected)
        g.fillAll(juce::Colours::lightblue);
    else if (rowNumber % 2)
        g.fillAll(alternateColour);
}

void DeviceSelector::buttonClicked(Button *button)
{
    if (button == &refreshButton) {
        options = get_device_options();
        table.updateContent();
    } else if (button == &acceptButton) {
        accept(&table, findParentComponentOfClass<DialogWindow>());
    }
}

void DeviceSelector::cellDoubleClicked(int rowNumber, int columnId, const MouseEvent &)
{
    accept(&table, findParentComponentOfClass<DialogWindow>());
}