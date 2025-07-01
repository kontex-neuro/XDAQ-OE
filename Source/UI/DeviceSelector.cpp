#include "DeviceSelector.h"

#include <CoreServicesHeader.h>
#include <fmt/format.h>
#include <xdaq/device_manager.h>

#include <filesystem>
#include <unordered_set>



namespace fs = std::filesystem;

std::vector<info> get_device_options()
{
#if defined(OS_MACOS)
    auto sharedDir =
        fs::path(CoreServices::getSavedStateDirectory().getFullPathName().toStdString());
    fs::path pluginDir = sharedDir / "plugins";
    if (!fs::exists(pluginDir)) {
        pluginDir = sharedDir / ("plugins-api" + std::to_string(PLUGIN_API_VER));
        if (!fs::exists(pluginDir)) throw std::runtime_error("Shared directory not found");
    }
    auto device_manager_dir = pluginDir / "XDAQ-OE.bundle" / "Contents" / "PlugIns" / "managers";
#else
    auto sharedDir =
        fs::path(CoreServices::getSavedStateDirectory().getFullPathName().toStdString());
    if (fs::exists(sharedDir / "shared"))
        sharedDir = sharedDir / "shared";
    else {
        sharedDir = sharedDir / ("shared-api" + std::to_string(PLUGIN_API_VER));
        if (!fs::exists(sharedDir)) throw std::runtime_error("Shared directory not found");
    }
    auto device_manager_dir = sharedDir / "XDAQ-OE" / "managers";
#endif

    std::vector<info> device_options;

    std::unordered_set<fs::path> search_paths;
    if (fs::exists(device_manager_dir))
        for (auto path : fs::directory_iterator(device_manager_dir))
            search_paths.insert(fs::canonical(path));
    else
        LOGE(fmt::format("Device manager directory not found: {}",
                         device_manager_dir.generic_string()));

    for (auto &path : search_paths) {
        try {
            auto device_manager = xdaq::get_device_manager(path);
            LOGD(fmt::format("Found device manager: {}", path.generic_string()));
            auto device_manager_info = json::parse(device_manager->info());
            for (auto &device_config : json::parse(device_manager->list_devices())) {
                device_config["mode"] = "rhd";
                fmt::print("Device: {}\n", device_config.dump(4));
                auto device = device_manager->create_device(device_config.dump());
                const auto info_str = device->get_info();
                fmt::print("Info: {}\n", info_str.value_or("N/A"));
                const auto status_str = device->get_status();
                fmt::print("Status: {}\n", status_str.value_or("N/A"));
                device_options.push_back({
                    .device_manager_path = path.generic_string(),
                    .display_name = device_manager_info["name"].get<std::string>(),
                    .device_config = device_config,
                    .device_info = info_str.has_value() ? json::parse(info_str.value())
                                                        : json{{"Serial Number", "N/A"}},
                });
            }
        } catch (...) {
            LOGD(fmt::format("Not a device manager: {}", path.generic_string()));
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