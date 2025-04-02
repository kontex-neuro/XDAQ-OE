#pragma once
#include <VisualizerEditorHeaders.h>

#include <nlohmann/json.hpp>


using json = nlohmann::json;

struct info {
    std::string device_manager_path;
    std::string display_name;
    json device_config;
    json device_info;
};

std::vector<info> get_device_options();

class DeviceSelector final : public Component,
                             public KeyListener,
                             public TableListBoxModel,
                             public Button::Listener
{
public:
    DeviceSelector(std::vector<info> &options);
    ~DeviceSelector() override {}

    int getNumRows() override { return options.size(); };

    bool keyPressed(const KeyPress &key, Component *originatingComponent) override;
    void buttonClicked(Button *button) override;
    void cellDoubleClicked(int rowNumber, int columnId, const MouseEvent &) override;


    void paintCell(juce::Graphics &g, int rowNumber, int columnId, int width, int height,
                   bool rowIsSelected) override;
    void paintRowBackground(juce::Graphics &g, int rowNumber, int width, int height,
                            bool rowIsSelected) override;



private:
    TableListBox table{{}, this};
    std::vector<info> &options;
    UtilityButton refreshButton{"REFRESH"};
    UtilityButton acceptButton{"LAUNCH"};
};
