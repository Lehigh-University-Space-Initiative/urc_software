/**
 * @file
 * Implementation of the Panel base class
 */
#include "Panel.h"

// Syntax: ": name(name), node_(node)" initializes the members from the parameters before the body runs
Panel::Panel(const std::string& name, const rclcpp::Node::SharedPtr& node)
    : name(name), node_(node)
{
}

Panel::~Panel()
{
}

void Panel::renderToScreen()
{
    // TODO: add window hiding
    ImGui::Begin(name.c_str());
    // Keeping windows from being removed from the main viewport (called after Begin, so it applies to the next window drawn)
    ImGui::SetNextWindowViewport(ImGui::GetMainViewport()->ID);
    drawBody();
    ImGui::End();
}

// Default setup/update do nothing; panels override them as needed
void Panel::setup()
{
}

void Panel::update()
{
}
