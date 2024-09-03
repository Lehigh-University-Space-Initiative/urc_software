#include "Panel.h"

Panel::Panel(const std::string &name, const rclcpp::Node::SharedPtr &node)
    : name(name), node_(node) {
}

Panel::~Panel() {
}

void Panel::renderToScreen() {
    // Begin window if open
    // TODO: add window hiding

    ImGui::Begin(name.c_str());
    drawBody();
    ImGui::End();
}

void Panel::setup() {
}

void Panel::update() {
}
