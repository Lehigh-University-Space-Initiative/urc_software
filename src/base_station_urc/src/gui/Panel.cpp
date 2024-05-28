#include "Panel.h"

Panel::Panel(std::string name)
    : name(name)
{
}

Panel::~Panel()
{
}

void Panel::renderToScreen()
{
    //begin window if open
    //TODO: add window hiding

    ImGui::Begin(name.c_str());
    drawBody();
    ImGui::End();
}

void Panel::setup()
{
}

void Panel::update()
{
}
