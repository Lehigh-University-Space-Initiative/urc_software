#pragma once
#include "GUI.h"
#include <string>

class Panel {
protected: 
    std::string name;

    //the begin and and statments are alread handled so continue wihtout them
    virtual void drawBody() = 0;
public:
    Panel(std::string name);
    virtual ~Panel();

    void renderToScreen();

    //called once at
    virtual void setup();
    //called inbetween frames during ROS update time
    virtual void update();
};