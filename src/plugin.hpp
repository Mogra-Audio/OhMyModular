#pragma once
#include <rack.hpp>

using namespace rack;

// Declare the plugin instance (global variable)
extern Plugin* pluginInstance;

// Declare the model of our module (it will be created in OhMyPendulum.cpp)
extern Model* modelOhMyPendulum;

// Expander module (created in OhMyPendulumExpander.cpp)
extern Model* modelOhMyPendulumExpander;
