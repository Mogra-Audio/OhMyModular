#pragma once
#include <rack.hpp>

using namespace rack;

// Объявляем экземпляр плагина (глобальная переменная)
extern Plugin* pluginInstance;

// Объявляем модель нашего модуля (она будет создана в OhMyPendulum.cpp)
extern Model* modelOhMyPendulum;
