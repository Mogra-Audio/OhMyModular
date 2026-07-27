#include "plugin.hpp"

Plugin* pluginInstance;

// INIT FUNC
void init(Plugin* p) {
    pluginInstance = p;
    p->addModel(modelOhMyPendulum);
}
