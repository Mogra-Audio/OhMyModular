#pragma once

// Panel and asset paths for every theme. Shared by the main module and its
// expander so both always render the same look.
struct ThemeInfo {
    const char* panel;
    const char* expanderPanel;
    const char* smallKnob;
    const char* hugeKnob;
    const char* input;
    const char* output;
};

static constexpr int THEME_COUNT = 5;

static constexpr ThemeInfo THEMES[THEME_COUNT] = {
    {
        /// Mono
        "res/Panel-Mono.svg",
        "res/Panel-Expander-Mono.svg",
        "res/SmallKnob.svg",
        "res/HugeKnob.svg",
        "res/Input.svg",
        "res/Output.svg"
    },
    {
        /// Creme
        "res/themes/theme_Creme/Panel-Creme.svg",
        "res/themes/theme_Creme/Panel-Expander-Creme.svg",
        "res/themes/theme_Creme/SmallKnob.svg",
        "res/themes/theme_Creme/HugeKnob.svg",
        "res/themes/theme_Creme/Input.svg",
        "res/themes/theme_Creme/Output.svg"
    },
    {
        /// Minty
        "res/themes/theme_Minty/Panel-Minty.svg",
        "res/themes/theme_Minty/Panel-Expander-Minty.svg",
        "res/themes/theme_Minty/SmallKnob.svg",
        "res/themes/theme_Minty/HugeKnob.svg",
        "res/themes/theme_Minty/Input.svg",
        "res/themes/theme_Minty/Output.svg"
    },
    {
        /// Mage
        "res/themes/theme_Mage/Panel-Mage.svg",
        "res/themes/theme_Mage/Panel-Expander-Mage.svg",
        "res/themes/theme_Mage/SmallKnob.svg",
        "res/themes/theme_Mage/HugeKnob.svg",
        "res/themes/theme_Mage/Input.svg",
        "res/themes/theme_Mage/Output.svg"
    },
    {
        /// Love
        "res/themes/theme_Love/Panel-Love.svg",
        "res/themes/theme_Love/Panel-Expander-Love.svg",
        "res/themes/theme_Love/SmallKnob-1.svg",
        "res/themes/theme_Love/HugeKnob.svg",
        "res/themes/theme_Love/Input-1.svg",
        "res/themes/theme_Love/Output-1.svg"
    }
};
