#include "plugin.hpp"
#include "Theme.hpp"
#include "ExpanderMessage.hpp"
#include "dsp/ArmKinematics.hpp"

#include <atomic>
#include <cmath>

////////////////////////////////////////////////////////////////////////////////
/// MODULE
////////////////////////////////////////////////////////////////////////////////

struct OhMyPendulumExpander : Module {
    enum ParamId { PARAMS_LEN };
    enum InputId {
        INPUT_TIME, INPUT_SKEW, INPUTS_LEN
    };
    enum OutputId {
        OUTPUT_ANGLE1_X, OUTPUT_ANGLE1_Y,
        OUTPUT_ANGLE2_X, OUTPUT_ANGLE2_Y,
        OUTPUTS_LEN
    };
    enum LightId { LIGHTS_LEN };

    // Theme resolved on the audio thread, applied by the widget.
    std::atomic<int> inheritedTheme{0};
    std::atomic<bool> parentPresent{false};

    // Keeps the last inherited theme after the expander is detached.
    std::atomic<int> theme{0};

    // This module owns the expander -> main message buffers.
    ExpanderToPendulumMsg toParent[2];
    bool lastCvActive = false;

    OhMyPendulumExpander() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configInput(INPUT_TIME, "Time CV");
        configInput(INPUT_SKEW, "Skew CV");
        configOutput(OUTPUT_ANGLE1_X, "Angle I - X");
        configOutput(OUTPUT_ANGLE1_Y, "Angle I - Y");
        configOutput(OUTPUT_ANGLE2_X, "Angle II - X");
        configOutput(OUTPUT_ANGLE2_Y, "Angle II - Y");

        leftExpander.producerMessage = &toParent[0];
        leftExpander.consumerMessage = &toParent[1];
    }

    void zeroOutputs() {
        for (int i = 0; i < OUTPUTS_LEN; i++) {
            outputs[i].setVoltage(0.f);
        }
    }

    void setCoordinateVoltage(int outputId, double coordinate) {
        float volts = coordinateToVolts(coordinate);
        outputs[outputId].setVoltage(std::isfinite(volts) ? clamp(volts, -5.f, 5.f) : 0.f);
    }

    void process(const ProcessArgs& args) override {
        Module* parent = leftExpander.module;
        bool hasParent = parent && parent->model == modelOhMyPendulum;
        parentPresent.store(hasParent, std::memory_order_relaxed);

        if (!hasParent) {
            zeroOutputs();
            return;
        }

        // Send the CV inputs back to the main module. When both inputs stay
        // disconnected there is nothing new to say, so skip the exchange.
        bool timeConnected = inputs[INPUT_TIME].isConnected();
        bool skewConnected = inputs[INPUT_SKEW].isConnected();
        bool cvActive = timeConnected || skewConnected;
        if (cvActive || cvActive != lastCvActive) {
            auto* out = static_cast<ExpanderToPendulumMsg*>(leftExpander.producerMessage);
            if (out) {
                out->timeCV = cvToUnit(inputs[INPUT_TIME].getVoltage());
                out->skewCV = cvToUnit(inputs[INPUT_SKEW].getVoltage());
                out->timeConnected = timeConnected;
                out->skewConnected = skewConnected;
                leftExpander.messageFlipRequested = true;
            }
        }
        lastCvActive = cvActive;

        // A bypassed parent produces no state; hold the outputs at zero.
        if (parent->isBypassed()) {
            zeroOutputs();
            return;
        }

        auto* in = static_cast<PendulumToExpanderMsg*>(parent->rightExpander.consumerMessage);
        if (!in) {
            zeroOutputs();
            return;
        }

        int theme = in->theme;
        if (theme != inheritedTheme.load(std::memory_order_relaxed)) {
            inheritedTheme.store(theme, std::memory_order_relaxed);
        }

        // The XY math is pure overhead unless at least one output is patched.
        bool anyOutput = false;
        for (int i = 0; i < OUTPUTS_LEN; i++) {
            if (outputs[i].isConnected()) {
                anyOutput = true;
                break;
            }
        }
        if (!anyOutput) return;

        ArmPoints p = computeArmPoints(in->theta1, in->theta2);
        setCoordinateVoltage(OUTPUT_ANGLE1_X, p.x1);
        setCoordinateVoltage(OUTPUT_ANGLE1_Y, p.y1);
        setCoordinateVoltage(OUTPUT_ANGLE2_X, p.x2);
        setCoordinateVoltage(OUTPUT_ANGLE2_Y, p.y2);
    }

    void onExpanderChange(const ExpanderChangeEvent& e) override {
        // The parent sits on the left.
        if (e.side == 0) {
            parentPresent.store(false, std::memory_order_relaxed);
        }
    }

    void processBypass(const ProcessArgs& args) override {
        // A bypassed expander must not keep overriding the main module's knobs.
        Module* parent = leftExpander.module;
        if (parent && parent->model == modelOhMyPendulum && lastCvActive) {
            auto* out = static_cast<ExpanderToPendulumMsg*>(leftExpander.producerMessage);
            if (out) {
                *out = ExpanderToPendulumMsg();
                leftExpander.messageFlipRequested = true;
            }
            lastCvActive = false;
        }
        zeroOutputs();
    }

    json_t* dataToJson() override {
        json_t* rootJ = json_object();
        json_object_set_new(rootJ, "theme", json_integer(theme.load(std::memory_order_relaxed)));
        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        json_t* themeJ = json_object_get(rootJ, "theme");
        if (themeJ) {
            theme.store(clamp((int)json_integer_value(themeJ), 0, THEME_COUNT - 1),
                        std::memory_order_relaxed);
        }
    }
};

////////////////////////////////////////////////////////////////////////////////
/// WIDGET
////////////////////////////////////////////////////////////////////////////////

struct OhMyPendulumExpanderWidget : ModuleWidget {
    int previewTheme = 0;
    math::Vec inputCenters[2];
    math::Vec outputCenters[4];
    app::SvgPort* inputs[2];
    app::SvgPort* outputs[4];
    int appliedTheme = -1;

    int curTheme() const {
        if (!module) return previewTheme;
        auto* m = static_cast<OhMyPendulumExpander*>(module);
        bool attached = m->parentPresent.load(std::memory_order_relaxed);
        int t = attached ? m->inheritedTheme.load(std::memory_order_relaxed)
                         : m->theme.load(std::memory_order_relaxed);
        return clamp(t, 0, THEME_COUNT - 1);
    }

    void applyTheme(int t) {
        setPanel(createPanel(asset::plugin(pluginInstance, THEMES[t].expanderPanel)));
        for (int i = 0; i < 2; i++) {
            inputs[i]->setSvg(APP->window->loadSvg(asset::plugin(pluginInstance, THEMES[t].input)));
            inputs[i]->box.pos = inputCenters[i] - inputs[i]->box.size / 2.0;
        }
        for (int i = 0; i < 4; i++) {
            outputs[i]->setSvg(APP->window->loadSvg(asset::plugin(pluginInstance, THEMES[t].output)));
            outputs[i]->box.pos = outputCenters[i] - outputs[i]->box.size / 2.0;
        }
        appliedTheme = t;
    }

    OhMyPendulumExpanderWidget(OhMyPendulumExpander* module) {
        setModule(module);
        float mw = 6.0f * RACK_GRID_WIDTH;
        float mh = RACK_GRID_HEIGHT;

        inputCenters[0] = mm2px(Vec(8.745f,  84.542f));
        inputCenters[1] = mm2px(Vec(21.732f, 84.542f));

        outputCenters[0] = mm2px(Vec(8.745f,  103.620f));
        outputCenters[1] = mm2px(Vec(21.732f, 103.653f));
        outputCenters[2] = mm2px(Vec(8.745f,  114.903f));
        outputCenters[3] = mm2px(Vec(21.732f, 114.903f));

        inputs[0] = createInput<app::SvgPort>(Vec(0,0), module, OhMyPendulumExpander::INPUT_TIME);
        inputs[1] = createInput<app::SvgPort>(Vec(0,0), module, OhMyPendulumExpander::INPUT_SKEW);
        for (int i = 0; i < 2; i++) addInput(inputs[i]);

        outputs[0] = createOutput<app::SvgPort>(Vec(0,0), module, OhMyPendulumExpander::OUTPUT_ANGLE1_X);
        outputs[1] = createOutput<app::SvgPort>(Vec(0,0), module, OhMyPendulumExpander::OUTPUT_ANGLE1_Y);
        outputs[2] = createOutput<app::SvgPort>(Vec(0,0), module, OhMyPendulumExpander::OUTPUT_ANGLE2_X);
        outputs[3] = createOutput<app::SvgPort>(Vec(0,0), module, OhMyPendulumExpander::OUTPUT_ANGLE2_Y);
        for (int i = 0; i < 4; i++) addOutput(outputs[i]);

        addChild(createWidget<ScrewBlack>(Vec(0, 0)));
        addChild(createWidget<ScrewBlack>(Vec(mw - RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(0, mh - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(mw - RACK_GRID_WIDTH, mh - RACK_GRID_WIDTH)));

        applyTheme(curTheme());
    }

    void step() override {
        ModuleWidget::step();
        if (!module) return;
        int t = curTheme();
        if (t != appliedTheme) {
            applyTheme(t);
            // Remember the inherited theme so detaching keeps the look.
            static_cast<OhMyPendulumExpander*>(module)->theme.store(t, std::memory_order_relaxed);
        }
    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
        menu->addChild(new MenuSeparator);

        auto* m = module ? static_cast<OhMyPendulumExpander*>(module) : nullptr;
        if (m && m->parentPresent.load(std::memory_order_relaxed)) {
            menu->addChild(createMenuLabel("Theme inherited from Oh My Pendulum"));
            return;
        }

        menu->addChild(createMenuLabel("Theme"));
        const char* names[] = {"Mono", "Creme", "Minty", "Mage", "Love"};
        for (int i = 0; i < THEME_COUNT; i++) {
            menu->addChild(createCheckMenuItem(names[i], "",
                [=]() { return curTheme() == i; },
                [=]() {
                    previewTheme = i;
                    if (module) static_cast<OhMyPendulumExpander*>(module)->theme.store(i, std::memory_order_relaxed);
                    applyTheme(i);
                }
            ));
        }
    }
};

Model* modelOhMyPendulumExpander = createModel<OhMyPendulumExpander, OhMyPendulumExpanderWidget>("OhMyPendulumExpander");
