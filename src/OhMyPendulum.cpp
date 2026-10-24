#include "plugin.hpp"
#include "Theme.hpp"
#include "ExpanderMessage.hpp"
#include "dsp/PendulumCore.hpp"

#include <atomic>
#include <cstdint>

// UI-only constants.
static constexpr float KNOB_MIN_ANGLE = -0.83f * (float)kPi;
static constexpr float KNOB_MAX_ANGLE =  0.83f * (float)kPi;
static constexpr double ANGLE_HOLD_LEASE = 0.5; // seconds
static constexpr float LINK_W_MM = 1.162f;
static constexpr float LINK_H_MM = 4.089f;

////////////////////////////////////////////////////////////////////////////////
/// MODULE
////////////////////////////////////////////////////////////////////////////////

struct OhMyPendulum : Module {
    enum ParamId {
        PARAM_ANGLE1, PARAM_ANGLE2, PARAM_TIME, PARAM_SKEW, PARAM_CHAOS, PARAMS_LEN
    };
    enum InputId {
        INPUT_1_RETRIG, INPUT_2_CHAOS, INPUTS_LEN
    };
    enum OutputId {
        OUTPUT_1, OUTPUT_2, OUTPUT_3, OUTPUT_4, OUTPUTS_LEN
    };
    enum LightId { LIGHTS_LEN };

    // Saved in the patch and mirrored to the right-hand expander.
    std::atomic<int> theme{0};

    // Expander link: this module owns the main -> expander message buffers.
    PendulumToExpanderMsg toExpander[2];
    std::atomic<bool> timeCvActive{false};
    std::atomic<bool> skewCvActive{false};

    PendulumCore core;

    // Display mirrors (audio thread writes, UI thread reads)
    std::atomic<double> dispTheta1{0.0};
    std::atomic<double> dispTheta2{0.0};

    std::atomic<bool> resetStateRequested{true};

    // Absolute audio frame until which the Angle knob hold stays valid. The UI
    // refreshes it while the knob is down; the audio thread only compares it.
    std::atomic<int64_t> angleHoldUntilFrame{0};

    dsp::SchmittTrigger retrigTrigger;

    OhMyPendulum() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        /// PARAMs CFG
        configParam(PARAM_ANGLE1, 0.f, 1.f, 0.f, "Angle I", "°", 0.f, 360.f, 0.f);
        configParam(PARAM_ANGLE2, 0.f, 1.f, 0.f, "Angle II", "°", 0.f, 360.f, 0.f);
        /**/paramQuantities[PARAM_ANGLE1]->displayPrecision = 6;
        /**/paramQuantities[PARAM_ANGLE2]->displayPrecision = 6;
        configParam(PARAM_TIME, 0.f, 1.f, 0.5f, "Time");
        configParam(PARAM_SKEW, 0.f, 1.f, 0.5f, "Skew");
        configParam(PARAM_CHAOS, 0.f, 1.f, 0.f, "Chaos");

        /// I/O CFG
        configInput(INPUT_1_RETRIG, "Retrigger");
        configInput(INPUT_2_CHAOS, "Chaos CV");
        configOutput(OUTPUT_1, "Angle I (Smooth)");
        configOutput(OUTPUT_2, "Angle I (Discrete)");
        configOutput(OUTPUT_3, "Angle II (Discrete)");
        configOutput(OUTPUT_4, "Angle II (Smooth)");

        // Expander message buffers (sender owns both halves of the pair).
        rightExpander.producerMessage = &toExpander[0];
        rightExpander.consumerMessage = &toExpander[1];
    }

    void onSampleRateChange(const SampleRateChangeEvent& e) override {
        core.onSampleRateChange(e.sampleTime);
    }

    void process(const ProcessArgs& args) override {
        const float angle1Param = params[PARAM_ANGLE1].getValue();
        const float angle2Param = params[PARAM_ANGLE2].getValue();
        const float timeParam = params[PARAM_TIME].getValue();
        const float skewParam = params[PARAM_SKEW].getValue();
        const float chaosParam = params[PARAM_CHAOS].getValue();

        // Expander link: the right-hand expander owns the Time/Skew CV inputs.
        // A patched input takes over the matching knob; smoothing hides the jump.
        Module* expander = rightExpander.module;
        const bool hasExpander = expander && expander->model == modelOhMyPendulumExpander;
        bool timeCvConnected = false;
        bool skewCvConnected = false;
        float timeCv = 0.f;
        float skewCv = 0.f;
        if (hasExpander) {
            auto* msg = static_cast<ExpanderToPendulumMsg*>(expander->leftExpander.consumerMessage);
            if (msg) {
                timeCvConnected = msg->timeConnected;
                skewCvConnected = msg->skewConnected;
                timeCv = msg->timeCV;
                skewCv = msg->skewCV;
            }
        }
        timeCvActive.store(timeCvConnected, std::memory_order_relaxed);
        skewCvActive.store(skewCvConnected, std::memory_order_relaxed);

        if (resetStateRequested.exchange(false)) {
            core.reset(angle1Param, angle2Param);
        }

        const bool retrigger = retrigTrigger.process(inputs[INPUT_1_RETRIG].getVoltage());
        const bool angleHeld = args.frame < angleHoldUntilFrame.load(std::memory_order_relaxed);

        PendulumCore::Inputs in;
        in.angle1 = angle1Param;
        in.angle2 = angle2Param;
        in.time = timeParam;
        in.skew = skewParam;
        in.timeCv = timeCv;
        in.skewCv = skewCv;
        in.timeCvActive = timeCvConnected;
        in.skewCvActive = skewCvConnected;
        in.chaos = chaosAmount(chaosParam, inputs[INPUT_2_CHAOS].getVoltage());
        in.retrigger = retrigger;
        in.angleHeld = angleHeld;

        core.process(in, args.sampleTime);

        // Publish state to the expander (it reads the previous sample).
        if (hasExpander) {
            auto* msg = static_cast<PendulumToExpanderMsg*>(rightExpander.producerMessage);
            if (msg) {
                msg->theta1 = core.state.theta1;
                msg->theta2 = core.state.theta2;
                msg->theme = theme.load(std::memory_order_relaxed);
                rightExpander.messageFlipRequested = true;
            }
        }

        // Publish to UI (relaxed: need atomicity, not ordering)
        dispTheta1.store(core.state.theta1, std::memory_order_relaxed);
        dispTheta2.store(core.state.theta2, std::memory_order_relaxed);

        outputs[OUTPUT_1].setVoltage(core.outputs.smooth1);
        outputs[OUTPUT_2].setVoltage(core.outputs.discrete1);
        outputs[OUTPUT_3].setVoltage(core.outputs.discrete2);
        outputs[OUTPUT_4].setVoltage(core.outputs.smooth2);
    }

    void onReset(const ResetEvent& e) override {
        Module::onReset(e);
        resetStateRequested.store(true);
        angleHoldUntilFrame.store(0, std::memory_order_relaxed);
    }

    void onRandomize(const RandomizeEvent& e) override {
        Module::onRandomize(e);
        resetStateRequested.store(true);
    }

    void onExpanderChange(const ExpanderChangeEvent& e) override {
        // The right-hand expander owns the Time/Skew overrides; drop them on change.
        if (e.side == 1) {
            timeCvActive.store(false, std::memory_order_relaxed);
            skewCvActive.store(false, std::memory_order_relaxed);
        }
    }

    void processBypass(const ProcessArgs& args) override {
        core.outputs.reset();
        for (int i = 0; i < OUTPUTS_LEN; i++) {
            outputs[i].setVoltage(0.f);
        }
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
/// ANGLE KNOB — reports mouse hold to module
////////////////////////////////////////////////////////////////////////////////

struct AngleKnob : app::SvgKnob {
    OhMyPendulum* mod = nullptr;
    bool isHeld = false; // UI-thread only

    void setHold(bool held) {
        isHeld = held;
        if (!mod) return;
        int64_t until = 0;
        if (held && APP->engine) {
            until = APP->engine->getFrame()
                  + (int64_t)(ANGLE_HOLD_LEASE * APP->engine->getSampleRate());
        }
        mod->angleHoldUntilFrame.store(until, std::memory_order_relaxed);
    }

    void onButton(const ButtonEvent& e) override {
        SvgKnob::onButton(e);
        if (!mod || e.button != GLFW_MOUSE_BUTTON_LEFT) return;
        if (e.action == GLFW_PRESS) {
            setHold(true);
        } else if (e.action == GLFW_RELEASE) {
            setHold(false);
        }
    }

    // Safety: user drags outside widget and releases
    void onDragEnd(const DragEndEvent& e) override {
        SvgKnob::onDragEnd(e);
        if (e.button == GLFW_MOUSE_BUTTON_LEFT) setHold(false);
    }

    void step() override {
        SvgKnob::step();
        if (!isHeld || !mod) return;
        if (!APP->window || !APP->window->win || !APP->engine) {
            setHold(false);
            return;
        }
        // A lost mouse-up must not freeze the pendulum forever. The physical
        // button state is the source of truth, not the event stream.
        bool focused = glfwGetWindowAttrib(APP->window->win, GLFW_FOCUSED) != 0;
        bool pressed = glfwGetMouseButton(APP->window->win, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        if (!focused || !pressed) {
            setHold(false);
            return;
        }
        // If the UI stopped stepping long enough for the lease to lapse, do not
        // revive it from a possibly stale pressed state.
        if (mod->angleHoldUntilFrame.load(std::memory_order_relaxed)
                <= APP->engine->getFrame()) {
            setHold(false);
            return;
        }
        setHold(true);
    }
};

////////////////////////////////////////////////////////////////////////////////
/// CV KNOB — dimmed and locked while the expander drives the parameter
////////////////////////////////////////////////////////////////////////////////

struct CvKnob : app::SvgKnob {
    bool disabled = false;

    void onButton(const ButtonEvent& e) override {
        if (disabled && e.button == GLFW_MOUSE_BUTTON_LEFT) {
            // Swallow the press so it can't start a module drag either.
            e.consume(this);
            return;
        }
        SvgKnob::onButton(e);
    }

    void onDoubleClick(const DoubleClickEvent& e) override {
        if (disabled) {
            e.consume(this);
            return;
        }
        SvgKnob::onDoubleClick(e);
    }

    void onDragStart(const DragStartEvent& e) override {
        if (disabled) return;
        SvgKnob::onDragStart(e);
    }

    void onDragMove(const DragMoveEvent& e) override {
        if (disabled) return;
        SvgKnob::onDragMove(e);
    }

    void onDragEnd(const DragEndEvent& e) override {
        // Always let the base clean up a drag that started before the lock.
        SvgKnob::onDragEnd(e);
    }

    void draw(const DrawArgs& args) override {
        SvgKnob::draw(args);
        if (!disabled) return;
        nvgBeginPath(args.vg);
        nvgCircle(args.vg, box.size.x * 0.5f, box.size.y * 0.5f, box.size.x * 0.5f);
        nvgFillColor(args.vg, nvgRGBA(0, 0, 0, 110));
        nvgFill(args.vg);
    }
};

////////////////////////////////////////////////////////////////////////////////
/// WIDGET
////////////////////////////////////////////////////////////////////////////////

struct OhMyPendulumWidget : ModuleWidget {
    int previewTheme = 0;
    math::Vec knobCenters[5];
    math::Vec inputCenters[2];
    math::Vec outputCenters[4];
    app::SvgKnob* knobs[5];
    CvKnob* timeKnob = nullptr;
    CvKnob* skewKnob = nullptr;
    app::SvgPort* inputs[2];
    app::SvgPort* outputs[4];
    widget::TransformWidget* link1 = nullptr;
    widget::TransformWidget* link2 = nullptr;
    math::Vec pendulumPos;

    int curTheme() const {
        int t = module
            ? static_cast<OhMyPendulum*>(module)->theme.load(std::memory_order_relaxed)
            : previewTheme;
        return clamp(t, 0, THEME_COUNT - 1);
    }

    std::string getKnobPath(int i) {
        const ThemeInfo& t = THEMES[curTheme()];
        return i == 4 ? t.hugeKnob : t.smallKnob;
    }
    std::string getInputPath()  { return THEMES[curTheme()].input; }
    std::string getOutputPath() { return THEMES[curTheme()].output; }
    std::string getPanelPath()  { return THEMES[curTheme()].panel; }

    void updatePendulum() {
        if (!module || !link1 || !link2) return;
        OhMyPendulum* m = static_cast<OhMyPendulum*>(module);
        float t1 = (float)m->dispTheta1.load(std::memory_order_relaxed);
        float t2 = (float)m->dispTheta2.load(std::memory_order_relaxed);
        float halfW = mm2px(Vec(LINK_W_MM * 0.5f, 0.0f)).x;
        Vec linkPivot = Vec(halfW, 0.0f);
        link1->identity();
        link1->rotate(t1, linkPivot);
        link2->identity();
        link2->rotate(t2 - t1, linkPivot);
    }

    void updateTheme() {
        setPanel(createPanel(asset::plugin(pluginInstance, getPanelPath())));
        for (int i = 0; i < 5; i++) {
            knobs[i]->setSvg(APP->window->loadSvg(asset::plugin(pluginInstance, getKnobPath(i))));
            knobs[i]->box.pos  = knobCenters[i] - knobs[i]->box.size / 2.0;
        }
        for (int i = 0; i < 2; i++) {
            inputs[i]->setSvg(APP->window->loadSvg(asset::plugin(pluginInstance, getInputPath())));
            inputs[i]->box.pos = inputCenters[i] - inputs[i]->box.size / 2.0;
        }
        for (int i = 0; i < 4; i++) {
            outputs[i]->setSvg(APP->window->loadSvg(asset::plugin(pluginInstance, getOutputPath())));
            outputs[i]->box.pos = outputCenters[i] - outputs[i]->box.size / 2.0;
        }
    }

    OhMyPendulumWidget(OhMyPendulum* module) {
        setModule(module);
        float mw = 11.0f * RACK_GRID_WIDTH;
        float mh = RACK_GRID_HEIGHT;

        knobCenters[0] = mm2px(Vec(8.89,  74.51));
        knobCenters[1] = mm2px(Vec(21.59, 74.51));
        knobCenters[2] = mm2px(Vec(34.29, 74.51));
        knobCenters[3] = mm2px(Vec(46.99, 74.51));
        knobCenters[4] = mm2px(Vec(14.51, 93.01));

        inputCenters[0] = mm2px(Vec(8.89,   114.877));
        inputCenters[1] = mm2px(Vec(20.141, 114.877));

        outputCenters[0] = mm2px(Vec(33.91,  103.59));
        outputCenters[1] = mm2px(Vec(46.894, 103.627));
        outputCenters[2] = mm2px(Vec(46.894, 114.877));
        outputCenters[3] = mm2px(Vec(33.91,  114.877));

        // Angle knobs — custom AngleKnob with hold detection
        knobs[0] = createParam<AngleKnob>(Vec(0,0), module, OhMyPendulum::PARAM_ANGLE1);
        knobs[1] = createParam<AngleKnob>(Vec(0,0), module, OhMyPendulum::PARAM_ANGLE2);
        if (module) {
            static_cast<AngleKnob*>(knobs[0])->mod = static_cast<OhMyPendulum*>(module);
            static_cast<AngleKnob*>(knobs[1])->mod = static_cast<OhMyPendulum*>(module);
        }
        // Standard knobs; Time/Skew dim while the expander drives them.
        knobs[2] = timeKnob = createParam<CvKnob>(Vec(0,0), module, OhMyPendulum::PARAM_TIME);
        knobs[3] = skewKnob = createParam<CvKnob>(Vec(0,0), module, OhMyPendulum::PARAM_SKEW);
        knobs[4] = createParam<app::SvgKnob>(Vec(0,0), module, OhMyPendulum::PARAM_CHAOS);
        for (int i = 0; i < 5; i++) {
            knobs[i]->minAngle = KNOB_MIN_ANGLE;
            knobs[i]->maxAngle = KNOB_MAX_ANGLE;
            knobs[i]->speed    = 2.f;
            addParam(knobs[i]);
        }

        inputs[0] = createInput<app::SvgPort>(Vec(0,0), module, OhMyPendulum::INPUT_1_RETRIG);
        inputs[1] = createInput<app::SvgPort>(Vec(0,0), module, OhMyPendulum::INPUT_2_CHAOS);
        for (int i = 0; i < 2; i++) addInput(inputs[i]);

        outputs[0] = createOutput<app::SvgPort>(Vec(0,0), module, OhMyPendulum::OUTPUT_1);
        outputs[1] = createOutput<app::SvgPort>(Vec(0,0), module, OhMyPendulum::OUTPUT_2);
        outputs[2] = createOutput<app::SvgPort>(Vec(0,0), module, OhMyPendulum::OUTPUT_3);
        outputs[3] = createOutput<app::SvgPort>(Vec(0,0), module, OhMyPendulum::OUTPUT_4);
        for (int i = 0; i < 4; i++) addOutput(outputs[i]);

        addChild(createWidget<ScrewBlack>(Vec(0, 0)));
        addChild(createWidget<ScrewBlack>(Vec(mw - RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewBlack>(Vec(0, mh - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewBlack>(Vec(mw - RACK_GRID_WIDTH, mh - RACK_GRID_WIDTH)));

        pendulumPos = mm2px(Vec(40.68f, 86.99f));
        float halfW = mm2px(Vec(LINK_W_MM * 0.5f, 0.0f)).x;
        float linkH = mm2px(Vec(0.0f, LINK_H_MM)).y;

        link1 = new widget::TransformWidget();
        link1->box.pos = Vec(pendulumPos.x - halfW, pendulumPos.y);
        link2 = new widget::TransformWidget();
        link2->box.pos = Vec(0.0f, linkH);

        auto* link2Svg = new widget::SvgWidget();
        link2Svg->setSvg(Svg::load(asset::plugin(pluginInstance, "res/Arm-2.svg")));
        link2->addChild(link2Svg);
        link1->addChild(link2);

        auto* link1Svg = new widget::SvgWidget();
        link1Svg->setSvg(Svg::load(asset::plugin(pluginInstance, "res/Arm-1.svg")));
        link1->addChild(link1Svg);
        addChild(link1);

        auto* cover = new widget::SvgWidget();
        cover->setSvg(Svg::load(asset::plugin(pluginInstance, "res/Cover.svg")));
        cover->box.pos = pendulumPos - cover->box.size / 2.0f;
        addChild(cover);

        updateTheme();
    }

    void updateCvKnobs() {
        if (!module || !timeKnob || !skewKnob) return;
        OhMyPendulum* m = static_cast<OhMyPendulum*>(module);
        timeKnob->disabled = m->timeCvActive.load(std::memory_order_relaxed);
        skewKnob->disabled = m->skewCvActive.load(std::memory_order_relaxed);
    }

    void step() override {
        ModuleWidget::step();
        updatePendulum();
        updateCvKnobs();
    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Theme"));
        const char* names[] = {"Mono", "Creme", "Minty", "Mage", "Love"};
        for (int i = 0; i < THEME_COUNT; i++) {
            menu->addChild(createCheckMenuItem(names[i], "",
                [=]() { return curTheme() == i; },
                [=]() {
                    previewTheme = i;
                    if (module) static_cast<OhMyPendulum*>(module)->theme.store(i, std::memory_order_relaxed);
                    updateTheme();
                }
            ));
        }
    }
};

Model* modelOhMyPendulum = createModel<OhMyPendulum, OhMyPendulumWidget>("OhMyPendulum");

/// HOLA :) <3
