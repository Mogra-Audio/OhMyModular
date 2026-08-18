#include "plugin.hpp"
#include <cmath>
#include <atomic>

////////////////////////////////////////////////////////////////////////////////
/// DSP CONSTANTS
////////////////////////////////////////////////////////////////////////////////
static constexpr double G = 9.81;
static constexpr double L1 = 1.0;
static constexpr double L2 = 1.0;
static constexpr double M1_DFLT = 1.0;
static constexpr double M2_DFLT = 1.0;

static constexpr double ANGLE_TO_VOLT = 5.0 / M_PI;

static constexpr double IMPULSE_1 = 0.05;
static constexpr double IMPULSE_2 = 0.03;

static constexpr float KNOB_MIN_ANGLE = -0.83f * M_PI;
static constexpr float KNOB_MAX_ANGLE =  0.83f * M_PI;

// Chaos
static constexpr double GHOST_THETA1 = 2.09439510239;
static constexpr double GHOST_THETA2 = 2.79252680319;
static constexpr double CHAOS_DEAD = 0.1;
static constexpr double CHAOS_INJ_ANGLE = 0.8;
static constexpr double CHAOS_INJ_MOM = 6.0;
static constexpr double CHAOS_REFERENCE_SAMPLE_RATE = 48000.0;
static constexpr double P_MAX = 25.0;
static constexpr double MASS_EPS = 1e-6;
static constexpr int GHOST_SUBSTEPS_MAX = 5;
static constexpr int BLEND_FRAMES = 256;

static constexpr int THEME_COUNT = 5;

// SVG link dimensions
static constexpr float LINK_W_MM = 1.162f;
static constexpr float LINK_H_MM = 4.089f;

////////////////////////////////////////////////////////////////////////////////
/// HELPERS
////////////////////////////////////////////////////////////////////////////////
static inline double normalizeAngle(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0.0) a += 2.0 * M_PI;
    return a - M_PI;
}

static inline double lerpAngle(double from, double to, double t) {
    double diff = to - from;
    if (diff > M_PI)  diff -= 2.0 * M_PI;
    if (diff < -M_PI) diff += 2.0 * M_PI;
    return from + diff * t;
}

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

    int theme = 0;

    // Pendulum state
    double theta1 = 0.0;
    double theta2 = 0.0;
    double pTheta1 = 0.0;
    double pTheta2 = 0.0;

    // Display mirrors (audio thread writes, UI thread reads)
    std::atomic<double> dispTheta1{0.0};
    std::atomic<double> dispTheta2{0.0};

    // Last applied masses (SKEW omega preservation)
    double lastM1 = M1_DFLT;
    double lastM2 = M2_DFLT;

    // Ghost pendulum
    double gTheta1 = GHOST_THETA1;
    double gTheta2 = GHOST_THETA2;
    double gP1 = 0.0;
    double gP2 = 0.0;

    bool simulationRunning = false;
    std::atomic<bool> resetStateRequested{true};

    // Angle knob hold (set from UI thread, read from audio thread)
    std::atomic<int> angleHoldCount{0};

    // Chaos snapshot/restore
    double savedTheta1 = 0.0;
    double savedTheta2 = 0.0;
    double savedP1 = 0.0;
    double savedP2 = 0.0;
    double blendFromTheta1 = 0.0;
    double blendFromTheta2 = 0.0;
    int    blendCounter = 0;
    bool   chaosWasActive = false;

    // Retrigger
    dsp::SchmittTrigger retrigTrigger;

    // Angle value tracking — catches dropdown / precise entry
    float lastAngle1Param = 0.f;
    float lastAngle2Param = 0.f;

    // Launch delay after knob release (seconds)
    double stableTime = 0.0;
    static constexpr double LAUNCH_DELAY_SEC = 0.085;

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
    }
    /// HAMILTONIAN FORMALISM
    void computeDerivatives(double th1, double th2, double p1, double p2,
                            double m1, double m2, double timeScale,
                            double& dTh1, double& dTh2, double& dP1, double& dP2) {
        double sin1 = std::sin(th1);
        double sin2 = std::sin(th2);
        double cosDiff = std::cos(th1 - th2);
        double sinDiff = std::sin(th1 - th2);

        double I1 = (m1 + m2) * L1 * L1;
        double I2 = m2 * L2 * L2;
        double I12 = m2 * L1 * L2;

        double denom = I1 * I2 - I12 * I12 * cosDiff * cosDiff;
        denom = std::max(denom, 1e-10);

        double omega1 = (I2 * p1 - I12 * cosDiff * p2) / denom;
        double omega2 = (I1 * p2 - I12 * cosDiff * p1) / denom;

        dTh1 = omega1 * timeScale;
        dTh2 = omega2 * timeScale;

        double coupling = I12 * omega1 * omega2 * sinDiff;
        dP1 = (-coupling - (m1 + m2) * G * L1 * sin1) * timeScale;
        dP2 = ( coupling - m2 * G * L2 * sin2) * timeScale;
    }
    /// RK4 INTEGRATION
    void rk4StepState(double& th1, double& th2, double& p1, double& p2,
                      double dt, double m1, double m2, double timeScale) {
        double dT1_1, dT2_1, dP1_1, dP2_1;
        computeDerivatives(th1, th2, p1, p2, m1, m2, timeScale, dT1_1, dT2_1, dP1_1, dP2_1);

        double dT1_2, dT2_2, dP1_2, dP2_2;
        computeDerivatives(th1 + 0.5*dt*dT1_1, th2 + 0.5*dt*dT2_1,
                           p1  + 0.5*dt*dP1_1, p2  + 0.5*dt*dP2_1,
                           m1, m2, timeScale, dT1_2, dT2_2, dP1_2, dP2_2);

        double dT1_3, dT2_3, dP1_3, dP2_3;
        computeDerivatives(th1 + 0.5*dt*dT1_2, th2 + 0.5*dt*dT2_2,
                           p1  + 0.5*dt*dP1_2, p2  + 0.5*dt*dP2_2,
                           m1, m2, timeScale, dT1_3, dT2_3, dP1_3, dP2_3);

        double dT1_4, dT2_4, dP1_4, dP2_4;
        computeDerivatives(th1 + dt*dT1_3, th2 + dt*dT2_3,
                           p1  + dt*dP1_3, p2  + dt*dP2_3,
                           m1, m2, timeScale, dT1_4, dT2_4, dP1_4, dP2_4);

        th1 += dt * (dT1_1 + 2.0*dT1_2 + 2.0*dT1_3 + dT1_4) / 6.0;
        th2 += dt * (dT2_1 + 2.0*dT2_2 + 2.0*dT2_3 + dT2_4) / 6.0;
        p1  += dt * (dP1_1 + 2.0*dP1_2 + 2.0*dP1_3 + dP1_4) / 6.0;
        p2  += dt * (dP2_1 + 2.0*dP2_2 + 2.0*dP2_3 + dP2_4) / 6.0;
    }

    /// GHOST STEP
    void stepGhost(double dt, double m1, double m2, int substeps) {
        double gdt = dt / (double)substeps;
        for (int i = 0; i < substeps; i++) {
            double dT1, dT2, dP1, dP2;
            computeDerivatives(gTheta1, gTheta2, gP1, gP2, m1, m2, 1.0,
                               dT1, dT2, dP1, dP2);
            gTheta1 += gdt * dT1;
            gTheta2 += gdt * dT2;
            gP1     += gdt * dP1;
            gP2     += gdt * dP2;
        }
        checkGhostStability();
    }

    /// CHAOS INJECTION
    void applyChaos(double chaos, double sampleTime) {
        if (chaos <= CHAOS_DEAD) return;

        double eff = (chaos - CHAOS_DEAD) / (1.0 - CHAOS_DEAD);
        double inj = eff * eff * eff * eff;
        inj *= sampleTime * CHAOS_REFERENCE_SAMPLE_RATE;

        theta1 += inj * CHAOS_INJ_ANGLE * std::sin(gTheta1);
        theta2 += inj * CHAOS_INJ_ANGLE * std::sin(gTheta2);
        pTheta1 += inj * CHAOS_INJ_MOM * gP1;
        pTheta2 += inj * CHAOS_INJ_MOM * gP2;

        pTheta1 = P_MAX * std::tanh(pTheta1 / P_MAX);
        pTheta2 = P_MAX * std::tanh(pTheta2 / P_MAX);
    }

    /// STABILITY CHECK (MAIN+GHOST)
    void checkStability() {
        if (std::isnan(theta1)  || std::isinf(theta1)  ||
            std::isnan(theta2)  || std::isinf(theta2)  ||
            std::isnan(pTheta1) || std::isinf(pTheta1) ||
            std::isnan(pTheta2) || std::isinf(pTheta2)) {
            theta1 = theta2 = pTheta1 = pTheta2 = 0.0;
        }
    }

    void checkGhostStability() {
        if (std::isnan(gTheta1) || std::isinf(gTheta1) ||
            std::isnan(gTheta2) || std::isinf(gTheta2) ||
            std::isnan(gP1)     || std::isinf(gP1)     ||
            std::isnan(gP2)     || std::isinf(gP2)) {
            gTheta1 = GHOST_THETA1;
            gTheta2 = GHOST_THETA2;
            gP1 = gP2 = 0.0;
        }
    }

    void saveChaosSnapshot() {
        savedTheta1 = theta1;
        savedTheta2 = theta2;
        savedP1 = pTheta1;
        savedP2 = pTheta2;
    }

    void initializeStateFromAngles(float angle1Param, float angle2Param) {
        theta1 = angle1Param * 2.0 * M_PI;
        theta2 = angle2Param * 2.0 * M_PI;
        pTheta1 = IMPULSE_1;
        pTheta2 = IMPULSE_2;

        simulationRunning = true;
        stableTime = 0;

        blendCounter = 0;
        chaosWasActive = false;
        saveChaosSnapshot();
    }

    void restoreChaosSnapshot() {
        theta1 = savedTheta1;
        theta2 = savedTheta2;
        pTheta1 = savedP1;
        pTheta2 = savedP2;
    }

    /// TIME SCALE CURVE
    static double computeTimeScale(float timeParam) {
        return 0.2 + timeParam * 1.6
             + std::max(0.0f, timeParam - 0.5f) * 10.4;
    }

    /// USER ACTIONS
    bool handleUserActions(float angle1Param, float angle2Param, double sampleTime) {
        bool stateExplicitlySet = false;

        // Retrig (Schmitt rising edge)
        if (retrigTrigger.process(inputs[INPUT_1_RETRIG].getVoltage())) {
            theta1 = angle1Param * 2.0 * M_PI;
            theta2 = angle2Param * 2.0 * M_PI;
            pTheta1 = IMPULSE_1;
            pTheta2 = IMPULSE_2;
            simulationRunning = true;
            stableTime = 0.0;
            stateExplicitlySet = true;
        }

        // Angle value change (dropdown / precise keyboard entry)
        bool angleHeld = angleHoldCount.load(std::memory_order_relaxed) > 0;
        bool angleValueChanged =
            (std::abs(angle1Param - lastAngle1Param) > 1e-6f) ||
            (std::abs(angle2Param - lastAngle2Param) > 1e-6f);
        lastAngle1Param = angle1Param;
        lastAngle2Param = angle2Param;
        if (angleValueChanged && !angleHeld) {
            theta1 = angle1Param * 2.0 * M_PI;
            theta2 = angle2Param * 2.0 * M_PI;
            pTheta1 = IMPULSE_1;
            pTheta2 = IMPULSE_2;
            simulationRunning = true;
            stableTime = 0.0;
            stateExplicitlySet = true;
        }

        // Angle knob hold / launch countdown
        if (angleHeld) {
            // User is holding an angle knob: freeze sim, track knob position
            theta1 = angle1Param * 2.0 * M_PI;
            theta2 = angle2Param * 2.0 * M_PI;
            pTheta1 = 0.0;
            pTheta2 = 0.0;
            simulationRunning = false;
            stableTime = 0;
            stateExplicitlySet = true;
        } else if (!simulationRunning) {
            // Knob released, counting down to launch
            stableTime += sampleTime;
            if (stableTime >= LAUNCH_DELAY_SEC) {
                simulationRunning = true;
                stableTime = 0.0;
                pTheta1 = IMPULSE_1;
                pTheta2 = IMPULSE_2;
            }
        }

        return stateExplicitlySet;
    }

    /// CHAOS SNAPSHOT / RESTORE
    void handleChaosTransition(bool chaosActive, bool stateExplicitlySet) {
        if (stateExplicitlySet) {
            saveChaosSnapshot();
        }
        if (chaosActive) {
            if (!chaosWasActive) {
                saveChaosSnapshot();
                // A new Chaos pass supersedes an unfinished restore fade.
                blendCounter = 0;
            }
        } else if (chaosWasActive) {
            if (stateExplicitlySet) {
                // A retrigger or manual angle set wins over an old snapshot.
                blendCounter = 0;
            } else {
                blendFromTheta1 = normalizeAngle(theta1);
                blendFromTheta2 = normalizeAngle(theta2);
                restoreChaosSnapshot();
                blendCounter = BLEND_FRAMES;
            }
        }
        chaosWasActive = chaosActive;
    }

    /// SKEW: mass distribution + angular velocity preservation
    void updateMasses(float skewParam, double& m1, double& m2) {
        m1 = 0.5 + (1.0 - skewParam) * 1.0;
        m2 = 0.5 + skewParam * 1.0;
        if (std::abs(m1 - lastM1) > MASS_EPS || std::abs(m2 - lastM2) > MASS_EPS) {
            double cosDiff = std::cos(theta1 - theta2);
            double I1o = (lastM1 + lastM2) * L1 * L1;
            double I2o = lastM2 * L2 * L2;
            double I12o = lastM2 * L1 * L2;
            double denom = std::max(I1o * I2o - I12o * I12o * cosDiff * cosDiff, 1e-10);
            double w1 = (I2o * pTheta1 - I12o * cosDiff * pTheta2) / denom;
            double w2 = (I1o * pTheta2 - I12o * cosDiff * pTheta1) / denom;
            double I1n = (m1 + m2) * L1 * L1;
            double I2n = m2 * L2 * L2;
            double I12n = m2 * L1 * L2;
            pTheta1 = I1n * w1 + I12n * cosDiff * w2;
            pTheta2 = I2n * w2 + I12n * cosDiff * w1;
            lastM1 = m1;
            lastM2 = m2;
        }
    }

    /// OUTPUT WITH CROSSFADE
    void renderOutputs() {
        double theta1Norm = normalizeAngle(theta1);
        double theta2Norm = normalizeAngle(theta2);
        if (blendCounter > 0) {
            double t = 1.0 - (double)blendCounter / (double)BLEND_FRAMES;
            theta1Norm = lerpAngle(blendFromTheta1, theta1Norm, t);
            theta2Norm = lerpAngle(blendFromTheta2, theta2Norm, t);
            blendCounter--;
        }
        float out1 = clamp((float)(theta1Norm * ANGLE_TO_VOLT), -5.0f, 5.0f);
        float out4 = clamp((float)(theta2Norm * ANGLE_TO_VOLT), -5.0f, 5.0f);
        float discreteStep = 1.0f / 12.0f;
        float out2 = clamp(std::round((float)(theta1Norm * ANGLE_TO_VOLT) / discreteStep) * discreteStep, -5.0f, 5.0f);
        float out3 = clamp(std::round((float)(theta2Norm * ANGLE_TO_VOLT) / discreteStep) * discreteStep, -5.0f, 5.0f);
        outputs[OUTPUT_1].setVoltage(out1);
        outputs[OUTPUT_2].setVoltage(out2);
        outputs[OUTPUT_3].setVoltage(out3);
        outputs[OUTPUT_4].setVoltage(out4);
    }

    void process(const ProcessArgs& args) override {
        // Read params
        float angle1Param = params[PARAM_ANGLE1].getValue();
        float angle2Param = params[PARAM_ANGLE2].getValue();
        float timeParam   = params[PARAM_TIME].getValue();
        float skewParam   = params[PARAM_SKEW].getValue();
        float chaosParam  = params[PARAM_CHAOS].getValue();

        if (resetStateRequested.exchange(false)) {
            initializeStateFromAngles(angle1Param, angle2Param);
        }

        // Chaos CV
        float chaosCV = inputs[INPUT_2_CHAOS].getVoltage() / 5.0f;
        double chaos = clamp(chaosParam + chaosCV, 0.f, 1.f);
        bool chaosActive = chaos > CHAOS_DEAD;

        // User actions (retrigger / angle change / knob hold / launch)
        bool stateExplicitlySet = handleUserActions(angle1Param, angle2Param, args.sampleTime);

        // Chaos snapshot / restore transitions
        handleChaosTransition(chaosActive, stateExplicitlySet);

        // Masses (SKEW)
        double m1, m2;
        updateMasses(skewParam, m1, m2);

        // Ghost pendulum
        int substeps = 1 + (int)(chaos * (GHOST_SUBSTEPS_MAX - 1));
        stepGhost(args.sampleTime, m1, m2, substeps);

        // Main integration
        if (simulationRunning) {
            rk4StepState(theta1, theta2, pTheta1, pTheta2,
                         args.sampleTime, m1, m2, computeTimeScale(timeParam));
        }

        // Chaos injection + stability
        applyChaos(chaos, args.sampleTime);
        checkStability();

        // Publish to UI (relaxed: need atomicity, not ordering)
        dispTheta1.store(theta1, std::memory_order_relaxed);
        dispTheta2.store(theta2, std::memory_order_relaxed);

        // Outputs
        renderOutputs();
    }

    void onReset(const ResetEvent& e) override {
        Module::onReset(e);
        resetStateRequested.store(true);
    }

    void onRandomize(const RandomizeEvent& e) override {
        Module::onRandomize(e);
        resetStateRequested.store(true);
    }

    json_t* dataToJson() override {
        json_t* rootJ = json_object();
        json_object_set_new(rootJ, "theme", json_integer(theme));
        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        json_t* themeJ = json_object_get(rootJ, "theme");
        if (themeJ) {
            theme = clamp((int)json_integer_value(themeJ), 0, THEME_COUNT - 1);
        }
    }
};

////////////////////////////////////////////////////////////////////////////////
/// ANGLE KNOB — reports mouse hold to module
////////////////////////////////////////////////////////////////////////////////
struct AngleKnob : app::SvgKnob {
    OhMyPendulum* mod = nullptr;
    bool isHeld = false; // UI-thread only, guards against double-decrement

    void onButton(const ButtonEvent& e) override {
        SvgKnob::onButton(e);
        if (!mod) return;
        if (e.button == GLFW_MOUSE_BUTTON_LEFT) {
            if (e.action == GLFW_PRESS && !isHeld) {
                isHeld = true;
                mod->angleHoldCount.fetch_add(1, std::memory_order_relaxed);
            }
            if (e.action == GLFW_RELEASE && isHeld) {
                isHeld = false;
                mod->angleHoldCount.fetch_sub(1, std::memory_order_relaxed);
            }
        }
    }

    // Safety: user drags outside widget and releases
    void onDragEnd(const DragEndEvent& e) override {
        SvgKnob::onDragEnd(e);
        if (mod && isHeld) {
            isHeld = false;
            mod->angleHoldCount.fetch_sub(1, std::memory_order_relaxed);
        }
    }
};



////////////////////////////////////////////////////////////////////////////////
/// WIDGET
////////////////////////////////////////////////////////////////////////////////
struct ThemeInfo {
    const char* panel;
    const char* smallKnob;
    const char* hugeKnob;
    const char* input;
    const char* output;
};

static constexpr ThemeInfo THEMES[] = {
    {
    /// Mono
        "res/Panel-Mono.svg",
        "res/SmallKnob.svg",
        "res/HugeKnob.svg",
        "res/Input.svg",
        "res/Output.svg"
    },
    /// Creme
    {
        "res/themes/theme_Creme/Panel-Creme.svg",
        "res/themes/theme_Creme/SmallKnob.svg",
        "res/themes/theme_Creme/HugeKnob.svg",
        "res/themes/theme_Creme/Input.svg",
        "res/themes/theme_Creme/Output.svg"
    },
    /// Minty
    {
        "res/themes/theme_Minty/Panel-Minty.svg",
        "res/themes/theme_Minty/SmallKnob.svg",
        "res/themes/theme_Minty/HugeKnob.svg",
        "res/themes/theme_Minty/Input.svg",
        "res/themes/theme_Minty/Output.svg"
    },
    /// Mage
    {
        "res/themes/theme_Mage/Panel-Mage.svg",
        "res/themes/theme_Mage/SmallKnob.svg",
        "res/themes/theme_Mage/HugeKnob.svg",
        "res/themes/theme_Mage/Input.svg",
        "res/themes/theme_Mage/Output.svg"
    },
    /// Love
    {
        "res/themes/theme_Love/Panel-Love.svg",
        "res/themes/theme_Love/SmallKnob-1.svg",
        "res/themes/theme_Love/HugeKnob.svg",
        "res/themes/theme_Love/Input-1.svg",
        "res/themes/theme_Love/Output-1.svg"
    }
};

struct OhMyPendulumWidget : ModuleWidget {
    int previewTheme = 0;

    math::Vec knobCenters[5];
    math::Vec inputCenters[2];
    math::Vec outputCenters[4];

    app::SvgKnob* knobs[5];
    app::SvgPort* inputs[2];
    app::SvgPort* outputs[4];

    widget::TransformWidget* link1 = nullptr;
    widget::TransformWidget* link2 = nullptr;
    math::Vec pendulumPos;

    int curTheme() const {
        return module ? static_cast<OhMyPendulum*>(module)->theme : previewTheme;
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

        // Standard knobs
        knobs[2] = createParam<app::SvgKnob>(Vec(0,0), module, OhMyPendulum::PARAM_TIME);
        knobs[3] = createParam<app::SvgKnob>(Vec(0,0), module, OhMyPendulum::PARAM_SKEW);
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

    void step() override {
        ModuleWidget::step();
        updatePendulum();
    }

    void appendContextMenu(Menu* menu) override {
        ModuleWidget::appendContextMenu(menu);
        menu->addChild(new MenuSeparator);
        menu->addChild(createMenuLabel("Theme"));
        const char* names[] = {"Mono", "Creme", "Minty", "Mage", "Love"};
        for (int i = 0; i < 5; i++) {
            menu->addChild(createCheckMenuItem(names[i], "",
                [=]() { return curTheme() == i; },
                [=]() {
                    if (module) static_cast<OhMyPendulum*>(module)->theme = i;
                    updateTheme();
                }
            ));
        }
    }
};

Model* modelOhMyPendulum = createModel<OhMyPendulum, OhMyPendulumWidget>("OhMyPendulum");
