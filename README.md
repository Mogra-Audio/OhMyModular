# OhMyModular

**Carefully crafted open-source VCV Rack modules with a twist.**

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![VCV Rack](https://img.shields.io/badge/VCV%20Rack-2.0+-brightgreen.svg)](https://vcvrack.com)
![Build Status](https://img.shields.io/github/actions/workflow/status/Mogra-Audio/OhMyModular/build.yml)

---

## About

OhMyModular is a small, open-source collection of VCV Rack modules by Mogra Audio. The first release contains two modules: OhMyPendulum, a chaotic CV generator built on a real-time double pendulum simulation, and OhMyPendulum Expander, which adds CV control and Cartesian outputs to it.

Most modulation sources are either periodic or random. A double pendulum is neither. It is deterministic, bounded, and extremely sensitive to its initial conditions, so it produces motion that never quite repeats. OhMyModular turns that motion into control voltages.

The plugin is released under the GPL-3.0-or-later. Release builds are published on the Releases page; the source and development happen on GitHub.

---

## Modules Overview

**OhMyPendulum** | Chaotic double pendulum CV generator module with a twist. 

OMP is based on a double pendulum simulation, it transforms its angular motion to control voltage values. Each arm provides a separate cv value.
Unlike traditional LFOs or random sources, OMP produces **mathematically-rich modulation** derived from real-time physics simulation using the 4th-order Runge-Kutta numerical integration method (RK4). The panel also includes a live animated **Double Pendulum** drawn directly on the module.

**OMP Features:**

- **Real-time** physics simulation & animation of a double pendulum system.
- **Conservative** energy for constant motion & generation.
- **5** control knobs for initial conditions and behavior shaping.
- **2 INs:** CHAOS CV & RETRIG for external control (+2 with an expander module: TIME CV & SKEW CV).
- **4 OUTs:** Angle CV outputs, smooth & discrete for each arm (+4 with an expander module).
- **CHAOS** — recursive parameter that uses a “ghost” double pendulum to modulate the main one in the background. It pushes the simulation into an extreme condition and induces even more chaotic behavior. This allows you to get more sophisticated output voltage patterns.
- **Expander** module to extend the possibilities.

#### Themes

Five themes are available from the context menu: Mono, Creme, Minty, Mage, and Love. The theme is saved with the patch, so a rack can contain modules with different themes. The expander inherits the theme of the main module it is attached to.

### OhMyPendulum Expander

OhMyPendulum Expander is a 6 HP module that attaches to the right of OhMyPendulum. It adds CV control over Time and Skew and exposes the Cartesian position of the pendulum as four outputs. It has no audio path of its own; it reads the state published by the main module.

---

## Installation

### NOTICE

OhMyModular is not in the VCV Library yet. Once it is accepted, installing from the Library will be the recommended way to keep it up to date.

### Release builds

Release builds are published on the [Releases](https://github.com/Mogra-Audio/OhMyModular/releases) page. Download the `.vcvplugin` for your platform and place it in the Rack user plugins directory:

| Platform | Directory |
| --- | --- |
| macOS (Apple Silicon) | `~/Library/Application Support/Rack2/plugins-mac-arm64/` |
| macOS (Intel) | `~/Library/Application Support/Rack2/plugins-mac-x64/` |
| Windows | `%LOCALAPPDATA%\Rack2\plugins-win-x64\` |
| Linux | `~/.local/share/Rack2/plugins-linux-x64/` |

Rack extracts `.vcvplugin` packages from that directory when it starts. Close Rack, copy the file, and start Rack again.

---

### Build from source

You will need the Rack SDK, Git, and a C++ toolchain.

```bash
git clone https://github.com/Mogra-Audio/OhMyModular.git
cd OhMyModular

export RACK_DIR=/path/to/Rack-SDK

make -j4
make test
make install
```

`make install` builds the plugin and copies the `.vcvplugin` archive into the Rack user plugins directory. Restart Rack after it finishes. `make dist` creates the archive without installing it.

---

## 📦 Requirements

- **VCV Rack 2.0+** (Free or Pro)
- **macOS** (ARM64/Intel), **Windows**, or **Linux**

---

## How It Works

The pendulum is described by two angles, θ₁ and θ₂, and their conjugate momenta, p₁ and p₂. The equations of motion come from the Hamiltonian of a double pendulum with point masses and massless rods. The state is advanced with RK4 at the audio sample rate, with the Time control scaling the integration step. Because the equations are conservative, the motion keeps its energy unless the user changes the masses or the chaos control.

The Chaos control runs a second, virtual double pendulum in the background. Its angles and momenta are mixed into the main state with a nonlinear gain. Turning Chaos up moves the system away from its current trajectory; turning it down starts a short settle phase. The settle servo brings the total mechanical energy back to the value it had before Chaos was engaged, adding or removing kinetic energy as needed, so the pendulum returns to its previous energy range instead of stopping.

The Skew control redistributes mass between the two arms. The total mass is constant; the angular velocities are preserved when the masses change, so the motion does not jump when the knob is turned.

The outputs are the normalized angles of the two arms. The Smooth outputs follow the continuous angle; the Discrete outputs quantize it to 1/12 V, which is one semitone in Rack's pitch standard.

The expander converts the two angles into Cartesian positions. The elbow is at (L₁ sin θ₁, −L₁ cos θ₁); the tip adds the second arm. Both use the same scale, so an XY scope shows the actual geometry rather than two independently normalized signals. The CV inputs on the expander are absolute overrides: 0–10 V maps to the full parameter range, and the corresponding knob is locked while a cable is patched.

## Musical Applications

- Slow filter or wavetable modulation. The smooth outputs move continuously and never repeat exactly.
- Two related modulation lanes. Feed Angle I and Angle II into different destinations; the arms are coupled, so the signals move together without being identical.
- Pitched patterns. The discrete outputs are quantized to semitones; pass them through a quantizer or sample-and-hold for scales.
- Clocked phrases. Patch a clock divider into Retrigger to restart the pendulum from the same pose every few bars.
- XY motion. The expander's X/Y pairs drive an XY scope, or two parameters that should follow a geometric path.
- Gates. Run a smooth output through a comparator to turn the pendulum's crossings into triggers.
- Chaos sweeps. Modulate the Chaos CV input with an envelope or LFO to move the system in and out of chaos without touching the knob.

## Compatibility and Notes

- OhMyModular is built for VCV Rack 2.x. The current release is 2.0.0.
- The modules are monophonic; polyphonic cables are not supported.
- The expander only works when it is placed directly to the right of OhMyPendulum. It inherits the main module's theme; without a parent its outputs are 0 V.
- The main angle outputs are slew-limited. The expander's XY outputs are not: they follow the true position, which is what an XY display should show.
- Loading a patch resets the pendulum from the Angle knobs. This is intentional: the patch stores the initial conditions, not the live state.
- Chaos is not a random source. It is deterministic; the same settings and starting state produce the same result.

---

## Contributing

Bug reports and pull requests are welcome. For larger changes, please open an issue first so we can discuss the approach.

```bash
git clone https://github.com/Mogra-Audio/OhMyModular.git
cd OhMyModular
git checkout -b feature/your-feature-name

export RACK_DIR=/path/to/Rack-SDK
make -j4
make test
```

The project has a small Rack-free test target (`make test`) for the expander math and shared helpers. Please run it before opening a pull request. Source comments are in English, and the shared math lives in `src/dsp/` and plain headers.

---

## Documentation
- [Modules Manual](docs/OhMyModular-Manual.pdf)
- [VCV Rack Manual](https://vcvrack.com/manual/)
- [Plugin Development Guide](https://vcvrack.com/manual/PluginDevelopmentTutorial)
- [DSP Guide](https://vcvrack.com/manual/PluginGuide#DSP-library)

---

## License

OhMyModular is free software, released under the GNU General Public License v3.0 or later. See [LICENSE](LICENSE) for details.

```
OhMyModular — a collection of VCV Rack modules
Copyright (C) 2026 Mogra Audio

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
```

---

## Contact

- GitHub: [Mogra Audio](https://github.com/Mogra-Audio)
- Donate: [Boosty](https://boosty.to/keithrose/donate)

---

⭐ Star this repo if you find it useful!

<div align="center">

Made with 🤍 and chaos by **Mogra Audio x keithrosseau**

</div>
