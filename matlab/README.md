# OpenRocket → Simulink Integration

Automated pipeline to import OpenRocket `.ork` rocket designs directly into MATLAB/Simulink for 6-DOF flight simulation and control design.

## Architecture

```
┌──────────────┐     ork-exporter.jar      ┌──────────────┐
│  .ork file   │ ──────────────────────▶    │  .mat file   │
│  (ZIP/XML)   │   Java CLI (headless)     │  (MAT 5.0)   │
└──────────────┘   Uses OR core library    └──────┬───────┘
                                                   │
                                          ork_import.m
                                                   │
                                          ┌────────▼───────┐
                                          │  Simulink Model │
                                          │  (6-DOF Plant)  │
                                          └────────────────┘
```

## Quick Start

### 1. Build the Exporter

```bash
# From the repo root (requires Java 17+):
export JAVA_HOME=/path/to/java17
./gradlew :exporter:shadowJar
```

This produces `exporter/build/libs/ork-exporter.jar`.

### 2. Command-Line Export

```bash
java -jar exporter/build/libs/ork-exporter.jar \
  --ork my_rocket.ork \
  --out my_rocket.mat \
  --run-sim
```

### 3. MATLAB Import

```matlab
% Add the matlab/ folder to your path
addpath('matlab');

% Import a .ork file
data = ork_import('my_rocket.ork', 'RunSim', true);

% Or set up workspace variables for Simulink directly
ork_setup_plant('my_rocket.ork');
```

### 4. Simulink Masked Block

```matlab
% Create the library (one-time)
ork_create_plant_block();

% Then copy the 'ORK Plant Loader' block into your model.
% Set the .ork file path in the mask dialog.
% Connect Mach and AoA inputs, wire outputs to your 6-DOF block.
```

## What Gets Exported

| Category | Variables | Description |
|----------|-----------|-------------|
| **Thrust** | `thrust_time`, `thrust_force` | Motor thrust curve (N vs s) |
| **Mass** | `mass_time`, `mass_total`, `mass_propellant` | Time-varying mass (kg) |
| **Inertia** | `Ixx_vs_time`, `Iyy_vs_time`, `Izz_vs_time` | Time-varying MOIs (kg·m²) |
| **CG** | `cg_x_vs_time` | CG position vs time (m) |
| **Aero** | `CD_table`, `CN_table`, `Cm_table`, `Croll_table` | 2D Mach×AoA tables |
| **Breakpoints** | `mach_bp`, `aoa_bp` | Breakpoints for aero tables |
| **Geometry** | `ref_area`, `ref_length`, `rocket_length` | Reference dimensions |
| **Fins** | `fin_count`, `fin_root_chord`, `fin_span`, `fin_sweep` | Fin geometry |
| **Motor** | `motor_name`, `n_motors`, `burn_time` | Motor info |
| **Launch** | `launch_rod_length`, `launch_rod_angle`, etc. | Simulation settings |
| **Validation** | `or_max_altitude`, `or_time`, `or_altitude`, ... | OR sim results |

## CLI Options

```
--ork <path>         Path to .ork file (required)
--out <path>         Output .mat file (default: same name as .ork)
--sim <index>        Simulation index, 0-based (default: 0)
--mach-max <value>   Max Mach for CD table (default: 3.0)
--mach-step <value>  Mach step size (default: 0.01)
--aoa-max <value>    Max AoA in degrees (default: 15.0)
--aoa-step <value>   AoA step in degrees (default: 1.0)
--mass-dt <value>    Mass sampling time step (default: 0.01 s)
--altitude <value>   Reference altitude for Reynolds number (default: 0 m)
--run-sim            Run OR simulation for validation data
```

## How It Works

1. **ork-exporter.jar** bootstraps OpenRocket's core library headlessly (no GUI) using Guice dependency injection.
2. Loads the `.ork` file via `GeneralRocketLoader` → `OpenRocketDocument`.
3. Extracts:
   - **Thrust curve** from `ThrustCurveMotor.getTimePoints()/getThrustPoints()`
   - **Mass vs time** by sampling `Motor.getTotalMass(t)` at each time step, combined with structural mass from `MassCalculator`
   - **Inertia vs time** from motor unit inertias scaled by time-varying mass, combined with structural inertia via parallel axis theorem
   - **CD(Mach, AoA)** by evaluating `BarrowmanCalculator.getAerodynamicForces()` over a 2D grid
   - **Reference geometry** from `FlightConfiguration.getReferenceArea()/getReferenceLength()`
4. Writes everything to a MAT 5.0 file using the [MFL library](https://github.com/HebiRobotics/MFL).

## Validation

Run `ork_validate('rocket.ork')` to compare the exported data against OpenRocket's own simulation results. This produces a 4-panel figure showing thrust, mass, CD, and trajectory comparisons.

## Requirements

- **Java 17+** for the exporter
- **MATLAB R2020b+** for the MATLAB scripts
- **Simulink** for the masked block (Aerospace Blockset recommended for 6-DOF)
- Build tools: Gradle 9+ (included via wrapper)

## Project Structure

```
exporter/                          Java exporter subproject
  build.gradle                     Gradle config with shadowJar
  src/main/java/.../OrkExporter.java   Main CLI class
matlab/
  ork_import.m                     Import .ork → MATLAB struct
  ork_setup_plant.m                Populate Simulink workspace vars
  ork_create_plant_block.m         Create masked Simulink library block
  ork_validate.m                   Comparison plots (OR vs export)
```
