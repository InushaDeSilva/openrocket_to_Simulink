# OpenRocket → Simulink Integration (v2.1)

Automated pipeline to import OpenRocket `.ork` rocket designs directly into MATLAB/Simulink for 6-DOF flight simulation, roll-control design, and Aerospace Blockset integration.

## Architecture

```
┌──────────────┐    ork-exporter.jar v2.1   ┌──────────────┐
│  .ork file   │ ───────────────────────▶   │  .mat file   │
│  (ZIP/XML)   │   Java CLI (headless)      │  (MAT 5.0)   │
└──────────────┘   OR core + 3D aero        └──────┬───────┘
                                                    │
                                           ork_import.m
                                           (SHA-256 cache)
                                                    │
                                           ┌────────▼───────┐
                                           │  Simulink Model │
                                           │  (6-DOF Plant)  │
                                           └────────┬───────┘
                                           Mach×AoA×Alt LUTs
                                           I_tensor (3×3×N)
                                           ork_coord_transforms
```

## Simulink Quick Start (v2.1)

The primary use case is building a Simulink plant model. Here's the fastest path:

```matlab
% 1. Add MATLAB toolbox to path
addpath('matlab');

% 2. Set up workspace (one command does everything)
ork_setup_plant('my_rocket.ork');

% 3. Now Simulink has these workspace variables ready:
%    thrust_time/force, mass_time/total, Ixx/Iyy/Izz_vs_time,
%    I_tensor_vs_time (3×3×N), mach_bp, aoa_bp, alt_bp,
%    CD_table (P×Q×R), CN/Cm/Croll_table, wind_speed_avg, etc.

% 4. For Aerospace Blockset coordinate conversion:
coord = ork_coord_transforms();
F_aero = coord.or_to_aero_body([Fx; Fy; Fz]);  % OR → NED body
```

### Build the Exporter First

```bash
export JAVA_HOME=/path/to/java17
./gradlew :exporter:shadowJar
# Output: exporter/build/libs/ork-exporter.jar
```

## What's New in v2.1

| Feature | Description |
|---------|-------------|
| **3D Aero Tables** | All aero coefficients are now Mach×AoA×Altitude (3D), with `alt_bp` breakpoints |
| **Motor Identity** | `motor_manufacturer`, `motor_total_impulse`, `motor_avg_thrust`, `motor_max_thrust` |
| **Inertia Tensor** | `I_tensor_vs_time` — 3×3×N full tensor (diagonal Ixx/Iyy/Izz, zero off-diag) |
| **Wind & Scenario** | `wind_speed_avg`, `wind_direction`, `wind_turbulence_intensity`, ISA flag |
| **Invariant/Scenario Split** | `rocket_*` prefixed aliases for invariant data, `scenario_*` for per-flight |
| **Rail Exit Velocity** | `or_rail_exit_velocity` from simulation |
| **Coordinate Transforms** | `ork_coord_transforms.m` maps OR body frame ↔ Aerospace Blockset NED body |
| **SHA-256 Caching** | MATLAB-side cache: skip re-export if .ork file hash unchanged |
| **Sanity Banner** | Import/validate print a summary box with motor, mass, impulse, table dims |
| **Altitude CLI** | `--alt-max` and `--alt-step` control altitude dimension of 3D tables |
| **Sim Listing** | Invalid `--sim` index now prints all available simulations with motor info |

## Coordinate Conventions

### OpenRocket Body Frame
- **x**: positive aft (from nose tip)
- **y**: right
- **z**: up
- CG, CP measured from nose tip along x [m]

### Aerospace Blockset Body Frame (NED)
- **x**: positive forward
- **y**: right
- **z**: down

### Conversion

The rotation matrix from OR to Aerospace Blockset is `diag([-1, 1, -1])`:

```matlab
coord = ork_coord_transforms();

% Convert a force vector
F_aero = coord.or_to_aero_body([F_x_or; F_y_or; F_z_or]);

% Convert inertia tensor
I_aero = coord.transform_inertia(I_or);  % diag: same values, frame-agnostic

% Convert CG position
cg_aero_x = coord.or_cg_to_aero(cg_x_from_nose, rocket_length);
% Returns distance from nose = rocket_length - cg_x_or (measured from aft)

% Get the raw rotation matrix
R = coord.rotation_matrix();  % [-1 0 0; 0 1 0; 0 0 -1]
```

### World Frame
- OR uses z-up (launch site frame)
- Aerospace Blockset uses NED (North-East-Down)

## Invariant vs. Scenario Data

v2.1 exports data in two logical partitions:

### Rocket (invariant across flights)
Prefixed with `rocket_*` in the .mat file:
- `rocket_ref_area`, `rocket_ref_length`, `rocket_length`
- All mass/inertia schedules, fin geometry, thrust curve
- These don't change unless you modify the .ork design

### Scenario (per-flight conditions)
Prefixed with `scenario_*` in the .mat file:
- `scenario_launch_rod_length`, `scenario_launch_rod_angle`
- `scenario_wind_speed_avg`, `scenario_wind_direction`
- `scenario_wind_turbulence_intensity`
- These change when you modify simulation conditions

Both partitions are also available under their original names (e.g., `ref_area`, `wind_speed_avg`).

## What Gets Exported

### Aerodynamics (3D: Mach × AoA × Altitude)

| Variable | Shape | Description |
|----------|-------|-------------|
| `mach_bp` | 1×P | Mach number breakpoints |
| `aoa_bp` | 1×Q | Angle of attack breakpoints [rad] |
| `alt_bp` | 1×R | Altitude breakpoints [m] |
| `CD_table` | P×Q×R | Total drag coefficient |
| `CN_table` | P×Q×R | Normal force coefficient |
| `Cm_table` | P×Q×R | Pitching moment coefficient |
| `Croll_table` | P×Q×R | Total roll moment coefficient |
| `CrollDamp_table` | P×Q×R | Roll damping coefficient |
| `CrollForce_table` | P×Q×R | Roll forcing coefficient |

Default grid: Mach 0–3 (step 0.01), AoA 0–15° (step 1°), Altitude 0–5000 m (step 500 m).

### Mass & Inertia (time-varying)

| Variable | Shape | Description |
|----------|-------|-------------|
| `mass_time` | 1×N | Time breakpoints [s] |
| `mass_total` | 1×N | Total mass [kg] |
| `mass_propellant` | 1×N | Propellant mass [kg] |
| `Ixx_vs_time` | 1×N | Axial MOI [kg·m²] |
| `Iyy_vs_time` | 1×N | Pitch MOI [kg·m²] |
| `Izz_vs_time` | 1×N | Yaw MOI [kg·m²] |
| `I_tensor_vs_time` | 3×3×N | Full inertia tensor (diagonal, zeros off-diag) |
| `dIxx_dt`, `dIyy_dt`, `dIzz_dt` | 1×N | MOI time derivatives |
| `cg_x_vs_time` | 1×N | CG position from nose [m] |

### Motor Identity

| Variable | Description |
|----------|-------------|
| `motor_name` | Motor designation (e.g., "H128") |
| `motor_manufacturer` | Manufacturer name |
| `motor_total_impulse` | Computed total impulse [N·s] (trapezoidal integration) |
| `motor_avg_thrust` | Average thrust estimate [N] |
| `motor_max_thrust` | Maximum thrust estimate [N] |
| `n_motors` | Number of motors |
| `burn_time` | Motor burn time [s] |

### Wind & Atmosphere (scenario)

| Variable | Description |
|----------|-------------|
| `wind_speed_avg` | Average wind speed [m/s] |
| `wind_direction` | Wind direction [rad] |
| `wind_turbulence_intensity` | Turbulence intensity factor |
| `isa_atmosphere` | true if ISA atmosphere model |
| `launch_humidity` | Launch relative humidity |
| `wind_model_type` | Wind model class name |

### Simulation Results (with `--run-sim`)

| Variable | Description |
|----------|-------------|
| `or_time`, `or_altitude`, `or_velocity` | Flight profile |
| `or_thrust`, `or_mass`, `or_mach`, `or_cd` | Motor & aero traces |
| `or_aoa`, `or_roll_rate` | Attitude |
| `or_cg`, `or_cp`, `or_stability` | Stability |
| `or_rail_exit_velocity` | Velocity at launch rail exit [m/s] |
| `or_max_altitude`, `or_max_velocity`, etc. | Peak values |

### Metadata

The `meta` struct contains: `exporter_version`, `openrocket_version`, `ork_sha256`, `export_timestamp`, coordinate conventions for body/CG/AoA, coefficient definitions for CD/CN/Cm/Croll, altitude and Mach ranges, Aerospace Blockset coordinate mapping info.

## CLI Options

```
--ork <path>           Path to .ork file (required)
--out <path>           Output .mat file (default: same name as .ork)
--sim <index>          Simulation index, 0-based (default: 0)
--mach-max <value>     Max Mach for aero tables (default: 3.0)
--mach-step <value>    Mach step size (default: 0.01)
--aoa-max <value>      Max AoA in degrees (default: 15.0)
--aoa-step <value>     AoA step in degrees (default: 1.0)
--mass-dt <value>      Mass sampling time step (default: 0.01 s)
--altitude <value>     Reference altitude for Reynolds number (default: 0 m)
--alt-max <value>      Max altitude for 3D aero [m] (default: 5000)  [NEW v2.1]
--alt-step <value>     Altitude step for 3D aero [m] (default: 500)  [NEW v2.1]
--aero-model <model>   Aero calculator: 'auto' or 'barrowman' (default: auto)
--run-sim              Run OR simulation for validation data
```

## MATLAB API

### `ork_import(orkFile, Name, Value, ...)`

Main import function. Returns a struct with all data.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SimIndex` | 0 | Simulation index (0-based) |
| `MachMax` | 3.0 | Max Mach |
| `AoAMax` | 15.0 | Max AoA [deg] |
| `AltMax` | 5000 | Max altitude [m] for 3D tables |
| `AltStep` | 500 | Altitude step [m] |
| `RunSim` | false | Run OR simulation |
| `UseCache` | true | Skip re-export if .ork hash unchanged |

### `ork_setup_plant(orkFile, Name, Value, ...)`

Calls `ork_import` and assigns all variables to workspace.

### `ork_create_plant_block([libName])`

Creates a Simulink library with masked block. Aero LUTs are n-D (Mach × AoA × Altitude). Block has 3 inputs: Mach, AoA, Altitude.

### `ork_update_plant_data(modelName, orkFile, ...)`

Re-imports .ork and updates workspace + any ORK Plant masked blocks.

### `ork_validate(orkFile, ...)`

Runs validation with sanity banner, 6-panel comparison plots, rail exit overlay, and pass/fail metrics.

### `ork_coord_transforms()`

Returns a struct of function handles for OR ↔ Aerospace Blockset mapping.

## Common Failures

| Symptom | Cause | Fix |
|---------|-------|-----|
| `exportFailed exit code 1` | Java not found or wrong version | Set `JavaHome` or `JAVA_HOME` to Java 17+ |
| `jarNotFound` | Shadow JAR not built | Run `./gradlew :exporter:shadowJar` |
| `Simulation index out of range` | .ork has fewer sims | Use `--sim 0` or check the printed sim list |
| n-D LUT error in Simulink | Table dimensions mismatch | Ensure `alt_bp`, `mach_bp`, `aoa_bp` match table dims |
| Coordinate sign mismatch | OR vs Aerospace Blockset frame | Use `ork_coord_transforms()` — x and z are flipped |
| Stale data after .ork edit | Cache returned old data | Use `'UseCache', false` or delete the `_export.mat` |

## Validation

```matlab
results = ork_validate('my_rocket.ork');
```

Produces a 6-panel figure and quantitative metrics:

| Metric | Threshold | Description |
|--------|-----------|-------------|
| Mass launch error | < 1% | Relative error in launch mass |
| Mass burnout error | < 1% | Relative error at burnout |
| Thrust NRMSE | < 5% | Normalized RMS error of thrust |
| CG launch error | < 5 mm | Absolute CG position error |
| 3D table dims | match | Table dimensions vs breakpoint lengths |

## Requirements

- **Java 17+** for the exporter
- **MATLAB R2020b+** for the MATLAB scripts
- **Simulink** with n-D Lookup Table blocks
- **Aerospace Blockset** (recommended for 6-DOF)
- Build tools: Gradle 9+ (included via wrapper)

## Project Structure

```
exporter/                               Java exporter subproject
  build.gradle                          Gradle config with shadowJar
  src/main/java/.../OrkExporter.java    Main CLI class (v2.1)
matlab/
  ork_import.m                          Import .ork → MATLAB struct
  ork_setup_plant.m                     Populate Simulink workspace vars
  ork_create_plant_block.m              Create masked Simulink library block
  ork_update_plant_data.m               Refresh data without block rebuild
  ork_validate.m                        Quantitative comparison plots
  ork_coord_transforms.m                OR ↔ Aerospace Blockset coordinate maps [NEW]
  README.md                             This file
```
