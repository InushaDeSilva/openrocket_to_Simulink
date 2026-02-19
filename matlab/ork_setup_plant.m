function ork_setup_plant(orkFilePath, varargin)
%ORK_SETUP_PLANT  Load .ork data and set up workspace variables for Simulink.
%
%   ORK_SETUP_PLANT(ORKFILEPATH) imports the .ork file and populates the
%   base workspace with variables needed by the Simulink 6-DOF plant model.
%
%   ORK_SETUP_PLANT(ORKFILEPATH, Name, Value, ...) forwards all options to
%   ORK_IMPORT and additionally accepts:
%
%     'Workspace' - 'base' (default) or 'caller' — where to put variables
%
%   Variables created in the workspace (v2.1):
%     ork               - Full imported data struct
%     ork_meta          - Metadata struct (coordinate conventions, versions)
%
%     ── Motor ──
%     thrust_time       - Thrust curve time breakpoints [s]
%     thrust_force      - Thrust values [N]
%     motor_name        - Motor designation
%     motor_manufacturer - Motor manufacturer name
%     motor_total_impulse - Total impulse [N·s]
%     motor_avg_thrust  - Average thrust estimate [N]
%     motor_max_thrust  - Max thrust estimate [N]
%
%     ── Mass & Inertia (time-varying) ──
%     mass_time         - Mass time breakpoints [s]
%     mass_total        - Total mass vs time [kg]
%     mass_propellant   - Propellant mass vs time [kg]
%     Ixx_vs_time       - Axial MOI vs time [kg-m²]
%     Iyy_vs_time       - Pitch MOI vs time [kg-m²]
%     Izz_vs_time       - Yaw MOI vs time [kg-m²]
%     I_tensor_vs_time  - 3×3×N inertia tensor (v2.1)
%     dIxx_dt … dIzz_dt - MOI rate of change [kg-m²/s]
%     cg_x_vs_time      - CG x-position vs time [m]
%     m_structure, I_structure_xx/yy/zz - Structural inertia
%     m_motor_launch, I_motor_launch_xx/yy/zz - Motor launch inertia
%     m_motor_vs_time, cg_motor_x_vs_time - Motor mass/CG vs time
%
%     ── Aerodynamics (3D: Mach × AoA × Altitude, v2.1) ──
%     mach_bp           - 1×P Mach breakpoints
%     aoa_bp            - 1×Q AoA breakpoints [rad]
%     alt_bp            - 1×R Altitude breakpoints [m] (v2.1)
%     CD_table          - P×Q×R drag coefficient table
%     CN_table, Cm_table, Croll_table, CrollDamp_table, CrollForce_table
%
%     ── Geometry (invariant, rocket_* aliases) ──
%     ref_area, ref_length, rocket_length
%
%     ── Scenario (per-flight, scenario_* aliases, v2.1) ──
%     wind_speed_avg, wind_direction, wind_turbulence_intensity
%     or_rail_exit_velocity  (v2.1)
%
%     ── Scalars ──
%     burn_time, mass_launch, mass_burnout, n_motors
%     launch_rod_length, launch_rod_angle, fin0_cant_angle_rad
%
%   Example:
%     ork_setup_plant('my_rocket.ork', 'RunSim', true);
%     % Now open your Simulink model — workspace variables are ready
%
%   See also: ork_import, ork_update_plant_data, ork_coord_transforms

    %% Parse workspace option
    p = inputParser;
    addParameter(p, 'Workspace', 'base', @ischar);
    addParameter(p, 'SimIndex', 0);
    addParameter(p, 'MachMax', 3.0);
    addParameter(p, 'MachStep', 0.01);
    addParameter(p, 'AoAMax', 15.0);
    addParameter(p, 'AoAStep', 1.0);
    addParameter(p, 'MassDt', 0.01);
    addParameter(p, 'RunSim', false);
    addParameter(p, 'Altitude', 0.0);
    addParameter(p, 'AltMax', 5000.0);
    addParameter(p, 'AltStep', 500.0);
    addParameter(p, 'AeroModel', 'auto');
    addParameter(p, 'JarPath', '');
    addParameter(p, 'JavaHome', '');
    addParameter(p, 'OutputFile', '');
    addParameter(p, 'UseCache', true);
    parse(p, varargin{:});
    ws = p.Results.Workspace;

    % Forward all options except 'Workspace' to ork_import
    importArgs = {};
    fields = {'SimIndex','MachMax','MachStep','AoAMax','AoAStep', ...
              'MassDt','RunSim','Altitude','AltMax','AltStep', ...
              'AeroModel','JarPath','JavaHome','OutputFile','UseCache'};
    for i = 1:numel(fields)
        importArgs = [importArgs, {fields{i}, p.Results.(fields{i})}]; %#ok<AGROW>
    end

    %% Import data
    data = ork_import(orkFilePath, importArgs{:});

    %% Assign to workspace
    assignVar(ws, 'ork', data);

    % ── Metadata ──
    if isfield(data, 'meta')
        assignVar(ws, 'ork_meta', data.meta);
    end

    % ── Motor ──
    assignVar(ws, 'thrust_time', data.thrust_time);
    assignVar(ws, 'thrust_force', data.thrust_force);
    safeAssign(ws, data, 'motor_name');
    safeAssign(ws, data, 'motor_manufacturer');
    safeAssign(ws, data, 'motor_total_impulse');
    safeAssign(ws, data, 'motor_avg_thrust');
    safeAssign(ws, data, 'motor_max_thrust');

    % ── Mass (time-varying) ──
    assignVar(ws, 'mass_time', data.mass_time);
    assignVar(ws, 'mass_total', data.mass_total);
    assignVar(ws, 'mass_propellant', data.mass_propellant);

    % ── Inertia (time-varying) ──
    assignVar(ws, 'Ixx_vs_time', data.Ixx_vs_time);
    assignVar(ws, 'Iyy_vs_time', data.Iyy_vs_time);
    assignVar(ws, 'Izz_vs_time', data.Izz_vs_time);
    safeAssign(ws, data, 'I_tensor_vs_time');  % v2.1: 3×3×N

    % dI/dt
    safeAssign(ws, data, 'dIxx_dt');
    safeAssign(ws, data, 'dIyy_dt');
    safeAssign(ws, data, 'dIzz_dt');

    % CG (time-varying)
    assignVar(ws, 'cg_x_vs_time', data.cg_x_vs_time);

    % Separated mass components
    safeAssign(ws, data, 'm_structure');
    safeAssign(ws, data, 'm_motor_launch');
    safeAssign(ws, data, 'I_structure_xx');
    safeAssign(ws, data, 'I_structure_yy');
    safeAssign(ws, data, 'I_structure_zz');
    safeAssign(ws, data, 'I_motor_launch_xx');
    safeAssign(ws, data, 'I_motor_launch_yy');
    safeAssign(ws, data, 'I_motor_launch_zz');
    safeAssign(ws, data, 'm_motor_vs_time');
    safeAssign(ws, data, 'cg_motor_x_vs_time');

    % ── Aerodynamics (3D: Mach × AoA × Altitude) ──
    assignVar(ws, 'mach_bp', data.mach_bp);
    assignVar(ws, 'aoa_bp', data.aoa_bp);
    safeAssign(ws, data, 'alt_bp');   % v2.1
    assignVar(ws, 'CD_table', data.CD_table);
    assignVar(ws, 'CN_table', data.CN_table);
    assignVar(ws, 'Cm_table', data.Cm_table);
    assignVar(ws, 'Croll_table', data.Croll_table);
    safeAssign(ws, data, 'CrollDamp_table');
    safeAssign(ws, data, 'CrollForce_table');

    % ── Geometry (invariant) ──
    assignVar(ws, 'ref_area', data.ref_area);
    assignVar(ws, 'ref_length', data.ref_length);
    safeAssign(ws, data, 'rocket_length');

    % ── Scalars ──
    assignVar(ws, 'burn_time', data.burn_time);
    assignVar(ws, 'mass_launch', data.mass_launch);
    assignVar(ws, 'mass_burnout', data.mass_burnout);
    assignVar(ws, 'n_motors', data.n_motors);

    % ── Launch config ──
    assignVar(ws, 'launch_rod_length', data.launch_rod_length);
    assignVar(ws, 'launch_rod_angle', data.launch_rod_angle);

    % Fin cant angle
    if isfield(data, 'fin0_cant_angle_rad')
        assignVar(ws, 'fin0_cant_angle_rad', data.fin0_cant_angle_rad);
    else
        assignVar(ws, 'fin0_cant_angle_rad', 0);
    end

    % ── Scenario / wind (v2.1) ──
    safeAssign(ws, data, 'wind_speed_avg');
    safeAssign(ws, data, 'wind_direction');
    safeAssign(ws, data, 'wind_turbulence_intensity');
    safeAssign(ws, data, 'or_rail_exit_velocity');

    % ── Coordinate transform helper ──
    % Import coord transforms for Aerospace Blockset mapping
    if exist('ork_coord_transforms', 'file')
        assignVar(ws, 'coord', ork_coord_transforms());
    end

    fprintf('Workspace variables set (%d vars). Ready for Simulink.\n', ...
        numel(fieldnames(data)) + 2);  % +2 for ork and ork_meta/coord
end

function assignVar(ws, name, value)
    if strcmp(ws, 'base')
        assignin('base', name, value);
    else
        assignin('caller', name, value);
    end
end

function safeAssign(ws, data, fieldName)
    if isfield(data, fieldName)
        assignVar(ws, fieldName, data.(fieldName));
    end
end
