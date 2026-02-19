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
%   Variables created in the workspace:
%     ork               - Full imported data struct
%     thrust_time       - Thrust curve time breakpoints [s]
%     thrust_force      - Thrust values [N]
%     mass_time         - Mass time breakpoints [s]
%     mass_total        - Total mass vs time [kg]
%     mass_propellant   - Propellant mass vs time [kg]
%     Ixx_vs_time       - Axial MOI vs time [kg·m²]
%     Iyy_vs_time       - Pitch MOI vs time [kg·m²]
%     Izz_vs_time       - Yaw MOI vs time [kg·m²]
%     cg_x_vs_time      - CG x-position vs time [m]
%     mach_bp           - Mach breakpoints for 2D aero tables
%     aoa_bp            - AoA breakpoints [rad] for 2D aero tables
%     CD_table          - P×Q drag coefficient table
%     CN_table          - P×Q normal force coefficient table
%     Cm_table          - P×Q pitching moment coefficient table
%     Croll_table       - P×Q roll moment coefficient table
%     ref_area          - Reference area [m²]
%     ref_length        - Reference length [m]
%     burn_time         - Motor burn time [s]
%     mass_launch       - Launch mass [kg]
%     mass_burnout      - Burnout mass [kg]
%     launch_rod_length - Launch rod length [m]
%     launch_rod_angle  - Launch rod angle [rad]
%
%   Example:
%     ork_setup_plant('my_rocket.ork', 'RunSim', true);
%     % Now open your Simulink model — workspace variables are ready
%
%   See also: ork_import

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
    addParameter(p, 'JarPath', '');
    addParameter(p, 'JavaHome', '');
    addParameter(p, 'OutputFile', '');
    parse(p, varargin{:});
    ws = p.Results.Workspace;

    % Forward all options except 'Workspace' to ork_import
    importArgs = {};
    fields = {'SimIndex','MachMax','MachStep','AoAMax','AoAStep', ...
              'MassDt','RunSim','Altitude','JarPath','JavaHome','OutputFile'};
    for i = 1:numel(fields)
        importArgs = [importArgs, {fields{i}, p.Results.(fields{i})}]; %#ok<AGROW>
    end

    %% Import data
    data = ork_import(orkFilePath, importArgs{:});

    %% Assign to workspace
    assignVar(ws, 'ork', data);

    % Thrust
    assignVar(ws, 'thrust_time', data.thrust_time);
    assignVar(ws, 'thrust_force', data.thrust_force);

    % Mass (time-varying)
    assignVar(ws, 'mass_time', data.mass_time);
    assignVar(ws, 'mass_total', data.mass_total);
    assignVar(ws, 'mass_propellant', data.mass_propellant);

    % Inertia (time-varying)
    assignVar(ws, 'Ixx_vs_time', data.Ixx_vs_time);
    assignVar(ws, 'Iyy_vs_time', data.Iyy_vs_time);
    assignVar(ws, 'Izz_vs_time', data.Izz_vs_time);

    % CG (time-varying)
    assignVar(ws, 'cg_x_vs_time', data.cg_x_vs_time);

    % Aero tables (2D: Mach × AoA)
    assignVar(ws, 'mach_bp', data.mach_bp);
    assignVar(ws, 'aoa_bp', data.aoa_bp);
    assignVar(ws, 'CD_table', data.CD_table);
    assignVar(ws, 'CN_table', data.CN_table);
    assignVar(ws, 'Cm_table', data.Cm_table);
    assignVar(ws, 'Croll_table', data.Croll_table);

    % Reference geometry
    assignVar(ws, 'ref_area', data.ref_area);
    assignVar(ws, 'ref_length', data.ref_length);

    % Scalars
    assignVar(ws, 'burn_time', data.burn_time);
    assignVar(ws, 'mass_launch', data.mass_launch);
    assignVar(ws, 'mass_burnout', data.mass_burnout);
    assignVar(ws, 'n_motors', data.n_motors);

    % Launch config
    assignVar(ws, 'launch_rod_length', data.launch_rod_length);
    assignVar(ws, 'launch_rod_angle', data.launch_rod_angle);

    fprintf('Workspace variables set. Ready for Simulink.\n');
end

function assignVar(ws, name, value)
    if strcmp(ws, 'base')
        assignin('base', name, value);
    else
        assignin('caller', name, value);
    end
end
