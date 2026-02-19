function data = ork_import(orkFilePath, varargin)
%ORK_IMPORT  Import an OpenRocket .ork file into MATLAB for Simulink.
%
%   DATA = ORK_IMPORT(ORKFILEPATH) loads the specified .ork file using
%   the ork-exporter Java helper and returns a struct with all rocket
%   parameters ready for use in Simulink.
%
%   DATA = ORK_IMPORT(ORKFILEPATH, Name, Value, ...) accepts optional
%   name-value pairs:
%
%   Name-Value Arguments:
%     'SimIndex'     - Simulation index (0-based, default: 0)
%     'MachMax'      - Maximum Mach number for CD table (default: 3.0)
%     'MachStep'     - Mach step for CD table (default: 0.01)
%     'AoAMax'       - Maximum AoA in degrees (default: 15.0)
%     'AoAStep'      - AoA step in degrees (default: 1.0)
%     'MassDt'       - Time step for mass sampling (default: 0.01 s)
%     'RunSim'       - Run OR simulation for validation (default: false)
%     'Altitude'     - Reference altitude for CD computation (default: 0)
%     'JarPath'      - Path to ork-exporter.jar (auto-detected by default)
%     'JavaHome'     - Path to Java 17+ installation (default: '' = system)
%     'OutputFile'   - Custom path for intermediate .mat file
%
%   Output:
%     DATA is a struct containing fields:
%
%     Mass & Inertia:
%       mass_launch, mass_burnout, mass_structure  - Scalar masses [kg]
%       mass_time       - 1×N time breakpoints [s]
%       mass_total      - 1×N total mass vs time [kg]
%       mass_propellant - 1×N propellant mass vs time [kg]
%       Ixx_launch, Iyy_launch, Izz_launch - Launch MOIs [kg·m²]
%       Ixx_vs_time, Iyy_vs_time, Izz_vs_time - 1×N MOI vs time
%       cg_launch_x     - CG x-position at launch [m]
%       cg_x_vs_time    - 1×N CG x-position vs time [m]
%
%     Motor:
%       thrust_time     - 1×M time breakpoints [s]
%       thrust_force    - 1×M thrust values [N]
%       n_motors        - Number of motors
%       burn_time       - Motor burn time [s]
%       motor_name      - Motor designation (string)
%
%     Aerodynamics (2D tables: Mach × AoA):
%       mach_bp         - 1×P Mach breakpoints
%       aoa_bp          - 1×Q AoA breakpoints [rad]
%       CD_table        - P×Q drag coefficient table
%       CN_table        - P×Q normal force coefficient table
%       Cm_table        - P×Q pitching moment coefficient table
%       Croll_table     - P×Q roll moment coefficient table
%       CDaxial_table   - P×Q axial drag coefficient table
%
%     Geometry:
%       ref_area, ref_length - Reference area [m²] and length [m]
%       rocket_length   - Total rocket length [m]
%       fin_count, fin_root_chord, fin_span, fin_sweep
%
%     Simulation Settings:
%       launch_rod_length, launch_rod_angle, launch_rod_direction
%       launch_altitude, launch_latitude, launch_longitude
%       launch_temperature, launch_pressure
%       time_step
%
%     Validation (if RunSim=true):
%       or_max_altitude, or_max_velocity, or_max_acceleration
%       or_time, or_altitude, or_velocity, etc.
%
%   Example:
%     data = ork_import('my_rocket.ork', 'RunSim', true);
%     fprintf('Launch mass: %.3f kg\n', data.mass_launch);
%     fprintf('OR max altitude: %.1f m\n', data.or_max_altitude);
%
%   See also: ork_setup_plant

    %% Parse inputs
    p = inputParser;
    addRequired(p, 'orkFilePath', @ischar);
    addParameter(p, 'SimIndex', 0, @isnumeric);
    addParameter(p, 'MachMax', 3.0, @isnumeric);
    addParameter(p, 'MachStep', 0.01, @isnumeric);
    addParameter(p, 'AoAMax', 15.0, @isnumeric);
    addParameter(p, 'AoAStep', 1.0, @isnumeric);
    addParameter(p, 'MassDt', 0.01, @isnumeric);
    addParameter(p, 'RunSim', false, @islogical);
    addParameter(p, 'Altitude', 0.0, @isnumeric);
    addParameter(p, 'JarPath', '', @ischar);
    addParameter(p, 'JavaHome', '', @ischar);
    addParameter(p, 'OutputFile', '', @ischar);
    parse(p, orkFilePath, varargin{:});
    opts = p.Results;

    %% Resolve paths
    orkFilePath = which_or_full(orkFilePath);
    if ~isfile(orkFilePath)
        error('ork_import:fileNotFound', 'ORK file not found: %s', orkFilePath);
    end

    % Find the JAR
    jarPath = opts.JarPath;
    if isempty(jarPath)
        jarPath = find_jar();
    end
    if ~isfile(jarPath)
        error('ork_import:jarNotFound', ...
            'ork-exporter.jar not found at: %s\nBuild it with: ./gradlew :exporter:shadowJar', jarPath);
    end

    % Output MAT file
    if isempty(opts.OutputFile)
        [orkDir, orkName, ~] = fileparts(orkFilePath);
        matPath = fullfile(orkDir, [orkName, '_export.mat']);
    else
        matPath = opts.OutputFile;
    end

    %% Build the Java command
    javaCmd = 'java';
    if ~isempty(opts.JavaHome)
        javaCmd = fullfile(opts.JavaHome, 'bin', 'java');
    end

    cmd = sprintf('%s -jar "%s" --ork "%s" --out "%s" --sim %d --mach-max %.4f --mach-step %.4f --aoa-max %.4f --aoa-step %.4f --mass-dt %.6f --altitude %.2f', ...
        javaCmd, jarPath, orkFilePath, matPath, opts.SimIndex, ...
        opts.MachMax, opts.MachStep, opts.AoAMax, opts.AoAStep, ...
        opts.MassDt, opts.Altitude);

    if opts.RunSim
        cmd = [cmd, ' --run-sim'];
    end

    %% Run the exporter
    fprintf('Running ork-exporter...\n');
    [status, output] = system(cmd);
    if status ~= 0
        error('ork_import:exportFailed', ...
            'ork-exporter failed (exit code %d):\n%s', status, output);
    end

    % Print the export log (filtered)
    lines = strsplit(output, newline);
    for i = 1:numel(lines)
        line = lines{i};
        if contains(line, 'OrkExporter')
            fprintf('  %s\n', line);
        end
    end

    %% Load the MAT file
    if ~isfile(matPath)
        error('ork_import:outputNotFound', 'Expected output file not found: %s', matPath);
    end

    data = load(matPath);
    fprintf('Loaded %d variables from %s\n', numel(fieldnames(data)), matPath);

    %% Validate key fields
    validate_data(data);

    fprintf('Import complete.\n');
end

%% Helper functions

function fullPath = which_or_full(filePath)
    % If the file is on the MATLAB path, resolve it; otherwise treat as-is
    w = which(filePath);
    if ~isempty(w)
        fullPath = w;
    else
        fullPath = filePath;
    end
    % Convert to absolute path
    if ~java.io.File(fullPath).isAbsolute()
        fullPath = fullfile(pwd, fullPath);
    end
end

function jarPath = find_jar()
    % Search for ork-exporter.jar relative to this script
    thisDir = fileparts(mfilename('fullpath'));

    % Try common locations
    candidates = {
        fullfile(thisDir, '..', 'exporter', 'build', 'libs', 'ork-exporter.jar')
        fullfile(thisDir, 'ork-exporter.jar')
        fullfile(thisDir, '..', 'ork-exporter.jar')
        'ork-exporter.jar'
    };

    for i = 1:numel(candidates)
        if isfile(candidates{i})
            jarPath = candidates{i};
            return;
        end
    end

    % Default to the build output location
    jarPath = fullfile(thisDir, '..', 'exporter', 'build', 'libs', 'ork-exporter.jar');
end

function validate_data(data)
    % Basic sanity checks on imported data
    if isfield(data, 'mass_launch') && data.mass_launch <= 0
        warning('ork_import:invalidMass', 'Launch mass is non-positive: %g', data.mass_launch);
    end
    if isfield(data, 'mass_total')
        if any(isnan(data.mass_total))
            warning('ork_import:nanMass', 'mass_total contains NaN values');
        end
        if any(data.mass_total < 0)
            warning('ork_import:negativeMass', 'mass_total contains negative values');
        end
    end
    if isfield(data, 'mass_time')
        dt = diff(data.mass_time);
        if any(dt < 0)
            warning('ork_import:nonMonotonic', 'mass_time is not monotonically increasing');
        end
    end
    if isfield(data, 'thrust_time')
        dt = diff(data.thrust_time);
        if any(dt < 0)
            warning('ork_import:nonMonotonic', 'thrust_time is not monotonically increasing');
        end
    end
    if isfield(data, 'CD_table')
        if any(isnan(data.CD_table(:)))
            warning('ork_import:nanCD', 'CD_table contains NaN values');
        end
    end
end
