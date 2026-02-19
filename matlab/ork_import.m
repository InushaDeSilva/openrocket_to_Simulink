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
%     'AltMax'       - Maximum altitude for 3D aero tables (default: 5000)
%     'AltStep'      - Altitude step for 3D aero tables (default: 500)
%     'AeroModel'    - Aero model: 'auto' or 'barrowman' (default: 'auto')
%     'JarPath'      - Path to ork-exporter.jar (auto-detected by default)
%     'JavaHome'     - Path to Java 17+ installation (default: '' = system)
%     'OutputFile'   - Custom path for intermediate .mat file
%     'UseCache'     - Skip re-export if .ork hash unchanged (default: true)
%
%   Output:
%     DATA is a struct containing fields organized by category:
%
%     Metadata:
%       meta            - Struct with export metadata, coordinate conventions,
%                         version info, file hash, export settings.
%
%     Mass & Inertia (time-varying):
%       mass_time       - 1xN time breakpoints [s]
%       mass_total      - 1xN total mass vs time [kg]
%       I_tensor_vs_time - 3x3xN inertia tensor (diagonal, zeros off-diag)
%       Ixx_vs_time, Iyy_vs_time, Izz_vs_time - 1xN MOI vs time
%       dIxx_dt, dIyy_dt, dIzz_dt  - 1xN time derivatives of MOI
%       cg_x_vs_time    - 1xN CG x-position vs time [m]
%
%     Motor:
%       thrust_time     - 1xM time breakpoints [s] (strictly monotonic)
%       thrust_force    - 1xM thrust values [N] (includes burnout tail)
%       motor_name      - Motor designation (string)
%       motor_manufacturer - Motor manufacturer name
%       motor_total_impulse - Computed total impulse [N·s]
%       motor_avg_thrust, motor_max_thrust - Thrust estimates [N]
%
%     Aerodynamics (3D tables: Mach x AoA x Altitude):
%       mach_bp         - 1xP Mach breakpoints
%       aoa_bp          - 1xQ AoA breakpoints [rad]
%       alt_bp          - 1xR Altitude breakpoints [m]
%       CD_table        - PxQxR drag coefficient table
%       CN_table, Cm_table, Croll_table, CrollDamp_table, CrollForce_table
%
%     Scenario (per-flight conditions):
%       wind_speed_avg, wind_direction, wind_turbulence_intensity
%       scenario_* prefixed aliases for all per-flight variables
%
%     Geometry (invariant):
%       ref_area, ref_length, rocket_length
%       rocket_* prefixed aliases for invariant data
%
%     Validation (if RunSim=true):
%       or_max_altitude, or_rail_exit_velocity, etc.
%       or_time, or_altitude, or_velocity, etc.
%
%   See also: ork_setup_plant, ork_update_plant_data, ork_coord_transforms

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
    addParameter(p, 'AltMax', 5000.0, @isnumeric);
    addParameter(p, 'AltStep', 500.0, @isnumeric);
    addParameter(p, 'AeroModel', 'auto', @ischar);
    addParameter(p, 'JarPath', '', @ischar);
    addParameter(p, 'JavaHome', '', @ischar);
    addParameter(p, 'OutputFile', '', @ischar);
    addParameter(p, 'UseCache', true, @islogical);
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

    %% MATLAB-side SHA-256 caching
    if opts.UseCache && isfile(matPath)
        try
            currentHash = compute_sha256(orkFilePath);
            cached = load(matPath, 'meta');
            if isfield(cached, 'meta') && isfield(cached.meta, 'ork_sha256')
                if strcmp(cached.meta.ork_sha256, currentHash)
                    fprintf('Cache HIT: .ork file unchanged (SHA-256 match). Loading cached .mat.\n');
                    data = load(matPath);
                    print_sanity_banner(data);
                    return;
                else
                    fprintf('Cache MISS: .ork file changed. Re-exporting.\n');
                end
            end
        catch
            % Cache check failed, proceed with export
        end
    end

    %% Build the Java command
    javaCmd = 'java';
    if ~isempty(opts.JavaHome)
        javaCmd = fullfile(opts.JavaHome, 'bin', 'java');
    end

    cmd = sprintf(['%s -jar "%s" --ork "%s" --out "%s" --sim %d ' ...
        '--mach-max %.4f --mach-step %.4f --aoa-max %.4f --aoa-step %.4f ' ...
        '--mass-dt %.6f --altitude %.2f --alt-max %.2f --alt-step %.2f ' ...
        '--aero-model %s'], ...
        javaCmd, jarPath, orkFilePath, matPath, opts.SimIndex, ...
        opts.MachMax, opts.MachStep, opts.AoAMax, opts.AoAStep, ...
        opts.MassDt, opts.Altitude, opts.AltMax, opts.AltStep, opts.AeroModel);

    if opts.RunSim
        cmd = [cmd, ' --run-sim'];
    end

    %% Run the exporter
    fprintf('Running ork-exporter v2.1...\n');
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

    %% Print sanity banner
    print_sanity_banner(data);

    fprintf('Import complete.\n');
end

%% ---- Sanity Banner ----
function print_sanity_banner(data)
    fprintf('\n╔══════════════════════════════════════════════════╗\n');
    fprintf('║         ORK Import Sanity Check (v2.1)          ║\n');
    fprintf('╠══════════════════════════════════════════════════╣\n');
    if isfield(data, 'meta')
        fprintf('║  Exporter:  %-37s║\n', data.meta.exporter_version);
        fprintf('║  OR:        %-37s║\n', data.meta.openrocket_version);
    end
    if isfield(data, 'motor_name')
        fprintf('║  Motor:     %-37s║\n', data.motor_name);
    end
    if isfield(data, 'motor_manufacturer')
        fprintf('║  Mfg:       %-37s║\n', data.motor_manufacturer);
    end
    if isfield(data, 'mass_launch')
        fprintf('║  Launch:    %-7.4f kg                          ║\n', data.mass_launch);
    end
    if isfield(data, 'mass_burnout')
        fprintf('║  Burnout:   %-7.4f kg                          ║\n', data.mass_burnout);
    end
    if isfield(data, 'burn_time')
        fprintf('║  Burn:      %-7.3f s                           ║\n', data.burn_time);
    end
    if isfield(data, 'motor_total_impulse')
        fprintf('║  Impulse:   %-7.2f N·s                         ║\n', data.motor_total_impulse);
    end
    if isfield(data, 'CD_table')
        dims = size(data.CD_table);
        if numel(dims) == 3
            fprintf('║  Aero:      %dx%dx%d (Mach×AoA×Alt)              ║\n', dims(1), dims(2), dims(3));
        else
            fprintf('║  Aero:      %dx%d (Mach×AoA)                    ║\n', dims(1), dims(2));
        end
    end
    if isfield(data, 'I_tensor_vs_time')
        dims = size(data.I_tensor_vs_time);
        fprintf('║  Tensor:    %dx%dx%d (3×3×time)                  ║\n', dims(1), dims(2), dims(3));
    end
    if isfield(data, 'wind_speed_avg')
        fprintf('║  Wind:      %-5.1f m/s @ %-5.1f°                  ║\n', ...
            data.wind_speed_avg, rad2deg(data.wind_direction));
    end
    fprintf('╚══════════════════════════════════════════════════╝\n\n');
end

%% ---- SHA-256 Hash ----
function hash = compute_sha256(filePath)
    % Compute SHA-256 of a file using Java (available in MATLAB)
    md = java.security.MessageDigest.getInstance('SHA-256');
    fis = java.io.FileInputStream(java.io.File(filePath));
    buf = zeros(1, 8192, 'int8');
    while true
        n = fis.read(buf);
        if n <= 0, break; end
        md.update(buf(1:n), 0, n);
    end
    fis.close();
    hashBytes = typecast(md.digest(), 'uint8');
    hash = sprintf('%02x', hashBytes);
end

%% Helper functions

function fullPath = which_or_full(filePath)
    w = which(filePath);
    if ~isempty(w)
        fullPath = w;
    else
        fullPath = filePath;
    end
    if ~java.io.File(fullPath).isAbsolute()
        fullPath = fullfile(pwd, fullPath);
    end
end

function jarPath = find_jar()
    thisDir = fileparts(mfilename('fullpath'));
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
    jarPath = fullfile(thisDir, '..', 'exporter', 'build', 'libs', 'ork-exporter.jar');
end

function validate_data(data)
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
        if any(dt <= 0)
            warning('ork_import:nonMonotonic', 'mass_time is not strictly monotonically increasing');
        end
    end
    if isfield(data, 'thrust_time')
        dt = diff(data.thrust_time);
        if any(dt <= 0)
            warning('ork_import:nonMonotonic', 'thrust_time is not strictly monotonically increasing');
        end
    end
    if isfield(data, 'CD_table')
        if any(isnan(data.CD_table(:)))
            warning('ork_import:nanCD', 'CD_table contains NaN values');
        end
    end
    % v2.1: validate 3D table dimensions
    if isfield(data, 'CD_table') && isfield(data, 'alt_bp')
        dims = size(data.CD_table);
        if numel(dims) == 3
            if dims(3) ~= numel(data.alt_bp)
                warning('ork_import:dimMismatch', 'CD_table dim3 (%d) != alt_bp length (%d)', ...
                    dims(3), numel(data.alt_bp));
            end
        end
    end
    % v2.1: validate inertia tensor
    if isfield(data, 'I_tensor_vs_time')
        dims = size(data.I_tensor_vs_time);
        if numel(dims) ~= 3 || dims(1) ~= 3 || dims(2) ~= 3
            warning('ork_import:tensorShape', 'I_tensor_vs_time should be 3x3xN');
        end
    end
    if ~isfield(data, 'meta')
        warning('ork_import:noMetadata', 'meta struct not found - may be using older exporter version');
    end
end
