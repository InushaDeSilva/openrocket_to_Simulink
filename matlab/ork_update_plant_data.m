function ork_update_plant_data(modelName, orkFilePath, varargin)
%ORK_UPDATE_PLANT_DATA  Update an existing Simulink model's data from .ork.
%
%   ORK_UPDATE_PLANT_DATA(MODELNAME, ORKFILEPATH) re-imports the .ork file
%   and updates the workspace variables (and any mask workspace data) without
%   regenerating the Simulink library block.
%
%   ORK_UPDATE_PLANT_DATA(MODELNAME, ORKFILEPATH, Name, Value, ...) forwards
%   options to ork_import.
%
%   This function is for the "data update" workflow:
%     1. Create block once: ork_create_plant_block()
%     2. Modify your .ork in OpenRocket
%     3. Update data only: ork_update_plant_data('my_model', 'rocket.ork')
%     4. Re-run your simulation — no block regeneration needed
%
%   Steps performed:
%     1. Calls ork_import() with the given .ork file and options
%     2. Updates the base workspace via all known variable names
%     3. If the model is loaded and contains masked ORK Plant blocks,
%        updates their mask workspace variables too
%     4. Reports what was updated
%
%   Example:
%     ork_update_plant_data('my_sim_model', 'rocket_v2.ork', 'RunSim', true);
%
%   See also: ork_import, ork_setup_plant, ork_create_plant_block

    %% Parse inputs
    p = inputParser;
    addRequired(p, 'modelName', @ischar);
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
    parse(p, modelName, orkFilePath, varargin{:});
    opts = p.Results;

    % Forward all options except modelName/orkFilePath to ork_import
    importArgs = {};
    fields = {'SimIndex','MachMax','MachStep','AoAMax','AoAStep', ...
              'MassDt','RunSim','Altitude','AltMax','AltStep', ...
              'AeroModel','JarPath','JavaHome','OutputFile','UseCache'};
    for i = 1:numel(fields)
        importArgs = [importArgs, {fields{i}, opts.(fields{i})}]; %#ok<AGROW>
    end

    %% Import fresh data
    fprintf('Re-importing .ork file: %s\n', orkFilePath);
    data = ork_import(orkFilePath, importArgs{:});

    %% Update base workspace variables
    fprintf('Updating base workspace variables...\n');
    assignin('base', 'ork', data);

    % ── Core variables ──
    varList = {'thrust_time', 'thrust_force', 'mass_time', 'mass_total', ...
               'mass_propellant', 'Ixx_vs_time', 'Iyy_vs_time', 'Izz_vs_time', ...
               'cg_x_vs_time', 'mach_bp', 'aoa_bp', 'CD_table', 'CN_table', ...
               'Cm_table', 'Croll_table', 'ref_area', 'ref_length', ...
               'burn_time', 'mass_launch', 'mass_burnout', 'n_motors', ...
               'launch_rod_length', 'launch_rod_angle'};

    % ── v2.0 variables ──
    v2Vars = {'dIxx_dt', 'dIyy_dt', 'dIzz_dt', ...
              'm_structure', 'm_motor_launch', ...
              'I_structure_xx', 'I_structure_yy', 'I_structure_zz', ...
              'I_motor_launch_xx', 'I_motor_launch_yy', 'I_motor_launch_zz', ...
              'm_motor_vs_time', 'cg_motor_x_vs_time', 'fin0_cant_angle_rad', ...
              'CrollDamp_table', 'CrollForce_table'};

    % ── v2.1 variables ──
    v21Vars = {'alt_bp', 'I_tensor_vs_time', ...
               'motor_name', 'motor_manufacturer', ...
               'motor_total_impulse', 'motor_avg_thrust', 'motor_max_thrust', ...
               'wind_speed_avg', 'wind_direction', 'wind_turbulence_intensity', ...
               'or_rail_exit_velocity', 'rocket_length', ...
               'scenario_launch_rod_length', 'scenario_launch_rod_angle', ...
               'scenario_wind_speed_avg', 'scenario_wind_direction', ...
               'scenario_wind_turbulence_intensity', ...
               'rocket_ref_area', 'rocket_ref_length'};

    allVars = [varList, v2Vars, v21Vars];

    nUpdated = 0;
    for i = 1:numel(allVars)
        vname = allVars{i};
        if isfield(data, vname)
            assignin('base', vname, data.(vname));
            nUpdated = nUpdated + 1;
        end
    end

    % Metadata
    if isfield(data, 'meta')
        assignin('base', 'ork_meta', data.meta);
        nUpdated = nUpdated + 1;
    end

    % Coordinate transform helper
    if exist('ork_coord_transforms', 'file')
        assignin('base', 'coord', ork_coord_transforms());
        nUpdated = nUpdated + 1;
    end

    fprintf('  Updated %d workspace variables.\n', nUpdated);

    %% Update masked blocks in the model (if loaded)
    if bdIsLoaded(modelName)
        fprintf('Model ''%s'' is loaded. Searching for ORK Plant blocks...\n', modelName);
        try
            blocks = find_system(modelName, 'RegExp', 'on', ...
                'MaskType', '.*', 'LookUnderMasks', 'all');

            nBlocks = 0;
            for b = 1:numel(blocks)
                blkPath = blocks{b};
                mask = Simulink.Mask.get(blkPath);
                if isempty(mask), continue; end
                try
                    mask.getParameter('ork_file_path');
                catch
                    continue;  % Not an ORK block
                end

                fprintf('  Updating block: %s\n', blkPath);
                set_param(blkPath, 'ork_file_path', sprintf('''%s''', orkFilePath));
                nBlocks = nBlocks + 1;
            end

            if nBlocks > 0
                fprintf('  Updated %d ORK Plant block(s).\n', nBlocks);
            else
                fprintf('  No ORK Plant masked blocks found in model.\n');
            end
        catch ME
            fprintf('  Warning: Could not search model for blocks: %s\n', ME.message);
        end
    else
        fprintf('Model ''%s'' is not loaded. Workspace variables updated only.\n', modelName);
    end

    fprintf('Data update complete. Re-run your simulation to use the new data.\n');
end
