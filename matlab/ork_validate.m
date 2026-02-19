function ork_validate(orkFilePath, varargin)
%ORK_VALIDATE  Compare OpenRocket simulation results with exported data.
%
%   ORK_VALIDATE(ORKFILEPATH) imports the .ork file with validation enabled
%   and plots comparison charts for thrust, drag, mass, and trajectory.
%
%   ORK_VALIDATE(ORKFILEPATH, Name, Value, ...) passes options to ork_import.
%
%   This function creates 4 subplots:
%     1. Thrust curve (imported vs OR simulation log)
%     2. Mass vs time (exported schedule vs OR log)
%     3. Drag coefficient vs Mach (from CD table at AoA=0°, vs OR log)
%     4. Altitude vs time (OR simulation)
%
%   Example:
%     ork_validate('my_rocket.ork');
%
%   See also: ork_import

    %% Import with validation enabled
    data = ork_import(orkFilePath, 'RunSim', true, varargin{:});

    %% Print summary comparison
    fprintf('\n=== OpenRocket Validation Summary ===\n');
    if isfield(data, 'or_max_altitude')
        fprintf('  Max altitude:     %.1f m\n', data.or_max_altitude);
    end
    if isfield(data, 'or_max_velocity')
        fprintf('  Max velocity:     %.1f m/s\n', data.or_max_velocity);
    end
    if isfield(data, 'or_max_acceleration')
        fprintf('  Max acceleration: %.1f m/s²\n', data.or_max_acceleration);
    end
    if isfield(data, 'or_max_mach')
        fprintf('  Max Mach:         %.3f\n', data.or_max_mach);
    end
    if isfield(data, 'or_time_to_apogee')
        fprintf('  Time to apogee:   %.2f s\n', data.or_time_to_apogee);
    end
    if isfield(data, 'or_flight_time')
        fprintf('  Flight time:      %.2f s\n', data.or_flight_time);
    end
    fprintf('  Launch mass:      %.4f kg\n', data.mass_launch);
    fprintf('  Burnout mass:     %.4f kg\n', data.mass_burnout);
    fprintf('  Burn time:        %.3f s\n', data.burn_time);
    fprintf('  Motor:            %s (x%d)\n', data.motor_name, data.n_motors);
    fprintf('  Ref area:         %.6f m²\n', data.ref_area);
    fprintf('  Ref length:       %.4f m\n', data.ref_length);
    fprintf('=====================================\n\n');

    %% Plot
    figure('Name', 'ORK Validation', 'NumberTitle', 'off', ...
        'Position', [100, 100, 1200, 800]);

    % 1. Thrust
    subplot(2, 2, 1);
    plot(data.thrust_time, data.thrust_force, 'b-', 'LineWidth', 2);
    hold on;
    if isfield(data, 'or_time') && isfield(data, 'or_thrust')
        plot(data.or_time, data.or_thrust, 'r--', 'LineWidth', 1.5);
        legend('Exported', 'OR Sim', 'Location', 'best');
    end
    xlabel('Time [s]');
    ylabel('Thrust [N]');
    title('Thrust Curve');
    grid on;
    hold off;

    % 2. Mass
    subplot(2, 2, 2);
    plot(data.mass_time, data.mass_total, 'b-', 'LineWidth', 2);
    hold on;
    if isfield(data, 'or_time') && isfield(data, 'or_mass')
        plot(data.or_time, data.or_mass, 'r--', 'LineWidth', 1.5);
        legend('Exported', 'OR Sim', 'Location', 'best');
    end
    xlabel('Time [s]');
    ylabel('Mass [kg]');
    title('Mass vs Time');
    grid on;
    hold off;

    % 3. CD vs Mach (at AoA = 0)
    subplot(2, 2, 3);
    cd_aoa0 = data.CD_table(:, 1);  % First AoA index (0°)
    plot(data.mach_bp, cd_aoa0, 'b-', 'LineWidth', 2);
    hold on;
    if isfield(data, 'or_mach') && isfield(data, 'or_cd')
        scatter(data.or_mach, data.or_cd, 10, 'r', 'filled', 'MarkerFaceAlpha', 0.3);
        legend('CD Table (AoA=0°)', 'OR Sim Points', 'Location', 'best');
    end
    xlabel('Mach Number');
    ylabel('C_D');
    title('Drag Coefficient vs Mach');
    grid on;
    hold off;

    % 4. Altitude
    subplot(2, 2, 4);
    if isfield(data, 'or_time') && isfield(data, 'or_altitude')
        plot(data.or_time, data.or_altitude, 'b-', 'LineWidth', 2);
        xlabel('Time [s]');
        ylabel('Altitude [m]');
        title(sprintf('OR Trajectory (Apogee: %.1f m)', data.or_max_altitude));
        grid on;
    else
        text(0.5, 0.5, 'No OR simulation data', ...
            'HorizontalAlignment', 'center', 'FontSize', 14);
        title('Altitude vs Time');
    end

    sgtitle(sprintf('ORK Validation: %s (Motor: %s)', ...
        data.motor_name, orkFilePath), 'Interpreter', 'none');
end
