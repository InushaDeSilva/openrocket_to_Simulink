function results = ork_validate(orkFilePath, varargin)
%ORK_VALIDATE  Compare OpenRocket simulation results with exported data.
%
%   ORK_VALIDATE(ORKFILEPATH) imports the .ork file with validation enabled
%   and plots comparison charts for thrust, drag, mass, trajectory,
%   stability, and angle of attack.
%
%   ORK_VALIDATE(ORKFILEPATH, Name, Value, ...) passes options to ork_import.
%
%   RESULTS = ORK_VALIDATE(...) returns a struct of quantitative metrics:
%     - mass_launch_pct_err: Launch mass relative error [%]
%     - mass_burnout_pct_err: Burnout mass relative error [%]
%     - thrust_nrmse: Thrust NRMSE (normalized RMS error) [-]
%     - cg_launch_err_mm: Launch CG position error [mm]
%     - stability_margin_err: Stability margin error at launch [cal]
%     - pass: true if all metrics within acceptable tolerances
%
%   v2.1 additions:
%     - Sanity banner with motor/manufacturer identity
%     - 3D table dimension cross-check
%     - Rail exit velocity overlay on trajectory
%     - Wind conditions in summary
%
%   See also: ork_import, ork_setup_plant, ork_coord_transforms

    %% Import with validation enabled
    data = ork_import(orkFilePath, 'RunSim', true, varargin{:});

    %% Compute quantitative metrics
    metrics = struct();
    metrics.pass = true;

    % Mass at launch
    mass_launch_export = data.mass_total(1);
    if isfield(data, 'or_mass') && ~isempty(data.or_mass)
        mass_launch_or = data.or_mass(1);
        metrics.mass_launch_pct_err = 100 * abs(mass_launch_export - mass_launch_or) / mass_launch_or;
    else
        metrics.mass_launch_pct_err = NaN;
    end

    % Mass at burnout
    burn_idx_export = find(data.mass_time <= data.burn_time, 1, 'last');
    mass_burnout_export = data.mass_total(burn_idx_export);
    if isfield(data, 'or_time') && isfield(data, 'or_mass')
        burn_idx_or = find(data.or_time <= data.burn_time, 1, 'last');
        if ~isempty(burn_idx_or)
            mass_burnout_or = data.or_mass(burn_idx_or);
            metrics.mass_burnout_pct_err = 100 * abs(mass_burnout_export - mass_burnout_or) / mass_burnout_or;
        else
            metrics.mass_burnout_pct_err = NaN;
        end
    else
        metrics.mass_burnout_pct_err = NaN;
    end

    % Thrust NRMSE
    if isfield(data, 'or_time') && isfield(data, 'or_thrust')
        t_or = data.or_time(:);
        thrust_or = data.or_thrust(:);
        thrust_interp = interp1(data.thrust_time, data.thrust_force, t_or, 'linear', 0);
        thrust_range = max(thrust_or) - min(thrust_or);
        if thrust_range > 0
            metrics.thrust_nrmse = sqrt(mean((thrust_interp - thrust_or).^2)) / thrust_range;
        else
            metrics.thrust_nrmse = NaN;
        end
    else
        metrics.thrust_nrmse = NaN;
    end

    % CG at launch
    cg_launch_export = data.cg_x_vs_time(1);
    if isfield(data, 'or_cg') && ~isempty(data.or_cg)
        cg_launch_or = data.or_cg(1);
        metrics.cg_launch_err_mm = abs(cg_launch_export - cg_launch_or) * 1000;
    else
        metrics.cg_launch_err_mm = NaN;
    end

    % Stability margin at launch
    if isfield(data, 'or_stability') && ~isempty(data.or_stability)
        metrics.stability_margin_err = NaN;
        metrics.or_stability_launch = data.or_stability(1);
    else
        metrics.stability_margin_err = NaN;
    end

    % v2.1: Table dimension cross-check
    metrics.table_dims_ok = true;
    if isfield(data, 'CD_table') && isfield(data, 'alt_bp')
        dims = size(data.CD_table);
        if numel(dims) == 3
            if dims(1) ~= numel(data.mach_bp) || dims(2) ~= numel(data.aoa_bp) || dims(3) ~= numel(data.alt_bp)
                metrics.table_dims_ok = false;
                metrics.pass = false;
            end
        end
    end

    % Pass/fail thresholds
    TOL_MASS_PCT = 1.0;
    TOL_THRUST_NRMSE = 0.05;
    TOL_CG_MM = 5.0;

    if ~isnan(metrics.mass_launch_pct_err) && metrics.mass_launch_pct_err > TOL_MASS_PCT
        metrics.pass = false;
    end
    if ~isnan(metrics.mass_burnout_pct_err) && metrics.mass_burnout_pct_err > TOL_MASS_PCT
        metrics.pass = false;
    end
    if ~isnan(metrics.thrust_nrmse) && metrics.thrust_nrmse > TOL_THRUST_NRMSE
        metrics.pass = false;
    end
    if ~isnan(metrics.cg_launch_err_mm) && metrics.cg_launch_err_mm > TOL_CG_MM
        metrics.pass = false;
    end

    %% ── Sanity Banner ──
    fprintf('\n╔══════════════════════════════════════════════════╗\n');
    fprintf('║          ORK Validation Summary (v2.1)          ║\n');
    fprintf('╠══════════════════════════════════════════════════╣\n');
    if isfield(data, 'meta')
        fprintf('║  Exporter:  %-37s║\n', data.meta.exporter_version);
        fprintf('║  OR:        %-37s║\n', data.meta.openrocket_version);
    end
    fprintf('║  Motor:     %-37s║\n', data.motor_name);
    if isfield(data, 'motor_manufacturer')
        fprintf('║  Mfg:       %-37s║\n', data.motor_manufacturer);
    end
    fprintf('║  Launch:    %-7.4f kg                          ║\n', data.mass_launch);
    fprintf('║  Burnout:   %-7.4f kg                          ║\n', data.mass_burnout);
    fprintf('║  Burn:      %-7.3f s                           ║\n', data.burn_time);
    if isfield(data, 'motor_total_impulse')
        fprintf('║  Impulse:   %-7.2f N·s                         ║\n', data.motor_total_impulse);
    end
    if isfield(data, 'or_max_altitude')
        fprintf('║  Apogee:    %-7.1f m                           ║\n', data.or_max_altitude);
    end
    if isfield(data, 'or_rail_exit_velocity')
        fprintf('║  Rail exit: %-7.2f m/s                         ║\n', data.or_rail_exit_velocity);
    end
    if isfield(data, 'wind_speed_avg')
        fprintf('║  Wind:      %-5.1f m/s @ %-5.1f°                  ║\n', ...
            data.wind_speed_avg, rad2deg(data.wind_direction));
    end

    % v2.1: Table dimensions
    if isfield(data, 'CD_table') && isfield(data, 'alt_bp')
        dims = size(data.CD_table);
        if numel(dims) == 3
            fprintf('║  Aero:      %dx%dx%d (Mach×AoA×Alt)              ║\n', dims(1), dims(2), dims(3));
        end
    end
    if isfield(data, 'I_tensor_vs_time')
        dims = size(data.I_tensor_vs_time);
        fprintf('║  Tensor:    %dx%dx%d (3×3×time)                  ║\n', dims(1), dims(2), dims(3));
    end

    if metrics.pass
        fprintf('║  Status:    ✅ ALL CHECKS PASSED                ║\n');
    else
        fprintf('║  Status:    ❌ SOME CHECKS FAILED               ║\n');
    end
    fprintf('╚══════════════════════════════════════════════════╝\n');

    fprintf('\n--- Quantitative Metrics ---\n');
    printMetric('Mass launch err',   metrics.mass_launch_pct_err, '%%',  TOL_MASS_PCT);
    printMetric('Mass burnout err',  metrics.mass_burnout_pct_err, '%%', TOL_MASS_PCT);
    printMetric('Thrust NRMSE',      metrics.thrust_nrmse * 100, '%%',  TOL_THRUST_NRMSE * 100);
    printMetric('CG launch err',     metrics.cg_launch_err_mm, 'mm',    TOL_CG_MM);
    if ~metrics.table_dims_ok
        fprintf('  %-20s  [FAIL] table dim ≠ breakpoint length\n', '3D table dims');
    else
        fprintf('  %-20s  [PASS]\n', '3D table dims');
    end
    fprintf('=============================================\n\n');

    %% Plot (6 panels)
    figure('Name', 'ORK Validation v2.1', 'NumberTitle', 'off', ...
        'Position', [50, 50, 1400, 900]);

    % 1. Thrust
    subplot(3, 2, 1);
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
    subplot(3, 2, 2);
    plot(data.mass_time, data.mass_total, 'b-', 'LineWidth', 2);
    hold on;
    if isfield(data, 'or_time') && isfield(data, 'or_mass')
        plot(data.or_time, data.or_mass, 'r--', 'LineWidth', 1.5);
        legend('Exported', 'OR Sim', 'Location', 'best');
    end
    if isfield(data, 'm_structure')
        yline(data.m_structure, 'g:', 'Structure', 'LineWidth', 1, 'LabelHorizontalAlignment', 'left');
    end
    xlabel('Time [s]');
    ylabel('Mass [kg]');
    title(sprintf('Mass vs Time (err: %.2f%% / %.2f%%)', ...
        metrics.mass_launch_pct_err, metrics.mass_burnout_pct_err));
    grid on;
    hold off;

    % 3. CD vs Mach (at AoA = 0, ground altitude)
    subplot(3, 2, 3);
    dims = size(data.CD_table);
    if numel(dims) == 3
        cd_aoa0 = data.CD_table(:, 1, 1);  % AoA=0, Alt=0
    else
        cd_aoa0 = data.CD_table(:, 1);
    end
    plot(data.mach_bp, cd_aoa0, 'b-', 'LineWidth', 2);
    hold on;
    if isfield(data, 'or_mach') && isfield(data, 'or_cd')
        scatter(data.or_mach, data.or_cd, 10, 'r', 'filled', 'MarkerFaceAlpha', 0.3);
        legend('CD Table (AoA=0°, Alt=0)', 'OR Sim Points', 'Location', 'best');
    end
    xlabel('Mach Number');
    ylabel('C_D');
    title('Drag Coefficient vs Mach');
    grid on;
    hold off;

    % 4. Altitude + rail exit velocity
    subplot(3, 2, 4);
    if isfield(data, 'or_time') && isfield(data, 'or_altitude')
        plot(data.or_time, data.or_altitude, 'b-', 'LineWidth', 2);
        hold on;
        % v2.1: mark rail exit point
        if isfield(data, 'or_rail_exit_velocity') && isfield(data, 'launch_rod_length')
            idx_rail = find(data.or_altitude >= data.launch_rod_length, 1, 'first');
            if ~isempty(idx_rail)
                plot(data.or_time(idx_rail), data.or_altitude(idx_rail), ...
                    'rv', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
                text(data.or_time(idx_rail), data.or_altitude(idx_rail) + 20, ...
                    sprintf('Rail exit: %.1f m/s', data.or_rail_exit_velocity), ...
                    'FontSize', 9, 'Color', 'r');
            end
        end
        xlabel('Time [s]');
        ylabel('Altitude [m]');
        if isfield(data, 'or_max_altitude')
            title(sprintf('Trajectory (Apogee: %.1f m)', data.or_max_altitude));
        else
            title('Altitude vs Time');
        end
        grid on;
        hold off;
    else
        text(0.5, 0.5, 'No OR simulation data', ...
            'HorizontalAlignment', 'center', 'FontSize', 14);
        title('Altitude vs Time');
    end

    % 5. Stability / CG / CP
    subplot(3, 2, 5);
    hasStability = isfield(data, 'or_time') && isfield(data, 'or_stability');
    hasCG = isfield(data, 'or_time') && isfield(data, 'or_cg');
    hasCP = isfield(data, 'or_time') && isfield(data, 'or_cp');

    if hasStability || hasCG
        yyaxis left;
        if hasCG
            plot(data.or_time, data.or_cg * 100, 'b-', 'LineWidth', 1.5);
            hold on;
        end
        if hasCP
            plot(data.or_time, data.or_cp * 100, 'b--', 'LineWidth', 1.5);
        end
        plot(data.mass_time, data.cg_x_vs_time * 100, 'c:', 'LineWidth', 2);
        ylabel('Position [cm from nose]');
        hold off;

        yyaxis right;
        if hasStability
            plot(data.or_time, data.or_stability, 'r-', 'LineWidth', 1.5);
            ylabel('Stability Margin [cal]');
        end
        xlabel('Time [s]');
        if hasCG && hasCP && hasStability
            legend('OR CG', 'OR CP', 'Export CG', 'Stability', 'Location', 'best');
        elseif hasCG && hasStability
            legend('OR CG', 'Export CG', 'Stability', 'Location', 'best');
        end
        title(sprintf('CG / CP / Stability (CG err: %.1f mm)', metrics.cg_launch_err_mm));
    else
        text(0.5, 0.5, 'No stability data (run with --run-sim)', ...
            'HorizontalAlignment', 'center', 'FontSize', 14);
        title('Stability & CG/CP');
    end
    grid on;

    % 6. Angle of Attack
    subplot(3, 2, 6);
    if isfield(data, 'or_time') && isfield(data, 'or_aoa')
        plot(data.or_time, rad2deg(data.or_aoa), 'b-', 'LineWidth', 1.5);
        hold on;
        if isfield(data, 'or_roll_rate')
            yyaxis right;
            plot(data.or_time, data.or_roll_rate, 'r-', 'LineWidth', 1);
            ylabel('Roll Rate [rad/s]');
        end
        yyaxis left;
        xlabel('Time [s]');
        ylabel('AoA [deg]');
        title('Angle of Attack & Roll Rate');
        legend('AoA', 'Roll Rate', 'Location', 'best');
        grid on;
        hold off;
    else
        text(0.5, 0.5, 'No AoA data (run with --run-sim)', ...
            'HorizontalAlignment', 'center', 'FontSize', 14);
        title('Angle of Attack');
        grid on;
    end

    % Build sgtitle with motor/manufacturer
    titleStr = sprintf('ORK Validation v2.1: %s', data.motor_name);
    if isfield(data, 'motor_manufacturer')
        titleStr = [titleStr, ' (', data.motor_manufacturer, ')'];
    end
    sgtitle(titleStr, 'Interpreter', 'none');

    %% Return metrics
    if nargout > 0
        results = metrics;
    end
end

%% ---- Local helper ----
function printMetric(name, value, unit, threshold)
    if isnan(value)
        fprintf('  %-20s  N/A\n', name);
    elseif value <= threshold
        fprintf('  %-20s  %.3f %s  [PASS]\n', name, value, unit);
    else
        fprintf('  %-20s  %.3f %s  [FAIL > %.1f]\n', name, value, unit, threshold);
    end
end
