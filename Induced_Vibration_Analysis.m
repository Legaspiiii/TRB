%% ========================================================================
%  INDUCED VIBRATION ANALYSIS
%  Forces and moments induced by two momentum wheels, one rotating arm,
%  and their bearings
%
%  Sources:
%    1. IMBALANCE (each rotor), once per revolution (1x RPM)
%       - Static imbalance  : center of mass is off the spin axis -> lateral force
%                             F = mass x eccentricity x speed^2
%       - Dynamic imbalance : rotor sits slightly tilted          -> moment
%                             M = (Izz - I_perpendicular) x tilt x speed^2
%
%    2. BEARINGS (each rotor's tapered roller bearing pair)
%       Rollers passing over tiny bumps on the rings give small kicks at
%       harmonics of the rotor's RPM (cage, cup pass, cone pass, roller spin).
%       - Harmonic numbers come from the bearing geometry (exact).
%       - Kick strength = bearing_kick_ratio x static imbalance force (estimate).
%       - Tapered rollers push at the cup angle, so every kick also has an
%         axial part: axial = lateral x tan(cup angle).
%
%    3. TWIST (each rotor)
%       Motor reaction torque on the shaft during spin-up / spin-down:
%       Mz = Izz x speed / ramp time
%
%  Assumptions (state these in your report):
%    - Hard-mounted: loads are calculated as if bolted to a rigid base.
%    - Tilt and bearing_kick_ratio are estimates (no test data).
%    - Not included: structural resonance, motor torque ripple,
%      test-based bearing amplitudes.
%
%  Directions:
%    lateral = sideways (perpendicular to the shaft)
%    axial   = along the shaft (spin axis)
%
%  How to use:
%    1. Edit SECTION 1 only (your numbers).
%    2. Run the script.
%    3. The Command Window prints Fx, Fy, Fz, Mx, My, Mz in four parts:
%         PART 1  ARM               (imbalance only)
%         PART 2  MOMENTUM WHEELS   (imbalance only)
%         PART 3  BEARINGS          (harmonics only)
%         PART 4  ALL TOGETHER      <- the final answer, at the very bottom
%       The final answer is also saved as final_Fx_N ... final_Mz_Nm.
%  ========================================================================

clear; clc;
fprintf('Calculating...\n');


%% ========================================================================
%  SECTION 1: INPUTS  (this is the only section you need to edit)
%  ========================================================================

% ------------------------------------------------------------------------
% 1A. GENERAL
% ------------------------------------------------------------------------
spin_up_time_s       = 10;     % [s] time to go from 0 to max speed   (for the twist)
spin_down_time_s     = 10;     % [s] time to go from max speed to 0    (for the twist)
reference_position_m = 0;      % [m] point along the shaft where total moments are reported
bearing_kick_ratio   = 0.01;   % [-] each bearing harmonic = this fraction of the static imbalance force (0.01 = 1%)
report_arm_rpm       = [50 200];  % [RPM] extra arm speeds to get results at (wheels follow at the same
                                  %       ratio as the max speeds). Use [] for none.

% ------------------------------------------------------------------------
% 1B. WHEEL 1
% ------------------------------------------------------------------------
wheel1_name               = 'Wheel 1';
wheel1_mass_kg            = 3.0;      % [kg]     rotating mass only
wheel1_eccentricity_um    = 15;       % [um]     center of mass offset from the spin axis
wheel1_Ixx_kgm2           = 0.017;    % [kg*m^2] from CAD, at the center of mass
wheel1_Iyy_kgm2           = 0.017;    % [kg*m^2] from CAD, at the center of mass
wheel1_Izz_kgm2           = 0.034;    % [kg*m^2] from CAD, about the shaft
wheel1_tilt_deg           = 0.04;     % [deg]    how crooked it sits on the shaft
wheel1_max_speed_rpm      = 515;      % [RPM]
wheel1_spin_direction     = -1;       % +1 or -1
wheel1_position_m         = -0.200;   % [m]      center of mass position along the shaft

wheel1_bearing_rollers        = 15;   % [-]   number of rollers      (bearing catalog)
wheel1_bearing_roller_dia_mm  = 7;    % [mm]  mean roller diameter   (bearing catalog)
wheel1_bearing_pitch_dia_mm   = 39;   % [mm]  pitch diameter         (bearing catalog)
wheel1_bearing_cup_angle_deg  = 14;   % [deg] cup (contact) angle    (bearing catalog)

% ------------------------------------------------------------------------
% 1C. ARM
% ------------------------------------------------------------------------
arm_name                  = 'Arm';
arm_mass_kg               = 4.0;      % [kg]     rotating mass only
arm_eccentricity_um       = 20;       % [um]     center of mass offset from the spin axis
arm_Ixx_kgm2              = 0.340;    % [kg*m^2] from CAD, at the center of mass
arm_Iyy_kgm2              = 0.004;    % [kg*m^2] from CAD (about the arm's own length)
arm_Izz_kgm2              = 0.350;    % [kg*m^2] from CAD, about the shaft
arm_tilt_deg              = 0.03;     % [deg]    how crooked it sits on the shaft
arm_max_speed_rpm         = 100;      % [RPM]
arm_spin_direction        = +1;       % +1 or -1
arm_position_m            = 0.000;    % [m]      center of mass position along the shaft

arm_bearing_rollers           = 20;   % [-]   number of rollers      (bearing catalog)
arm_bearing_roller_dia_mm     = 8;    % [mm]  mean roller diameter   (bearing catalog)
arm_bearing_pitch_dia_mm      = 55;   % [mm]  pitch diameter         (bearing catalog)
arm_bearing_cup_angle_deg     = 15;   % [deg] cup (contact) angle    (bearing catalog)

% ------------------------------------------------------------------------
% 1D. WHEEL 2
% ------------------------------------------------------------------------
wheel2_name               = 'Wheel 2';
wheel2_mass_kg            = 3.0;      % [kg]     rotating mass only
wheel2_eccentricity_um    = 15;       % [um]     center of mass offset from the spin axis
wheel2_Ixx_kgm2           = 0.017;    % [kg*m^2] from CAD, at the center of mass
wheel2_Iyy_kgm2           = 0.017;    % [kg*m^2] from CAD, at the center of mass
wheel2_Izz_kgm2           = 0.034;    % [kg*m^2] from CAD, about the shaft
wheel2_tilt_deg           = 0.04;     % [deg]    how crooked it sits on the shaft
wheel2_max_speed_rpm      = 515;      % [RPM]
wheel2_spin_direction     = -1;       % +1 or -1
wheel2_position_m         = 0.200;    % [m]      center of mass position along the shaft

wheel2_bearing_rollers        = 15;   % [-]   number of rollers      (bearing catalog)
wheel2_bearing_roller_dia_mm  = 7;    % [mm]  mean roller diameter   (bearing catalog)
wheel2_bearing_pitch_dia_mm   = 39;   % [mm]  pitch diameter         (bearing catalog)
wheel2_bearing_cup_angle_deg  = 14;   % [deg] cup (contact) angle    (bearing catalog)

% ------------------------------------------------------------------------
% 1E. PLOTS
% ------------------------------------------------------------------------
make_plots  = true;    % true = draw the 2 plots
save_plots  = false;   % true = also save them as PNG files
plot_folder = 'Induced_Vibration_Plots';


%% ========================================================================
%  SECTION 2: PUT THE THREE ROTORS SIDE BY SIDE
%  Every quantity is a row of 3 numbers: [Wheel 1, Arm, Wheel 2]
%  ========================================================================

rotor_names     = {wheel1_name, arm_name, wheel2_name};
number_of_rotors = 3;
arm_index       = 2;

mass_kg          = [wheel1_mass_kg,         arm_mass_kg,         wheel2_mass_kg];
eccentricity_um  = [wheel1_eccentricity_um, arm_eccentricity_um, wheel2_eccentricity_um];
Ixx_kgm2         = [wheel1_Ixx_kgm2,        arm_Ixx_kgm2,        wheel2_Ixx_kgm2];
Iyy_kgm2         = [wheel1_Iyy_kgm2,        arm_Iyy_kgm2,        wheel2_Iyy_kgm2];
Izz_kgm2         = [wheel1_Izz_kgm2,        arm_Izz_kgm2,        wheel2_Izz_kgm2];
tilt_deg         = [wheel1_tilt_deg,        arm_tilt_deg,        wheel2_tilt_deg];
max_speed_rpm    = [wheel1_max_speed_rpm,   arm_max_speed_rpm,   wheel2_max_speed_rpm];
spin_direction   = [wheel1_spin_direction,  arm_spin_direction,  wheel2_spin_direction];
position_m       = [wheel1_position_m,      arm_position_m,      wheel2_position_m];

bearing_rollers       = [wheel1_bearing_rollers,       arm_bearing_rollers,       wheel2_bearing_rollers];
bearing_roller_dia_mm = [wheel1_bearing_roller_dia_mm, arm_bearing_roller_dia_mm, wheel2_bearing_roller_dia_mm];
bearing_pitch_dia_mm  = [wheel1_bearing_pitch_dia_mm,  arm_bearing_pitch_dia_mm,  wheel2_bearing_pitch_dia_mm];
bearing_cup_angle_deg = [wheel1_bearing_cup_angle_deg, arm_bearing_cup_angle_deg, wheel2_bearing_cup_angle_deg];


%% ========================================================================
%  SECTION 3: UNIT CONVERSIONS
%  ========================================================================

speed_rad_s      = max_speed_rpm * 2 * pi / 60;     % [rad/s]
speed_hz         = max_speed_rpm / 60;              % [Hz]  revolutions per second (1x RPM)
eccentricity_m   = eccentricity_um * 1e-6;          % [m]
tilt_rad         = tilt_deg * pi / 180;             % [rad]
lever_arm_m      = abs(position_m - reference_position_m);   % [m] distance to the reference point
ramp_time_s      = max(min(spin_up_time_s, spin_down_time_s), eps);


%% ========================================================================
%  SECTION 4: STATIC AND DYNAMIC IMBALANCE  (1x RPM)
%  ========================================================================

% Static imbalance: center of mass off the spin axis -> lateral force
static_imbalance_force_N = mass_kg .* eccentricity_m .* speed_rad_s .^ 2;

% Dynamic imbalance: rotor tilted on the spin axis -> moment
dynamic_imbalance_moment_Nm = max(abs(Izz_kgm2 - Ixx_kgm2), abs(Izz_kgm2 - Iyy_kgm2)) .* tilt_rad .* speed_rad_s .^ 2;


%% ========================================================================
%  SECTION 5: BEARING HARMONICS
%  ------------------------------------------------------------------------
%  Harmonic number h = number of kicks per revolution of that rotor.
%  Frequency = h x (rotor RPM / 60).
%  Cup (outer ring) rotates with the rotor, cone (inner ring) is on the
%  stationary shaft.
%  ========================================================================

harmonic_names   = {'Cage', 'Cup pass', 'Cone pass', 'Roller spin'};
number_of_harmonics = 4;
bearing_harmonic = zeros(number_of_rotors, number_of_harmonics);   % rows = rotors

for r = 1:number_of_rotors
    g = bearing_roller_dia_mm(r) * cosd(bearing_cup_angle_deg(r)) / bearing_pitch_dia_mm(r);
    bearing_harmonic(r, 1) = (1 + g) / 2;                                                   % cage
    bearing_harmonic(r, 2) = bearing_rollers(r) * (1 - g) / 2;                              % cup pass
    bearing_harmonic(r, 3) = bearing_rollers(r) * (1 + g) / 2;                              % cone pass
    bearing_harmonic(r, 4) = bearing_pitch_dia_mm(r) / (2 * bearing_roller_dia_mm(r)) * (1 - g ^ 2);  % roller spin
end

bearing_frequency_hz  = bearing_harmonic .* (speed_hz');                        % [Hz] rows = rotors
bearing_lateral_each_N = bearing_kick_ratio * static_imbalance_force_N;                           % one harmonic, sideways
bearing_axial_each_N   = bearing_lateral_each_N .* tand(bearing_cup_angle_deg);  % one harmonic, along the shaft
bearing_lateral_N      = number_of_harmonics * bearing_lateral_each_N;          % all 4 harmonics lined up
bearing_axial_N        = number_of_harmonics * bearing_axial_each_N;


%% ========================================================================
%  SECTION 6: FORCES AND MOMENTS OF EACH SOURCE  (worst case, max speed)
%  ------------------------------------------------------------------------
%  Axes: z = along the shaft (spin axis), x and y = sideways.
%
%  Fx, Fy : sideways force. It ROTATES with the rotor, so x and y see the
%           same peak a quarter turn apart  ->  peak Fx = peak Fy.
%  Fz     : force along the shaft (only the tapered bearings make this).
%  Mx, My : tipping moment at the reference point. It also rotates, so
%           peak Mx = peak My. = own moment + sideways force x distance.
%  Mz     : twist about the shaft (motor reaction, only during ramps).
%
%  Each source is stored as 6 numbers: [Fx; Fy; Fz; Mx; My; Mz]
%  Worst case = every peak lines up at the same moment (safe design value).
%  ========================================================================

% ---- Imbalance of each rotor (wheels and arm) ----
imbalance_loads = zeros(6, number_of_rotors);          % columns = Wheel 1, Arm, Wheel 2
for r = 1:number_of_rotors
    imbalance_moment = dynamic_imbalance_moment_Nm(r) + lever_arm_m(r) * static_imbalance_force_N(r);
    imbalance_loads(:, r) = [static_imbalance_force_N(r);     % Fx
                             static_imbalance_force_N(r);     % Fy
                             0;                               % Fz (imbalance does not push along the shaft)
                             imbalance_moment;                % Mx
                             imbalance_moment;                % My
                             spin_direction(r) * Izz_kgm2(r) * speed_rad_s(r) / ramp_time_s];  % Mz (signed twist)
end

% ---- Bearings of each rotor (all 4 harmonics lined up) ----
bearing_loads = zeros(6, number_of_rotors);            % columns = Wheel 1 bearings, Arm bearings, Wheel 2 bearings
for r = 1:number_of_rotors
    bearing_moment = lever_arm_m(r) * bearing_lateral_N(r);
    bearing_loads(:, r) = [bearing_lateral_N(r);    % Fx
                           bearing_lateral_N(r);    % Fy
                           bearing_axial_N(r);      % Fz
                           bearing_moment;          % Mx
                           bearing_moment;          % My
                           0];                      % Mz (bearing drag is internal: motor and bearings cancel)
end

% ---- Group the columns ----
wheel_columns = [1 3];
arm_loads     = imbalance_loads(:, arm_index);
wheel_loads   = imbalance_loads(:, wheel_columns);
wheels_total  = sum(wheel_loads, 2);                   % Mz adds with sign (both wheels spin the same way)
bearings_total = sum(bearing_loads, 2);
all_total     = arm_loads + wheels_total + bearings_total;
all_total(6)  = abs(all_total(6));                     % net twist (wheels cancel the arm)

% Show twist as a size (positive) everywhere except where noted
arm_loads(6)     = abs(arm_loads(6));
wheel_loads(6,:) = abs(wheel_loads(6,:));
wheels_total(6)  = abs(wheels_total(6));

% ---- Final answer saved in the Workspace: [Fx Fy Fz Mx My Mz] ----
final_arm_loads      = arm_loads';
final_wheel_1_loads  = wheel_loads(:, 1)';
final_wheel_2_loads  = wheel_loads(:, 2)';
final_bearing_loads  = bearings_total';
final_Fx_N  = all_total(1);
final_Fy_N  = all_total(2);
final_Fz_N  = all_total(3);
final_Mx_Nm = all_total(4);
final_My_Nm = all_total(5);
final_Mz_Nm = all_total(6);


%% ========================================================================
%  SECTION 7: PLOTS
%  ------------------------------------------------------------------------
%  Plot 1: sideways force (Fx = Fy) vs RPM    Plot 2: axial force (Fz) vs RPM
%  Every load grows with speed squared: load(speed) = load(max) x (speed/max)^2
%  The wheels follow the arm, so the x-axis is arm speed.
%  ========================================================================

if make_plots
    speed_fraction = linspace(0, 1.2, 121)';          % 0 to 120% of max speed
    arm_axis_rpm   = speed_fraction * max_speed_rpm(arm_index);
    curve_shape    = speed_fraction .^ 2;
    x_label_text   = sprintf('Arm speed [RPM]    (wheel speed = %.2f x arm speed)', max_speed_rpm(1) / max_speed_rpm(arm_index));

    plot_row     = [1, 3];                              % row 1 = Fx, row 3 = Fz
    plot_titles  = {'Plot 1: Sideways force (Fx = Fy) vs RPM', 'Plot 2: Axial force (Fz) vs RPM'};
    plot_ylabels = {'Fx = Fy [N]', 'Fz [N]'};
    plot_files   = {'Plot_1_Fx_Fy_vs_RPM.png', 'Plot_2_Fz_vs_RPM.png'};
    curve_names  = {'Arm', 'Wheel 1', 'Wheel 2', 'Bearings (all)', 'TOTAL'};
    curve_colors = [0.85 0.33 0.10; 0.00 0.45 0.74; 0.47 0.67 0.19; 0.49 0.18 0.56; 0 0 0];
    curve_lines  = {'-.', '-', '--', ':', '-'};

    for n = 1:2
        fprintf('Drawing plot %d...\n', n);
        try
            row = plot_row(n);
            curve_values = [arm_loads(row), wheel_loads(row, 1), wheel_loads(row, 2), bearings_total(row), all_total(row)];
            figure(n);
            clf;
            try
                set(gcf, 'Renderer', 'painters');
            catch
            end
            hold on;
            grid on;
            box on;
            for c = 1:numel(curve_values)
                line_width = 2;
                if c == numel(curve_values)
                    line_width = 3;
                end
                plot(arm_axis_rpm, curve_values(c) * curve_shape, curve_lines{c}, ...
                     'Color', curve_colors(c, :), 'LineWidth', line_width);
            end
            plot(max_speed_rpm(arm_index), all_total(row), 'ko', 'MarkerFaceColor', 'k');
            text(max_speed_rpm(arm_index), all_total(row), sprintf('%.3g N at max speed   ', all_total(row)), ...
                 'HorizontalAlignment', 'right');
            xlabel(x_label_text);
            ylabel(plot_ylabels{n});
            title(plot_titles{n});
            legend(curve_names, 'Location', 'northwest');
            hold off;
            fprintf('Plot %d done.\n', n);

            if save_plots
                if ~exist(plot_folder, 'dir')
                    mkdir(plot_folder);
                end
                saveas(gcf, fullfile(plot_folder, plot_files{n}));
                fprintf('Saved: %s\n', fullfile(pwd, plot_folder, plot_files{n}));
            end
        catch
            fprintf('PLOT %d PROBLEM: %s\n', n, lasterr);
        end
    end
end


%% ========================================================================
%  SECTION 8: PRINTED RESULTS  -  ARM, WHEELS, BEARINGS, ALL TOGETHER
%  ------------------------------------------------------------------------
%  Every part prints the same 6 rows: Fx, Fy, Fz, Mx, My, Mz
%  ========================================================================

row_labels = {'Fx  [N]   sideways', 'Fy  [N]   sideways', 'Fz  [N]   along shaft', ...
              'Mx  [N*m] tipping',  'My  [N*m] tipping',  'Mz  [N*m] twist (ramps)'};
part_line  = repmat('=', 1, 90);

part_titles  = {sprintf('PART 1: ARM  -  static + dynamic imbalance  (%.0f RPM)', max_speed_rpm(arm_index)), ...
                sprintf('PART 2: MOMENTUM WHEELS  -  static + dynamic imbalance  (%.0f RPM)', max_speed_rpm(1)), ...
                'PART 3: BEARINGS  -  harmonics of each rotor''s RPM', ...
                'PART 4: ALL TOGETHER  -  FINAL ANSWER  (arm + wheels + bearings)'};
part_columns = {{'Arm'}, ...
                {rotor_names{1}, rotor_names{3}, 'Wheels total'}, ...
                {[rotor_names{1}, ' brg'], [rotor_names{2}, ' brg'], [rotor_names{3}, ' brg'], 'Bearings total'}, ...
                {'Arm', 'Wheels', 'Bearings', 'TOTAL'}};
part_values  = {arm_loads, ...
                [wheel_loads, wheels_total], ...
                [bearing_loads, bearings_total], ...
                [arm_loads, wheels_total, bearings_total, all_total]};

for part = 1:4
    columns = part_columns{part};
    values  = part_values{part};
    fprintf('\n%s\n %s\n%s\n', part_line, part_titles{part}, part_line);
    fprintf('   %-26s', '');
    fprintf('%16s', columns{:});
    fprintf('\n');
    for k = 1:6
        fprintf('   %-26s', row_labels{k});
        fprintf('%16.4g', values(k, :));
        fprintf('\n');
    end

    % Extra information for each part
    if part == 1
        fprintf('\n   Fx = Fy: static imbalance %.4g N\n', static_imbalance_force_N(arm_index));
        fprintf('   Mx = My: dynamic imbalance %.4g N*m + force x distance %.4g N*m\n', ...
            dynamic_imbalance_moment_Nm(arm_index), lever_arm_m(arm_index) * static_imbalance_force_N(arm_index));
        fprintf('   Frequency: 1x RPM = %.2f Hz\n', speed_hz(arm_index));
    elseif part == 2
        for r = wheel_columns
            fprintf('\n   %s:  Fx = Fy static imbalance %.4g N  |  Mx = My: dynamic imbalance %.4g + force x distance %.4g N*m  |  1x = %.2f Hz', ...
                rotor_names{r}, static_imbalance_force_N(r), dynamic_imbalance_moment_Nm(r), ...
                lever_arm_m(r) * static_imbalance_force_N(r), speed_hz(r));
        end
        fprintf('\n');
    elseif part == 3
        fprintf('\n   Each harmonic (frequency = harmonic x RPM / 60):\n');
        fprintf('   %-10s %-13s %10s %12s %14s %14s\n', 'Rotor', 'Harmonic', 'x RPM', 'Freq [Hz]', 'Fx = Fy [N]', 'Fz [N]');
        for r = 1:number_of_rotors
            for k = 1:number_of_harmonics
                fprintf('   %-10s %-13s %10.3f %12.2f %14.4g %14.4g\n', rotor_names{r}, harmonic_names{k}, ...
                    bearing_harmonic(r, k), bearing_frequency_hz(r, k), bearing_lateral_each_N(r), bearing_axial_each_N(r));
            end
        end
    elseif part == 4
        fprintf('\n   Mz TOTAL is the NET twist: the wheels twist opposite to the arm, so they cancel.\n');
        fprintf('   Each motor mount still feels its own twist (see PART 1 and PART 2).\n');
        fprintf('\n   ASSUMPTIONS: hard-mounted (rigid base). Tilt and bearing kick ratio are estimates.\n');
        fprintf('   NOT INCLUDED: structural resonance, motor torque ripple, test-based bearing amplitudes.\n');
        if ~isempty(report_arm_rpm)
            fprintf('\n   AT OTHER SPEEDS  (TOTAL; wheels follow the arm; every value grows with speed squared)\n');
            fprintf('   %10s %12s %14s %14s %16s\n', 'Arm RPM', 'Wheel RPM', 'Fx = Fy [N]', 'Fz [N]', 'Mx = My [N*m]');
            for i = 1:numel(report_arm_rpm)
                speed_ratio = report_arm_rpm(i) / max_speed_rpm(arm_index);
                fprintf('   %10.1f %12.1f %14.4g %14.4g %16.4g\n', report_arm_rpm(i), max_speed_rpm(1) * speed_ratio, ...
                    all_total(1) * speed_ratio ^ 2, all_total(3) * speed_ratio ^ 2, all_total(4) * speed_ratio ^ 2);
            end
        end
    end
end
fprintf('%s\n', part_line);
