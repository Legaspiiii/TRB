%% ========================================================================
%  INDUCED VIBRATION ANALYSIS
%  Two momentum wheels + one rotating arm + their bearings
%
%  What this script does:
%    Estimates the forces and moments that the spinning parts and their
%    bearings push into the structure, using manufacturing tolerances
%    instead of test data. Imbalance loads follow the speed-squared model
%    used in industry (NASA SDO, Masterson, Zhang/Aglietti).
%
%  Sources included:
%    SHAKE  - static imbalance (center of mass off the spin axis)  -> lateral force
%    ROCK   - dynamic imbalance (part tilted on the spin axis)     -> moment
%    TWIST  - motor reaction torque during spin-up and spin-down   -> moment about the shaft
%    KICKS  - bearing rollers passing over tiny bumps (estimated)  -> lateral + axial force
%
%  Directions:
%    axial   = along the shaft (z, the spin axis)
%    lateral = sideways, perpendicular to the shaft (x and y)
%
%  Plots (2 total):
%    Plot 1 - Lateral force vs RPM
%    Plot 2 - Axial force vs RPM
%
%  How to use:
%    1. Edit SECTION 1 only.
%    2. Run the script.
%    3. Read the printed summary and the 2 plots.
%  ========================================================================

clear; clc; close all;
set(0, 'DefaultFigureVisible', 'on');   % make sure plot windows are allowed to show


%% ========================================================================
%  SECTION 1: INPUTS  (this is the only section you need to edit)
%  ========================================================================

% ------------------------------------------------------------------------
% 1A. RUN PROFILE  (used for the spin-up / spin-down twist)
% ------------------------------------------------------------------------
spin_up_time_s   = 10;      % [s] time to go from 0 to max speed
spin_down_time_s = 10;      % [s] time to go from max speed back to 0

% ------------------------------------------------------------------------
% 1B. REFERENCE POINT
% ------------------------------------------------------------------------
reference_position_m = 0;   % [m] position along the shaft (z) where total moments are reported

% ------------------------------------------------------------------------
% 1C. WHEEL 1
% ------------------------------------------------------------------------
wheel1_name                   = 'Wheel 1';
wheel1_mass_kg                = 3.0;      % [kg]    rotating mass only
wheel1_eccentricity_um        = 5;        % [um]    center of mass offset from spin axis
wheel1_Ixx_kgm2               = 0.017;    % [kg*m^2] from CAD, at the center of mass
wheel1_Iyy_kgm2               = 0.017;    % [kg*m^2] from CAD, at the center of mass
wheel1_Izz_kgm2               = 0.034;    % [kg*m^2] from CAD, about the shaft
wheel1_tilt_deg               = 0.01;     % [deg]   assumed assembly tilt
wheel1_max_speed_rpm          = 515;      % [RPM]   speed during the hold
wheel1_spin_direction         = -1;       % +1 or -1
wheel1_cm_position_m          = -0.200;   % [m]     center of mass position along z
wheel1_bearing_A_position_m   = -0.215;   % [m]     bearing A position along z
wheel1_bearing_B_position_m   = -0.185;   % [m]     bearing B position along z

% Wheel 1 bearings (from the bearing catalog)
wheel1_bearing_rollers           = 15;     % [-]    number of rollers
wheel1_bearing_roller_dia_mm     = 7;      % [mm]   mean roller diameter
wheel1_bearing_pitch_dia_mm      = 39;     % [mm]   pitch diameter
wheel1_bearing_cup_angle_deg     = 14;     % [deg]  cup (contact) angle
wheel1_bearing_runout_um         = 10;     % [um]   radial runout (precision class)
wheel1_bearing_load_offset_mm    = 5;      % [mm]   load center offset "a" (0 if unknown)

% ------------------------------------------------------------------------
% 1D. ARM
% ------------------------------------------------------------------------
arm_name                      = 'Arm';
arm_mass_kg                   = 4.0;      % [kg]    rotating mass only
arm_eccentricity_um           = 10;       % [um]    center of mass offset from spin axis
arm_Ixx_kgm2                  = 0.340;    % [kg*m^2] from CAD, at the center of mass
arm_Iyy_kgm2                  = 0.004;    % [kg*m^2] from CAD (about the arm's own length)
arm_Izz_kgm2                  = 0.350;    % [kg*m^2] from CAD, about the shaft
arm_tilt_deg                  = 0.01;     % [deg]   assumed assembly tilt
arm_max_speed_rpm             = 100;      % [RPM]   speed during the hold
arm_spin_direction            = +1;       % +1 or -1
arm_cm_position_m             = 0.000;    % [m]     center of mass position along z
arm_bearing_A_position_m      = -0.020;   % [m]     bearing A position along z
arm_bearing_B_position_m      = 0.020;    % [m]     bearing B position along z

% Arm bearings (from the bearing catalog)
arm_bearing_rollers              = 20;     % [-]    number of rollers
arm_bearing_roller_dia_mm        = 8;      % [mm]   mean roller diameter
arm_bearing_pitch_dia_mm         = 55;     % [mm]   pitch diameter
arm_bearing_cup_angle_deg        = 15;     % [deg]  cup (contact) angle
arm_bearing_runout_um            = 10;     % [um]   radial runout (precision class)
arm_bearing_load_offset_mm       = 6;      % [mm]   load center offset "a" (0 if unknown)

% ------------------------------------------------------------------------
% 1E. WHEEL 2
% ------------------------------------------------------------------------
wheel2_name                   = 'Wheel 2';
wheel2_mass_kg                = 3.0;      % [kg]    rotating mass only
wheel2_eccentricity_um        = 5;        % [um]    center of mass offset from spin axis
wheel2_Ixx_kgm2               = 0.017;    % [kg*m^2] from CAD, at the center of mass
wheel2_Iyy_kgm2               = 0.017;    % [kg*m^2] from CAD, at the center of mass
wheel2_Izz_kgm2               = 0.034;    % [kg*m^2] from CAD, about the shaft
wheel2_tilt_deg               = 0.01;     % [deg]   assumed assembly tilt
wheel2_max_speed_rpm          = 515;      % [RPM]   speed during the hold
wheel2_spin_direction         = -1;       % +1 or -1
wheel2_cm_position_m          = 0.200;    % [m]     center of mass position along z
wheel2_bearing_A_position_m   = 0.185;    % [m]     bearing A position along z
wheel2_bearing_B_position_m   = 0.215;    % [m]     bearing B position along z

% Wheel 2 bearings (from the bearing catalog)
wheel2_bearing_rollers           = 15;     % [-]    number of rollers
wheel2_bearing_roller_dia_mm     = 7;      % [mm]   mean roller diameter
wheel2_bearing_pitch_dia_mm      = 39;     % [mm]   pitch diameter
wheel2_bearing_cup_angle_deg     = 14;     % [deg]  cup (contact) angle
wheel2_bearing_runout_um         = 10;     % [um]   radial runout (precision class)
wheel2_bearing_load_offset_mm    = 5;      % [mm]   load center offset "a" (0 if unknown)

% ------------------------------------------------------------------------
% 1F. ANALYSIS SETTINGS
% ------------------------------------------------------------------------
include_bearing_runout = true;   % true  = add bearing runout on top of eccentricity
                                 % false = eccentricity was measured on the assembled part
bearing_kick_ratio     = 0.01;   % [-] strength of each bearing kick as a fraction of the shake (0.01 = 1%)

save_plots  = true;                        % save the 2 plots as PNG files
plot_folder = 'Induced_Vibration_Plots';   % folder the PNG files go into


%% ========================================================================
%  SECTION 2: ORGANIZE INPUTS  (put the three parts side by side)
%  ------------------------------------------------------------------------
%  Every quantity below is a row of 3 numbers: [Wheel 1, Arm, Wheel 2]
%  ========================================================================

part_names      = {wheel1_name, arm_name, wheel2_name};
number_of_parts = 3;
arm_index       = 2;
wheel_indices   = [1 3];

part_mass_kg          = [wheel1_mass_kg,         arm_mass_kg,         wheel2_mass_kg];
part_eccentricity_um  = [wheel1_eccentricity_um, arm_eccentricity_um, wheel2_eccentricity_um];
part_Ixx_kgm2         = [wheel1_Ixx_kgm2,        arm_Ixx_kgm2,        wheel2_Ixx_kgm2];
part_Iyy_kgm2         = [wheel1_Iyy_kgm2,        arm_Iyy_kgm2,        wheel2_Iyy_kgm2];
part_Izz_kgm2         = [wheel1_Izz_kgm2,        arm_Izz_kgm2,        wheel2_Izz_kgm2];
part_tilt_deg         = [wheel1_tilt_deg,        arm_tilt_deg,        wheel2_tilt_deg];
part_max_speed_rpm    = [wheel1_max_speed_rpm,   arm_max_speed_rpm,   wheel2_max_speed_rpm];
part_spin_direction   = [wheel1_spin_direction,  arm_spin_direction,  wheel2_spin_direction];
part_cm_position_m    = [wheel1_cm_position_m,   arm_cm_position_m,   wheel2_cm_position_m];
bearing_A_position_m  = [wheel1_bearing_A_position_m, arm_bearing_A_position_m, wheel2_bearing_A_position_m];
bearing_B_position_m  = [wheel1_bearing_B_position_m, arm_bearing_B_position_m, wheel2_bearing_B_position_m];

bearing_rollers           = [wheel1_bearing_rollers,        arm_bearing_rollers,        wheel2_bearing_rollers];
bearing_roller_dia_mm     = [wheel1_bearing_roller_dia_mm,  arm_bearing_roller_dia_mm,  wheel2_bearing_roller_dia_mm];
bearing_pitch_dia_mm      = [wheel1_bearing_pitch_dia_mm,   arm_bearing_pitch_dia_mm,   wheel2_bearing_pitch_dia_mm];
bearing_cup_angle_deg     = [wheel1_bearing_cup_angle_deg,  arm_bearing_cup_angle_deg,  wheel2_bearing_cup_angle_deg];
bearing_runout_um         = [wheel1_bearing_runout_um,      arm_bearing_runout_um,      wheel2_bearing_runout_um];
bearing_load_offset_mm    = [wheel1_bearing_load_offset_mm, arm_bearing_load_offset_mm, wheel2_bearing_load_offset_mm];

% Plot style per part: Wheel 1 = blue solid, Arm = orange dash-dot, Wheel 2 = green dashed
part_colors  = [0.00 0.45 0.74;
                0.85 0.33 0.10;
                0.47 0.67 0.19];
part_lines   = {'-', '-.', '--'};


%% ========================================================================
%  SECTION 3: UNIT CONVERSIONS AND DERIVED QUANTITIES
%  ========================================================================

% ---- Unit conversions ----
part_max_speed_rad_s = part_max_speed_rpm * 2 * pi / 60;   % [rad/s]
part_max_speed_hz    = part_max_speed_rpm / 60;            % [Hz] revolutions per second
part_eccentricity_m  = part_eccentricity_um * 1e-6;        % [m]
part_tilt_rad        = part_tilt_deg * pi / 180;           % [rad]

% ---- Bearing pair geometry (back-to-back pairs act wider than they physically are) ----
bearing_low_m          = min(bearing_A_position_m, bearing_B_position_m);
bearing_high_m         = max(bearing_A_position_m, bearing_B_position_m);
bearing_A_effective_m  = bearing_low_m  - bearing_load_offset_mm * 1e-3;
bearing_B_effective_m  = bearing_high_m + bearing_load_offset_mm * 1e-3;
bearing_spacing_m      = bearing_B_effective_m - bearing_A_effective_m;
bearing_mid_position_m = (bearing_low_m + bearing_high_m) / 2;

% ---- Bearing runout adds to eccentricity and tilt ----
part_runout_m       = bearing_runout_um * 1e-6 * include_bearing_runout;
part_total_ecc_m    = part_eccentricity_m + part_runout_m;
part_total_tilt_rad = part_tilt_rad + 2 * part_runout_m ./ bearing_spacing_m;

% ---- Imbalance strengths ----
part_inertia_difference = max(abs(part_Izz_kgm2 - part_Ixx_kgm2), abs(part_Izz_kgm2 - part_Iyy_kgm2));
static_imbalance_kgm    = part_mass_kg .* part_total_ecc_m;                 % U_s [kg*m]
dynamic_imbalance_kgm2  = part_inertia_difference .* part_total_tilt_rad;   % U_d [kg*m^2]

% ---- Distances from the reference point (lever arms for moments) ----
cm_distance_m          = abs(part_cm_position_m - reference_position_m);
bearing_mid_distance_m = abs(bearing_mid_position_m - reference_position_m);

% ---- Bearing kick frequencies as harmonic numbers (kicks per revolution) ----
%      Cup (outer ring) rotates, cone (inner ring) is stationary.
%      Rows = parts, columns = kick types.
bearing_gamma = bearing_roller_dia_mm .* cosd(bearing_cup_angle_deg) ./ bearing_pitch_dia_mm;

kick_names      = {'cage', 'cup pass', 'cone pass', 'roller spin'};
number_of_kicks = 4;
kick_harmonic   = zeros(number_of_parts, number_of_kicks);
for p = 1:number_of_parts
    g = bearing_gamma(p);
    kick_harmonic(p, 1) = (1 + g) / 2;                                                    % cage
    kick_harmonic(p, 2) = bearing_rollers(p) * (1 - g) / 2;                               % cup pass
    kick_harmonic(p, 3) = bearing_rollers(p) * (1 + g) / 2;                               % cone pass
    kick_harmonic(p, 4) = bearing_pitch_dia_mm(p) / (2 * bearing_roller_dia_mm(p)) * (1 - g ^ 2); % roller spin
end


%% ========================================================================
%  SECTION 4: LOADS OF EACH PART AT MAX SPEED
%  ------------------------------------------------------------------------
%  Every imbalance load grows with speed squared, so the value at any other
%  speed is:  load(speed) = load(max speed) x (speed / max speed)^2
%  ========================================================================

omega_squared = part_max_speed_rad_s .^ 2;

% ---- Lateral (sideways) ----
shake_N              = static_imbalance_kgm .* omega_squared;              % shake force
kick_lateral_each_N  = bearing_kick_ratio * shake_N;                       % one bearing kick, sideways
lateral_force_N      = shake_N + number_of_kicks * kick_lateral_each_N;    % part total (all kicks lined up)

% ---- Axial (along the shaft) ----
% Tapered rollers push at the cup angle, so each sideways kick also pushes along the shaft.
kick_axial_each_N    = kick_lateral_each_N .* tand(bearing_cup_angle_deg); % one bearing kick, axial
axial_force_N        = number_of_kicks * kick_axial_each_N;                % part total (all kicks lined up)

% ---- Moments ----
rock_Nm              = dynamic_imbalance_kgm2 .* omega_squared;            % rock from tilt
moment_at_reference_Nm = rock_Nm + cm_distance_m .* shake_N ...
                       + bearing_mid_distance_m .* number_of_kicks .* kick_lateral_each_N;   % rock + force x distance
ramp_time_s          = max(min(spin_up_time_s, spin_down_time_s), eps);
twist_Nm             = part_Izz_kgm2 .* part_max_speed_rad_s / ramp_time_s; % twist about the shaft during ramps

% ---- Totals for the whole system ----
% Worst case: every peak lines up.  Random phase: root-sum-square (typical value).
total_lateral_worst_N  = sum(lateral_force_N);
total_lateral_rss_N    = sqrt(sum(shake_N .^ 2 + number_of_kicks * kick_lateral_each_N .^ 2));
total_axial_worst_N    = sum(axial_force_N);
total_axial_rss_N      = sqrt(sum(number_of_kicks * kick_axial_each_N .^ 2));
total_moment_worst_Nm  = sum(moment_at_reference_Nm);
total_moment_rss_Nm    = sqrt(sum(rock_Nm .^ 2 + (cm_distance_m .* shake_N) .^ 2 ...
                       + number_of_kicks * (bearing_mid_distance_m .* kick_lateral_each_N) .^ 2));
net_twist_Nm           = abs(sum(part_spin_direction .* part_Izz_kgm2 .* part_max_speed_rad_s)) / ramp_time_s;


%% ========================================================================
%  SECTION 5: LOAD ON EACH BEARING  (worst case at max speed)
%  ------------------------------------------------------------------------
%  Shake splits like a wheelbarrow (closer bearing takes more).
%  Rock pushes the two bearings in opposite directions: rock / spacing.
%  Bearing kicks are shared equally between the two bearings.
%  ========================================================================

bearing_A_lateral_N = abs(shake_N .* (bearing_B_effective_m - part_cm_position_m) ./ bearing_spacing_m) ...
                    + rock_Nm ./ bearing_spacing_m + number_of_kicks * kick_lateral_each_N / 2;
bearing_B_lateral_N = abs(shake_N .* (part_cm_position_m - bearing_A_effective_m) ./ bearing_spacing_m) ...
                    + rock_Nm ./ bearing_spacing_m + number_of_kicks * kick_lateral_each_N / 2;
bearing_axial_N     = number_of_kicks * kick_axial_each_N / 2;


%% ========================================================================
%  SECTION 6: MOMENTUM CHECK  (do the wheels cancel the arm?)
%  ========================================================================

part_momentum_Nms      = part_spin_direction .* part_Izz_kgm2 .* part_max_speed_rad_s;
arm_momentum_Nms       = abs(part_momentum_Nms(arm_index));
net_momentum_Nms       = sum(part_momentum_Nms);
leftover_percent       = 100 * abs(net_momentum_Nms) / arm_momentum_Nms;
wheel_speed_needed_rpm = arm_momentum_Nms / sum(part_Izz_kgm2(wheel_indices)) * 60 / (2 * pi);


%% ========================================================================
%  SECTION 7: SANITY CHECKS
%  ========================================================================

% Double speed must give 4x force
force_ratio = sum(static_imbalance_kgm .* (2 * part_max_speed_rad_s) .^ 2) / sum(shake_N);
check_speed = abs(force_ratio - 4) < 1e-9;

% The two bearings together must carry exactly the shake (signed wheelbarrow split)
share_A = (bearing_B_effective_m - part_cm_position_m) ./ bearing_spacing_m;
share_B = (part_cm_position_m - bearing_A_effective_m) ./ bearing_spacing_m;
check_bearings = max(abs(share_A + share_B - 1)) < 1e-9;

% Wheels should cancel the arm momentum (within 1%)
check_momentum = leftover_percent < 1;
if any(part_spin_direction(wheel_indices) == part_spin_direction(arm_index))
    check_momentum = false;
end


%% ========================================================================
%  SECTION 8: PRINTED SUMMARY
%  ========================================================================

line_text = repmat('=', 1, 78);
fprintf('%s\n INDUCED VIBRATION ANALYSIS - SUMMARY\n%s\n', line_text, line_text);
fprintf('Bearing runout included: %d    Bearing kick ratio: %.1f%%\n', ...
    include_bearing_runout, 100 * bearing_kick_ratio);

fprintf('\n--- 1. EACH PART AT MAX SPEED ---------------------------------------------------\n');
fprintf('%-28s %12s %12s %12s\n', '', part_names{1}, part_names{2}, part_names{3});
fprintf('%-28s %12.0f %12.0f %12.0f\n', 'Max speed [RPM]',              part_max_speed_rpm .* part_spin_direction);
fprintf('%-28s %12.2f %12.2f %12.2f\n', 'Total eccentricity [um]',      part_total_ecc_m * 1e6);
fprintf('%-28s %12.4f %12.4f %12.4f\n', 'Total tilt [deg]',             part_total_tilt_rad * 180 / pi);
fprintf('%-28s %12.3f %12.3f %12.3f\n', 'Static imbalance [g*mm]',      static_imbalance_kgm * 1e6);
fprintf('%-28s %12.1f %12.1f %12.1f\n', 'Dynamic imbalance [g*mm^2]',   dynamic_imbalance_kgm2 * 1e9);
fprintf('%-28s %12.4g %12.4g %12.4g\n', 'Lateral force [N]',            lateral_force_N);
fprintf('%-28s %12.4g %12.4g %12.4g\n', 'Axial force [N]',              axial_force_N);
fprintf('%-28s %12.4g %12.4g %12.4g\n', 'Rock moment [N*m]',            rock_Nm);
fprintf('%-28s %12.4g %12.4g %12.4g\n', 'Moment at reference [N*m]',    moment_at_reference_Nm);
fprintf('%-28s %12.4g %12.4g %12.4g\n', 'Ramp twist [N*m]',             twist_Nm);

fprintf('\n--- 2. VIBRATION FREQUENCIES AT MAX SPEED [Hz] ----------------------------------\n');
fprintf('%-28s %12s %12s %12s\n', '', part_names{1}, part_names{2}, part_names{3});
fprintf('%-28s %12.2f %12.2f %12.2f\n', '1x imbalance', part_max_speed_hz);
for k = 1:number_of_kicks
    fprintf('%-28s %12.2f %12.2f %12.2f\n', ['Bearing ', kick_names{k}], kick_harmonic(:, k)' .* part_max_speed_hz);
end

fprintf('\n--- 3. TOTAL LOADS ON THE STRUCTURE  (MAIN ANSWER) -------------------------------\n');
fprintf('%-34s %14s %14s\n', '', 'Worst case', 'Random phase');
fprintf('%-34s %14.4g %14.4g\n', 'Lateral force [N]',             total_lateral_worst_N, total_lateral_rss_N);
fprintf('%-34s %14.4g %14.4g\n', 'Axial force [N]',               total_axial_worst_N,   total_axial_rss_N);
fprintf('%-34s %14.4g %14.4g\n', 'Moment at reference [N*m]',     total_moment_worst_Nm, total_moment_rss_Nm);
fprintf('%-34s %14.4g\n',        'Net twist during ramps [N*m]',  net_twist_Nm);

fprintf('\n--- 4. LOAD ON EACH BEARING (worst case) ----------------------------------------\n');
fprintf('%-28s %12s %12s %12s\n', '', part_names{1}, part_names{2}, part_names{3});
fprintf('%-28s %12.4g %12.4g %12.4g\n', 'Bearing A lateral [N]', bearing_A_lateral_N);
fprintf('%-28s %12.4g %12.4g %12.4g\n', 'Bearing B lateral [N]', bearing_B_lateral_N);
fprintf('%-28s %12.4g %12.4g %12.4g\n', 'Each bearing axial [N]', bearing_axial_N);

fprintf('\n--- 5. MOMENTUM CHECK -----------------------------------------------------------\n');
fprintf('Arm momentum:                     %10.4f N*m*s\n', arm_momentum_Nms);
fprintf('Net momentum:                     %10.4f N*m*s  (%.2f%% of the arm)\n', net_momentum_Nms, leftover_percent);
fprintf('Wheel speed that cancels the arm: %10.1f RPM  (you entered %.0f and %.0f RPM)\n', ...
    wheel_speed_needed_rpm, part_max_speed_rpm(wheel_indices));

fprintf('\n--- 6. SANITY CHECKS -----------------------------------------------------------\n');
check_labels = {'FAIL', 'PASS'};
fprintf('  %s  Double speed gives 4x force (got %.4fx)\n', check_labels{check_speed + 1}, force_ratio);
fprintf('  %s  Bearing A + B carry exactly the shake\n',  check_labels{check_bearings + 1});
fprintf('  %s  Wheels cancel the arm momentum within 1%%\n', check_labels{check_momentum + 1});

fprintf('\n--- 7. ASSUMPTIONS -------------------------------------------------------------\n');
fprintf('  1. Rigid mounting: no structural resonance amplification.\n');
fprintf('  2. Eccentricity, runout and tilt come from tolerances, not measurements.\n');
fprintf('  3. Imbalance loads grow with speed squared.\n');
fprintf('  4. Bearing kicks are estimated: %.1f%% of the shake each, axial = lateral x tan(cup angle).\n', 100 * bearing_kick_ratio);
fprintf('%s\n', line_text);


%% ========================================================================
%  SECTION 9: KEY RESULTS  (printed before the plots, so you always get them)
%  ========================================================================

[~, biggest_part] = max(lateral_force_N);
fprintf('\n%s\n', line_text);
fprintf(' KEY RESULTS - loads your system puts on the structure at max speed\n');
fprintf('%s\n', line_text);
fprintf(' %-34s %12s %14s\n', '', 'WORST CASE', 'TYPICAL');
fprintf(' %-34s %12.4g %14.4g   N\n',   'Lateral (sideways) force',       total_lateral_worst_N, total_lateral_rss_N);
fprintf(' %-34s %12.4g %14.4g   N\n',   'Axial (along the shaft) force',  total_axial_worst_N,   total_axial_rss_N);
fprintf(' %-34s %12.4g %14.4g   N*m\n', 'Moment (tipping)',               total_moment_worst_Nm, total_moment_rss_Nm);
fprintf(' %-34s %12.4g %14s   N*m\n',   'Net twist during spin-up/down',  net_twist_Nm, '');
fprintf('\n Twist on each motor mount during spin-up/down:  %s %.3g, %s %.3g, %s %.3g N*m\n', ...
    part_names{1}, twist_Nm(1), part_names{2}, twist_Nm(2), part_names{3}, twist_Nm(3));
fprintf(' Biggest single source of lateral force:          %s (%.3g N)\n', ...
    part_names{biggest_part}, lateral_force_N(biggest_part));
fprintf(' All sanity checks passed:                        %s\n', check_labels{(check_speed && check_bearings && check_momentum) + 1});
fprintf('%s\n', line_text);


%% ========================================================================
%  SECTION 10: PLOTS  (2 plots, one graph each)
%  ------------------------------------------------------------------------
%  Plot 1: lateral force vs RPM      Plot 2: axial force vs RPM
%  If anything goes wrong while plotting, the error is printed and the
%  results above are not affected.
%  ========================================================================

speed_scale  = linspace(0, 1.2, 121);                          % 0 to 120% of max speed
arm_axis_rpm = speed_scale * part_max_speed_rpm(arm_index);    % x-axis
wheel_ratio  = part_max_speed_rpm(1) / part_max_speed_rpm(arm_index);
x_label_text = sprintf('Arm speed [RPM]    (wheel speed = %.2f x arm speed)', wheel_ratio);
legend_text  = [part_names, {'Total (worst case)', 'Total (typical)'}];

plot_forces = {lateral_force_N, axial_force_N};
plot_worst  = [total_lateral_worst_N, total_axial_worst_N];
plot_rss    = [total_lateral_rss_N,   total_axial_rss_N];
plot_titles = {'Plot 1: Lateral force vs RPM', 'Plot 2: Axial force vs RPM'};
plot_ylabel = {'Lateral force [N]', 'Axial force [N]'};
plot_files  = {'Plot_1_Lateral_Force_vs_RPM.png', 'Plot_2_Axial_Force_vs_RPM.png'};

for n = 1:2
    try
        figure(n);
        clf;
        hold on;
        grid on;
        box on;
        force_at_max = plot_forces{n};
        for p = 1:number_of_parts
            plot(arm_axis_rpm, force_at_max(p) * speed_scale .^ 2, part_lines{p}, ...
                 'Color', part_colors(p, :), 'LineWidth', 2);
        end
        plot(arm_axis_rpm, plot_worst(n) * speed_scale .^ 2, 'k-',  'LineWidth', 2.5);
        plot(arm_axis_rpm, plot_rss(n)   * speed_scale .^ 2, 'k--', 'LineWidth', 2);
        plot(part_max_speed_rpm(arm_index), plot_worst(n), 'ko', 'MarkerFaceColor', 'k');
        text(part_max_speed_rpm(arm_index), plot_worst(n), sprintf('%.3g N at max speed   ', plot_worst(n)), ...
             'HorizontalAlignment', 'right');
        xlabel(x_label_text);
        ylabel(plot_ylabel{n});
        title(plot_titles{n});
        legend(legend_text, 'Location', 'northwest');
        hold off;
        drawnow;

        if save_plots
            if ~exist(plot_folder, 'dir')
                mkdir(plot_folder);
            end
            saveas(gcf, fullfile(plot_folder, plot_files{n}));
            fprintf('Saved plot: %s\n', fullfile(pwd, plot_folder, plot_files{n}));
        end
    catch
        fprintf('\nPLOT %d PROBLEM: %s\n', n, lasterr);
        fprintf('The results above are still correct. Send this message to get the plot fixed.\n');
    end
end
