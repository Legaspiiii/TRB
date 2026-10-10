%% ========================================================================
%  INDUCED VIBRATION ANALYSIS
%  Two momentum wheels + one rotating arm + their bearings
%
%  What this script does:
%    Estimates the forces and moments that the spinning parts and their
%    bearings push into the structure, using manufacturing tolerances
%    instead of test data. All imbalance loads follow the speed-squared
%    model used in industry (NASA SDO, Masterson, Zhang/Aglietti).
%
%  Sources included:
%    SHAKE  - static imbalance (center of mass off the spin axis)
%    ROCK   - dynamic imbalance (part tilted on the spin axis)
%    TWIST  - motor reaction torque during spin-up and spin-down
%    KICKS  - bearing rollers passing over tiny bumps (estimated)
%
%  Coordinates:
%    z = along the shaft (spin axis), x and y = perpendicular to the shaft
%
%  How to use:
%    1. Edit SECTION 1 only (inputs and switches).
%    2. Run the script.
%    3. Read the printed summary and the plots.
%  ========================================================================

clear; clc; close all;


%% ========================================================================
%  SECTION 1: INPUTS  (this is the only section you need to edit)
%  ========================================================================

% ------------------------------------------------------------------------
% 1A. RUN PROFILE  (all three parts ramp up, hold and ramp down together)
% ------------------------------------------------------------------------
spin_up_time_s   = 10;      % [s] time to go from 0 to max speed
hold_time_s      = 30;      % [s] time spent at max speed
spin_down_time_s = 10;      % [s] time to go from max speed back to 0

% ------------------------------------------------------------------------
% 1B. REFERENCE POINT  (where the total forces and moments are reported)
% ------------------------------------------------------------------------
reference_position_m = 0;   % [m] position along the shaft (z). 0 = arm center

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
wheel1_bearing_stiffness_N_per_m = 2e8;    % [N/m]  radial stiffness of ONE bearing
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
arm_bearing_stiffness_N_per_m    = 3e8;    % [N/m]  radial stiffness of ONE bearing
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
wheel2_bearing_stiffness_N_per_m = 2e8;    % [N/m]  radial stiffness of ONE bearing
wheel2_bearing_load_offset_mm    = 5;      % [mm]   load center offset "a" (0 if unknown)

% ------------------------------------------------------------------------
% 1F. ANALYSIS SETTINGS
% ------------------------------------------------------------------------
include_bearing_runout = true;   % true  = add bearing runout on top of eccentricity
                                 % false = eccentricity was measured on the assembled part
bearing_kick_ratio     = 0.01;   % [-]   bearing kick strength as a fraction of imbalance (0.01 = 1%)
rock_phase_deg         = 0;      % [deg] angle between heavy spot and tilt direction (0 = worst case)
stiffness_uncertainty  = 0.20;   % [-]   +/- band on whirl frequencies (0.20 = +/-20%)
campbell_speed_factor  = 1.2;    % [-]   Campbell plot x-axis goes to this x max speed
points_per_cycle       = 20;     % [-]   time samples per cycle of the fastest vibration
max_number_of_samples  = 500000; % [-]   cap on time samples (keeps memory reasonable)

% ------------------------------------------------------------------------
% 1G. PLOT SWITCHES  (true = make the plot, false = skip it)
% ------------------------------------------------------------------------
make_plot_1_speed_vs_force    = true;
make_plot_2_speed_vs_torque   = true;
make_plot_3_spectrum          = true;
make_plot_4_campbell          = true;
make_plot_5_waterfall         = true;
make_plot_6_loads_over_time   = true;
make_plot_7_momentum          = true;

save_plots  = true;                        % save every plot as a PNG file
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

part_mass_kg          = [wheel1_mass_kg,        arm_mass_kg,        wheel2_mass_kg];
part_eccentricity_um  = [wheel1_eccentricity_um, arm_eccentricity_um, wheel2_eccentricity_um];
part_Ixx_kgm2         = [wheel1_Ixx_kgm2,       arm_Ixx_kgm2,       wheel2_Ixx_kgm2];
part_Iyy_kgm2         = [wheel1_Iyy_kgm2,       arm_Iyy_kgm2,       wheel2_Iyy_kgm2];
part_Izz_kgm2         = [wheel1_Izz_kgm2,       arm_Izz_kgm2,       wheel2_Izz_kgm2];
part_tilt_deg         = [wheel1_tilt_deg,       arm_tilt_deg,       wheel2_tilt_deg];
part_max_speed_rpm    = [wheel1_max_speed_rpm,  arm_max_speed_rpm,  wheel2_max_speed_rpm];
part_spin_direction   = [wheel1_spin_direction, arm_spin_direction, wheel2_spin_direction];
part_cm_position_m    = [wheel1_cm_position_m,  arm_cm_position_m,  wheel2_cm_position_m];
bearing_A_position_m  = [wheel1_bearing_A_position_m, arm_bearing_A_position_m, wheel2_bearing_A_position_m];
bearing_B_position_m  = [wheel1_bearing_B_position_m, arm_bearing_B_position_m, wheel2_bearing_B_position_m];

bearing_rollers           = [wheel1_bearing_rollers,           arm_bearing_rollers,           wheel2_bearing_rollers];
bearing_roller_dia_mm     = [wheel1_bearing_roller_dia_mm,     arm_bearing_roller_dia_mm,     wheel2_bearing_roller_dia_mm];
bearing_pitch_dia_mm      = [wheel1_bearing_pitch_dia_mm,      arm_bearing_pitch_dia_mm,      wheel2_bearing_pitch_dia_mm];
bearing_cup_angle_deg     = [wheel1_bearing_cup_angle_deg,     arm_bearing_cup_angle_deg,     wheel2_bearing_cup_angle_deg];
bearing_runout_um         = [wheel1_bearing_runout_um,         arm_bearing_runout_um,         wheel2_bearing_runout_um];
bearing_stiffness_N_per_m = [wheel1_bearing_stiffness_N_per_m, arm_bearing_stiffness_N_per_m, wheel2_bearing_stiffness_N_per_m];
bearing_load_offset_mm    = [wheel1_bearing_load_offset_mm,    arm_bearing_load_offset_mm,    wheel2_bearing_load_offset_mm];

% Colors used for each part in every plot: Wheel 1 = blue, Arm = orange, Wheel 2 = green
part_colors  = [0.00 0.45 0.74;
                0.85 0.33 0.10;
                0.47 0.67 0.19];
total_color  = [0 0 0];

% Line style per part, so two parts with identical values never hide each other
part_line_styles = {'-', '-.', '--'};


%% ========================================================================
%  SECTION 3: UNIT CONVERSIONS AND DERIVED QUANTITIES
%  ========================================================================

% ---- Unit conversions (Equations 0.1 to 0.3) ----
part_max_speed_rad_s = part_max_speed_rpm * 2 * pi / 60;   % [rad/s]
part_max_speed_hz    = part_max_speed_rpm / 60;            % [Hz]
part_eccentricity_m  = part_eccentricity_um * 1e-6;        % [m]
part_tilt_rad        = part_tilt_deg * pi / 180;           % [rad]
rock_phase_rad       = rock_phase_deg * pi / 180;          % [rad]

% ---- Bearing geometry ----
bearing_low_position_m  = min(bearing_A_position_m, bearing_B_position_m);
bearing_high_position_m = max(bearing_A_position_m, bearing_B_position_m);
bearing_load_offset_m   = bearing_load_offset_mm * 1e-3;

% Back-to-back pairs act wider than they physically are (Equation 2.3)
bearing_A_effective_m   = bearing_low_position_m  - bearing_load_offset_m;
bearing_B_effective_m   = bearing_high_position_m + bearing_load_offset_m;
bearing_spacing_m       = bearing_B_effective_m - bearing_A_effective_m;
bearing_mid_position_m  = (bearing_low_position_m + bearing_high_position_m) / 2;

% ---- Bearing runout adds to eccentricity and tilt (Equations 2.1, 2.2) ----
part_runout_m        = bearing_runout_um * 1e-6 * include_bearing_runout;
part_total_ecc_m     = part_eccentricity_m + part_runout_m;
part_total_tilt_rad  = part_tilt_rad + 2 * part_runout_m ./ bearing_spacing_m;

% ---- Imbalance strengths (Equations 3.1 and 4.1a) ----
part_inertia_difference = max(abs(part_Izz_kgm2 - part_Ixx_kgm2), abs(part_Izz_kgm2 - part_Iyy_kgm2));
static_imbalance_kgm    = part_mass_kg .* part_total_ecc_m;                 % U_s [kg*m]
dynamic_imbalance_kgm2  = part_inertia_difference .* part_total_tilt_rad;   % U_d [kg*m^2]

% Same values in the units used by specs and papers (Equations 0.4, 0.5)
static_imbalance_gmm    = static_imbalance_kgm * 1e6;     % [g*mm]
dynamic_imbalance_gmm2  = dynamic_imbalance_kgm2 * 1e9;   % [g*mm^2]

% ---- Distances from the reference point (used for lever-arm moments) ----
cm_distance_m          = part_cm_position_m - reference_position_m;
bearing_mid_distance_m = bearing_mid_position_m - reference_position_m;

% ---- Bearing kick frequencies as harmonic numbers (Equations 7.1 to 7.6) ----
%      Cup (outer ring) rotates, cone (inner ring) is stationary.
%      Each column is one kick type, each row is one part.
bearing_gamma = bearing_roller_dia_mm .* cosd(bearing_cup_angle_deg) ./ bearing_pitch_dia_mm;

kick_names = {'cage', 'cup pass', 'cone pass', 'roller spin'};
number_of_kicks = 4;
kick_harmonic = zeros(number_of_parts, number_of_kicks);
kick_harmonic(:, 1) = (1 + bearing_gamma) / 2;                                          % cage
kick_harmonic(:, 2) = bearing_rollers .* (1 - bearing_gamma) / 2;                        % cup pass
kick_harmonic(:, 3) = bearing_rollers .* (1 + bearing_gamma) / 2;                        % cone pass
kick_harmonic(:, 4) = bearing_pitch_dia_mm ./ (2 * bearing_roller_dia_mm) .* (1 - bearing_gamma .^ 2); % roller spin

kick_frequency_hz = kick_harmonic .* (part_max_speed_hz');   % at max speed [Hz]


%% ========================================================================
%  SECTION 4: PEAK LOAD FORMULAS  (worst case and random phase)
%  ------------------------------------------------------------------------
%  These small formulas give the peak loads at constant speed. They are
%  used for the summary, the speed plots and the sanity checks.
%  Inputs are scale factors: 1 = baseline value.
%  ========================================================================

% Shake force amplitude of each part (Equation 3.2)
calc_shake_N = @(ecc_scale, runout_scale, speed_scale) ...
    part_mass_kg .* (part_eccentricity_m * ecc_scale + part_runout_m * runout_scale) ...
    .* (part_max_speed_rad_s * speed_scale) .^ 2;

% Rock moment amplitude of each part (Equation 4.2)
calc_rock_Nm = @(tilt_scale, runout_scale, speed_scale) ...
    part_inertia_difference .* (part_tilt_rad * tilt_scale + 2 * part_runout_m * runout_scale ./ bearing_spacing_m) ...
    .* (part_max_speed_rad_s * speed_scale) .^ 2;

% Worst case: every peak lines up (Equations 10.8 and 10.9)
calc_worst_force_N = @(ecc_scale, runout_scale, kick_ratio, speed_scale) ...
    sum(calc_shake_N(ecc_scale, runout_scale, speed_scale) * (1 + number_of_kicks * kick_ratio));

calc_worst_moment_Nm = @(ecc_scale, tilt_scale, runout_scale, kick_ratio, speed_scale) ...
    sum(calc_rock_Nm(tilt_scale, runout_scale, speed_scale) ...
        + abs(cm_distance_m) .* calc_shake_N(ecc_scale, runout_scale, speed_scale) ...
        + abs(bearing_mid_distance_m) .* calc_shake_N(ecc_scale, runout_scale, speed_scale) * number_of_kicks * kick_ratio);

% Random phase: root-sum-square (Equations 10.10 and 10.11)
calc_rss_force_N = @(ecc_scale, runout_scale, kick_ratio, speed_scale) ...
    sqrt(sum(calc_shake_N(ecc_scale, runout_scale, speed_scale) .^ 2 * (1 + number_of_kicks * kick_ratio ^ 2)));

calc_rss_moment_Nm = @(ecc_scale, tilt_scale, runout_scale, kick_ratio, speed_scale) ...
    sqrt(sum(calc_rock_Nm(tilt_scale, runout_scale, speed_scale) .^ 2 ...
        + (cm_distance_m .* calc_shake_N(ecc_scale, runout_scale, speed_scale)) .^ 2 ...
        + number_of_kicks * (bearing_mid_distance_m .* calc_shake_N(ecc_scale, runout_scale, speed_scale) * kick_ratio) .^ 2));

% Baseline peak values at max speed
peak_shake_N        = calc_shake_N(1, 1, 1);
peak_rock_Nm        = calc_rock_Nm(1, 1, 1);
peak_kick_N         = peak_shake_N * number_of_kicks * bearing_kick_ratio;   % all 4 kicks lined up
peak_twist_Nm       = part_Izz_kgm2 .* part_max_speed_rad_s / max(min(spin_up_time_s, spin_down_time_s), eps);
worst_force_N       = calc_worst_force_N(1, 1, bearing_kick_ratio, 1);
worst_moment_Nm     = calc_worst_moment_Nm(1, 1, 1, bearing_kick_ratio, 1);
rss_force_N         = calc_rss_force_N(1, 1, bearing_kick_ratio, 1);
rss_moment_Nm       = calc_rss_moment_Nm(1, 1, 1, bearing_kick_ratio, 1);


%% ========================================================================
%  SECTION 5: TIME AND SPEED PROFILE  (Equations 1.1 to 1.5)
%  ========================================================================

total_time_s = spin_up_time_s + hold_time_s + spin_down_time_s;

% Choose a sample rate fast enough for the fastest vibration in the system
fastest_harmonic_per_part = max([ones(number_of_parts, 1), kick_harmonic], [], 2)';
fastest_frequency_hz      = max(fastest_harmonic_per_part .* part_max_speed_hz);
sample_rate_hz            = points_per_cycle * fastest_frequency_hz;
number_of_samples         = round(total_time_s * sample_rate_hz) + 1;

if number_of_samples > max_number_of_samples
    sample_rate_hz    = (max_number_of_samples - 1) / total_time_s;
    number_of_samples = max_number_of_samples;
    warning('Sample rate reduced to %.0f Hz to limit memory use.', sample_rate_hz);
    if sample_rate_hz < 2.5 * fastest_frequency_hz
        warning('Sample rate is too low for the fastest bearing kick. Shorten the run or raise max_number_of_samples.');
    end
end

time_step_s = 1 / sample_rate_hz;
time_s      = (0:number_of_samples - 1) * time_step_s;

% Speed shape: goes 0 -> 1 during spin-up, stays 1 during hold, 1 -> 0 during spin-down
speed_shape = ones(1, number_of_samples);
accel_shape = zeros(1, number_of_samples);

in_spin_up   = time_s < spin_up_time_s;
in_spin_down = time_s > spin_up_time_s + hold_time_s;
in_hold      = ~in_spin_up & ~in_spin_down;

if spin_up_time_s > 0
    speed_shape(in_spin_up) = time_s(in_spin_up) / spin_up_time_s;
    accel_shape(in_spin_up) = 1 / spin_up_time_s;
end
if spin_down_time_s > 0
    speed_shape(in_spin_down) = (total_time_s - time_s(in_spin_down)) / spin_down_time_s;
    accel_shape(in_spin_down) = -1 / spin_down_time_s;
end

% Each row = one part, each column = one instant in time
signed_max_speed_rad_s = part_max_speed_rad_s .* part_spin_direction;
speed_rad_s  = signed_max_speed_rad_s' * speed_shape;    % [rad/s]   signed speed
accel_rad_s2 = signed_max_speed_rad_s' * accel_shape;    % [rad/s^2] signed acceleration

% Angle turned so far = running total of speed x time (trapezoid rule)
angle_rad = [zeros(number_of_parts, 1), ...
             cumsum((speed_rad_s(:, 1:end-1) + speed_rad_s(:, 2:end)) / 2, 2) * time_step_s];

speed_squared = speed_rad_s .^ 2;


%% ========================================================================
%  SECTION 6: SHAKE, ROCK AND TWIST OVER TIME
%  ========================================================================

% ---- Shake: static imbalance force (Equations 3.3, 3.4) ----
shake_x_N = (static_imbalance_kgm') .* (speed_squared .* cos(angle_rad) + accel_rad_s2 .* sin(angle_rad));
shake_y_N = (static_imbalance_kgm') .* (speed_squared .* sin(angle_rad) - accel_rad_s2 .* cos(angle_rad));

% ---- Rock: dynamic imbalance moment (Equations 4.3, 4.4) ----
rock_angle_rad = angle_rad + rock_phase_rad;
rock_x_Nm = (dynamic_imbalance_kgm2') .* (speed_squared .* cos(rock_angle_rad) + accel_rad_s2 .* sin(rock_angle_rad));
rock_y_Nm = (dynamic_imbalance_kgm2') .* (speed_squared .* sin(rock_angle_rad) - accel_rad_s2 .* cos(rock_angle_rad));

% ---- Twist: motor reaction torque on the stationary shaft (Equation 6.1) ----
twist_z_Nm = -(part_Izz_kgm2') .* accel_rad_s2;


%% ========================================================================
%  SECTION 7: BEARING KICKS OVER TIME  (Equations 8.1 to 8.3, updated)
%  ------------------------------------------------------------------------
%  Kick strength = bearing_kick_ratio x shake force, at each bearing frequency.
%  Phases are fixed and spread out (different for every part and every
%  kick type) so the kicks do not all line up or cancel by coincidence.
%  ========================================================================

kick_x_N = zeros(number_of_parts, number_of_samples);
kick_y_N = zeros(number_of_parts, number_of_samples);
kick_amplitude_N = bearing_kick_ratio * (static_imbalance_kgm') .* speed_squared;

for kick_number = 1:number_of_kicks
    kick_phase_rad = (kick_number + number_of_kicks * (0:number_of_parts - 1)') * pi / 7;
    kick_angle_rad = kick_harmonic(:, kick_number) .* angle_rad + kick_phase_rad;
    kick_x_N = kick_x_N + kick_amplitude_N .* sin(kick_angle_rad);
    kick_y_N = kick_y_N + kick_amplitude_N .* cos(kick_angle_rad);
end


%% ========================================================================
%  SECTION 8: TOTALS AT THE REFERENCE POINT  (Equations 10.1 to 10.7)
%  ========================================================================

total_force_x_N  = sum(shake_x_N + kick_x_N, 1);
total_force_y_N  = sum(shake_y_N + kick_y_N, 1);

total_moment_x_Nm = sum(rock_x_Nm - (cm_distance_m') .* shake_y_N - (bearing_mid_distance_m') .* kick_y_N, 1);
total_moment_y_Nm = sum(rock_y_Nm + (cm_distance_m') .* shake_x_N + (bearing_mid_distance_m') .* kick_x_N, 1);
total_moment_z_Nm = sum(twist_z_Nm, 1);

total_radial_force_N   = sqrt(total_force_x_N .^ 2 + total_force_y_N .^ 2);
total_radial_moment_Nm = sqrt(total_moment_x_Nm .^ 2 + total_moment_y_Nm .^ 2);

simulated_peak_force_N   = max(total_radial_force_N);
simulated_peak_moment_Nm = max(total_radial_moment_Nm);
simulated_peak_twist_Nm  = max(abs(total_moment_z_Nm));


%% ========================================================================
%  SECTION 9: LOAD ON EACH BEARING  (Equations 9.1 to 9.6)
%  ========================================================================

bearing_A_peak_N = zeros(1, number_of_parts);
bearing_B_peak_N = zeros(1, number_of_parts);
bearing_sum_error_N = zeros(1, number_of_parts);

for p = 1:number_of_parts
    z_A = bearing_A_effective_m(p);
    z_B = bearing_B_effective_m(p);
    z_cm = part_cm_position_m(p);
    spacing = bearing_spacing_m(p);

    % x-direction share (wheelbarrow split of shake + opposite push from rock)
    load_A_x = shake_x_N(p, :) * (z_B - z_cm) / spacing - rock_y_Nm(p, :) / spacing + kick_x_N(p, :) / 2;
    load_B_x = shake_x_N(p, :) * (z_cm - z_A) / spacing + rock_y_Nm(p, :) / spacing + kick_x_N(p, :) / 2;

    % y-direction share
    load_A_y = shake_y_N(p, :) * (z_B - z_cm) / spacing + rock_x_Nm(p, :) / spacing + kick_y_N(p, :) / 2;
    load_B_y = shake_y_N(p, :) * (z_cm - z_A) / spacing - rock_x_Nm(p, :) / spacing + kick_y_N(p, :) / 2;

    bearing_A_peak_N(p) = max(sqrt(load_A_x .^ 2 + load_A_y .^ 2));
    bearing_B_peak_N(p) = max(sqrt(load_B_x .^ 2 + load_B_y .^ 2));

    % Check: the two bearings must carry exactly the shake + kick (Equation 14.4)
    bearing_sum_error_N(p) = max(abs(load_A_x + load_B_x - shake_x_N(p, :) - kick_x_N(p, :)));
end


%% ========================================================================
%  SECTION 10: MOMENTUM CHECK  (Equations 11.1 to 11.4)
%  ========================================================================

part_momentum_Nms  = (part_Izz_kgm2') .* speed_rad_s;   % each part, over time
net_momentum_Nms   = sum(part_momentum_Nms, 1);
net_torque_Nm      = total_moment_z_Nm;

arm_momentum_at_max_Nms = part_Izz_kgm2(arm_index) * part_max_speed_rad_s(arm_index);
net_momentum_at_max_Nms = sum(part_Izz_kgm2 .* signed_max_speed_rad_s);
leftover_percent        = 100 * abs(net_momentum_at_max_Nms) / arm_momentum_at_max_Nms;

wheel_speed_needed_rad_s = arm_momentum_at_max_Nms / sum(part_Izz_kgm2(wheel_indices));
wheel_speed_needed_rpm   = wheel_speed_needed_rad_s * 60 / (2 * pi);


%% ========================================================================
%  SECTION 11: CAMPBELL DIAGRAM DATA  (whirl modes and crossings)
%  ------------------------------------------------------------------------
%  Natural "wobble" frequency of each part on its bearing pair, split into
%  nutation (forward whirl, rises with speed) and precession (backward
%  whirl, drops with speed) by the gyroscopic effect.
%  ========================================================================

campbell_points = 400;
tilt_stiffness_Nm_per_rad = bearing_stiffness_N_per_m .* bearing_spacing_m .^ 2 / 2;   % (Eq. 12.1)
radial_mode_hz = sqrt(2 * bearing_stiffness_N_per_m ./ part_mass_kg) / (2 * pi);         % (Eq. 12.3)

% Whirl frequency in Hz for a given perpendicular inertia, spin inertia, stiffness and speed
calc_forward_whirl_hz = @(I_perp, I_spin, k_tilt, spin_rad_s) ...
    (I_spin * spin_rad_s + sqrt((I_spin * spin_rad_s) .^ 2 + 4 * I_perp * k_tilt)) / (2 * I_perp) / (2 * pi);
calc_backward_whirl_hz = @(I_perp, I_spin, k_tilt, spin_rad_s) ...
    (-I_spin * spin_rad_s + sqrt((I_spin * spin_rad_s) .^ 2 + 4 * I_perp * k_tilt)) / (2 * I_perp) / (2 * pi);

campbell_speed_rpm   = cell(1, number_of_parts);
campbell_modes_hz    = cell(1, number_of_parts);   % rows: modes, columns: speed points
campbell_modes_low   = cell(1, number_of_parts);
campbell_modes_high  = cell(1, number_of_parts);
campbell_mode_names  = cell(1, number_of_parts);
campbell_sources_hz  = cell(1, number_of_parts);   % rows: 1x + kicks
campbell_source_names = [{'1x imbalance'}, kick_names];
crossing_list = {};   % each row: part, source, mode, speed RPM, inside operating range

for p = 1:number_of_parts
    speed_rpm = linspace(0, campbell_speed_factor * part_max_speed_rpm(p), campbell_points);
    spin_rad_s = speed_rpm * 2 * pi / 60;
    campbell_speed_rpm{p} = speed_rpm;

    % Perpendicular inertias to check (both if they differ, like the arm)
    if abs(part_Ixx_kgm2(p) - part_Iyy_kgm2(p)) > 0.01 * max(part_Ixx_kgm2(p), part_Iyy_kgm2(p))
        perp_inertias = [part_Ixx_kgm2(p), part_Iyy_kgm2(p)];
        perp_labels   = {'about x', 'about y'};
    else
        perp_inertias = part_Ixx_kgm2(p);
        perp_labels   = {''};
    end

    modes = [];  modes_low = [];  modes_high = [];  names = {};
    for i = 1:numel(perp_inertias)
        k_nom  = tilt_stiffness_Nm_per_rad(p);
        k_low  = k_nom * (1 - stiffness_uncertainty);
        k_high = k_nom * (1 + stiffness_uncertainty);
        I_perp = perp_inertias(i);
        I_spin = part_Izz_kgm2(p);

        modes      = [modes;      calc_forward_whirl_hz(I_perp, I_spin, k_nom, spin_rad_s)];
        modes_low  = [modes_low;  calc_forward_whirl_hz(I_perp, I_spin, k_low, spin_rad_s)];
        modes_high = [modes_high; calc_forward_whirl_hz(I_perp, I_spin, k_high, spin_rad_s)];
        names{end + 1} = strtrim(['nutation ', perp_labels{i}]);

        modes      = [modes;      calc_backward_whirl_hz(I_perp, I_spin, k_nom, spin_rad_s)];
        modes_low  = [modes_low;  calc_backward_whirl_hz(I_perp, I_spin, k_low, spin_rad_s)];
        modes_high = [modes_high; calc_backward_whirl_hz(I_perp, I_spin, k_high, spin_rad_s)];
        names{end + 1} = strtrim(['precession ', perp_labels{i}]);
    end
    radial_line = radial_mode_hz(p) * ones(1, campbell_points);
    modes      = [modes;      radial_line];
    modes_low  = [modes_low;  radial_line * sqrt(1 - stiffness_uncertainty)];
    modes_high = [modes_high; radial_line * sqrt(1 + stiffness_uncertainty)];
    names{end + 1} = 'radial (side-to-side)';

    campbell_modes_hz{p}   = modes;
    campbell_modes_low{p}  = modes_low;
    campbell_modes_high{p} = modes_high;
    campbell_mode_names{p} = names;

    % Source lines: 1x imbalance and the 4 bearing kicks
    sources = [1, kick_harmonic(p, :)]' * (speed_rpm / 60);
    campbell_sources_hz{p} = sources;

    % Find every speed where a source line crosses a mode line
    for s = 1:size(sources, 1)
        for m = 1:size(modes, 1)
            difference = sources(s, :) - modes(m, :);
            sign_change = find(difference(1:end-1) .* difference(2:end) < 0);
            for c = sign_change
                fraction = difference(c) / (difference(c) - difference(c + 1));
                crossing_rpm = speed_rpm(c) + fraction * (speed_rpm(c + 1) - speed_rpm(c));
                inside_range = crossing_rpm <= part_max_speed_rpm(p);
                crossing_list(end + 1, :) = {part_names{p}, campbell_source_names{s}, names{m}, crossing_rpm, inside_range};
            end
        end
    end
end


%% ========================================================================
%  SECTION 12: SANITY CHECKS  (Equations 14.1 to 14.5)
%  ========================================================================

check_names  = {};
check_passed = [];

% 14.1  No eccentricity and no runout -> no shake
check_names{end + 1}  = 'Zero eccentricity and runout gives zero shake';
check_passed(end + 1) = calc_worst_force_N(0, 0, bearing_kick_ratio, 1) == 0;

% 14.2  No tilt and no runout -> no rock
check_names{end + 1}  = 'Zero tilt and runout gives zero rock';
check_passed(end + 1) = all(calc_rock_Nm(0, 0, 1) == 0);

% 14.3  Double speed -> 4x force
speed_ratio_check = calc_worst_force_N(1, 1, bearing_kick_ratio, 2) / worst_force_N;
check_names{end + 1}  = sprintf('Double speed gives 4x force (got %.4fx)', speed_ratio_check);
check_passed(end + 1) = abs(speed_ratio_check - 4) < 1e-9;

% 14.4  Bearings A + B carry exactly the shake + kicks
check_names{end + 1}  = sprintf('Bearing A + B = part force (largest error %.2e N)', max(bearing_sum_error_N));
check_passed(end + 1) = max(bearing_sum_error_N) < 1e-9 * max(1, max(peak_shake_N));

% 14.5  Time simulation during the hold never exceeds the worst-case formula
if any(in_hold)
    hold_peak_force_N = max(total_radial_force_N(in_hold));
    check_names{end + 1}  = sprintf('Simulated hold peak (%.4g N) <= worst case (%.4g N)', hold_peak_force_N, worst_force_N);
    check_passed(end + 1) = hold_peak_force_N <= worst_force_N * (1 + 1e-6);
end


%% ========================================================================
%  SECTION 13: PRINTED SUMMARY
%  ========================================================================

line_text = repmat('=', 1, 78);
fprintf('%s\n INDUCED VIBRATION ANALYSIS - SUMMARY\n%s\n', line_text, line_text);

fprintf('\nRUN PROFILE: spin-up %.1f s, hold %.1f s, spin-down %.1f s  (sample rate %.0f Hz, %d samples)\n', ...
    spin_up_time_s, hold_time_s, spin_down_time_s, sample_rate_hz, number_of_samples);
fprintf('Bearing runout included: %s    Bearing kick ratio: %.1f%%    Reference point: z = %.3f m\n', ...
    mat2str(include_bearing_runout), 100 * bearing_kick_ratio, reference_position_m);

fprintf('\n--- 1. INPUTS AND IMBALANCE ---------------------------------------------------\n');
fprintf('%-22s %12s %12s %12s\n', '', part_names{:});
fprintf('%-22s %12.0f %12.0f %12.0f\n', 'Max speed [RPM]',         part_max_speed_rpm .* part_spin_direction);
fprintf('%-22s %12.3f %12.3f %12.3f\n', 'Mass [kg]',               part_mass_kg);
fprintf('%-22s %12.2f %12.2f %12.2f\n', 'Total eccentricity [um]', part_total_ecc_m * 1e6);
fprintf('%-22s %12.4f %12.4f %12.4f\n', 'Total tilt [deg]',        part_total_tilt_rad * 180 / pi);
fprintf('%-22s %12.3f %12.3f %12.3f\n', 'Static imb. [g*mm]',      static_imbalance_gmm);
fprintf('%-22s %12.1f %12.1f %12.1f\n', 'Dynamic imb. [g*mm^2]',   dynamic_imbalance_gmm2);

fprintf('\n--- 2. PEAK LOADS PER PART (at max speed) -------------------------------------\n');
fprintf('%-22s %12s %12s %12s\n', '', part_names{:});
fprintf('%-22s %12.4g %12.4g %12.4g\n', 'Shake force [N]',       peak_shake_N);
fprintf('%-22s %12.4g %12.4g %12.4g\n', 'Rock moment [N*m]',     peak_rock_Nm);
fprintf('%-22s %12.4g %12.4g %12.4g\n', 'Bearing kicks [N]',     peak_kick_N);
fprintf('%-22s %12.4g %12.4g %12.4g\n', 'Ramp twist [N*m]',      peak_twist_Nm);

fprintf('\n--- 3. BEARING FREQUENCIES (at max speed) -------------------------------------\n');
fprintf('%-22s %12s %12s %12s\n', '', part_names{:});
fprintf('%-22s %12.2f %12.2f %12.2f\n', '1x imbalance [Hz]', part_max_speed_hz);
for k = 1:number_of_kicks
    fprintf('%-22s %12.2f %12.2f %12.2f\n', [kick_names{k}, ' [Hz]'], kick_frequency_hz(:, k));
end
fprintf('%-22s %12.3f %12.3f %12.3f\n', 'Cone pass harmonic [x]', kick_harmonic(:, 3));

fprintf('\n--- 4. TOTAL LOADS AT THE REFERENCE POINT  (MAIN ANSWER) ----------------------\n');
fprintf('%-34s %14s %14s %14s\n', '', 'Worst case', 'Random phase', 'Simulated');
fprintf('%-34s %14.4g %14.4g %14.4g\n', 'Radial force [N]',         worst_force_N,   rss_force_N,   simulated_peak_force_N);
fprintf('%-34s %14.4g %14.4g %14.4g\n', 'Radial moment Mx/My [N*m]', worst_moment_Nm, rss_moment_Nm, simulated_peak_moment_Nm);
fprintf('%-34s %14s %14s %14.4g\n',     'Net twist Mz (ramps only) [N*m]', '-', '-', simulated_peak_twist_Nm);

fprintf('\n--- 5. PEAK LOAD ON EACH BEARING ---------------------------------------------\n');
fprintf('%-22s %12s %12s %12s\n', '', part_names{:});
fprintf('%-22s %12.4g %12.4g %12.4g\n', 'Bearing A [N]', bearing_A_peak_N);
fprintf('%-22s %12.4g %12.4g %12.4g\n', 'Bearing B [N]', bearing_B_peak_N);
fprintf('%-22s %12.1f %12.1f %12.1f\n', 'Eff. spacing [mm]', bearing_spacing_m * 1e3);

fprintf('\n--- 6. MOMENTUM CHECK ---------------------------------------------------------\n');
fprintf('Arm momentum at max speed:        %10.4f N*m*s\n', arm_momentum_at_max_Nms);
fprintf('Net momentum at max speed:        %10.4f N*m*s  (%.2f%% of the arm)\n', net_momentum_at_max_Nms, leftover_percent);
fprintf('Wheel speed that cancels the arm: %10.1f RPM  (you entered %.0f and %.0f RPM)\n', ...
    wheel_speed_needed_rpm, part_max_speed_rpm(wheel_indices));
fprintf('Peak net twist on the structure:  %10.4f N*m  (during the ramps)\n', max(abs(net_torque_Nm)));
if part_spin_direction(arm_index) == part_spin_direction(1) || part_spin_direction(arm_index) == part_spin_direction(3)
    warning('A wheel spins the same direction as the arm, so it adds to the arm momentum instead of cancelling it.');
end

fprintf('\n--- 7. CAMPBELL DIAGRAM: NATURAL FREQUENCIES AT 0 RPM (ESTIMATES) -----------\n');
for p = 1:number_of_parts
    fprintf('%-10s', part_names{p});
    mode_names = campbell_mode_names{p};
    for m = 1:numel(mode_names)
        fprintf('  %s: %.1f Hz', mode_names{m}, campbell_modes_hz{p}(m, 1));
    end
    fprintf('\n');
end
fprintf('\nSpeeds where a vibration source crosses a natural frequency (speeds to watch):\n');
if isempty(crossing_list)
    fprintf('  None found up to %.0f%% of max speed.\n', 100 * campbell_speed_factor);
else
    for c = 1:size(crossing_list, 1)
        if crossing_list{c, 5}
            range_text = 'INSIDE operating range';
        else
            range_text = 'above max speed';
        end
        fprintf('  %-8s %-14s crosses %-24s at %8.1f RPM  (%s)\n', crossing_list{c, 1:4}, range_text);
    end
end

fprintf('\n--- 8. SANITY CHECKS ---------------------------------------------------------\n');
for c = 1:numel(check_names)
    if check_passed(c)
        fprintf('  PASS  %s\n', check_names{c});
    else
        fprintf('  FAIL  %s\n', check_names{c});
    end
end

fprintf('\n--- 9. ASSUMPTIONS -----------------------------------------------------------\n');
fprintf('  1. Rigid mounting: no structural bending or resonance amplification.\n');
fprintf('     Results are valid when speeds stay away from the crossings listed above.\n');
fprintf('  2. Eccentricity, runout and tilt come from tolerances, not measurements.\n');
fprintf('  3. Imbalance loads scale with speed squared (papers measured 2.1 to 2.25).\n');
fprintf('  4. Bearing kick strength is an estimate: %.1f%% of the imbalance per kick.\n', 100 * bearing_kick_ratio);
fprintf('  5. Unknown phases: worst case adds all peaks, random phase uses root-sum-square.\n');
fprintf('  6. Whirl frequencies use estimated bearing stiffness (+/-%.0f%% band).\n', 100 * stiffness_uncertainty);
fprintf('  7. Not included: broadband noise, resonance amplification, platform gyroscopics.\n');
fprintf('%s\n', line_text);


%% ========================================================================
%  SECTION 14: PLOTS
%  ------------------------------------------------------------------------
%  Every plot gets its own window (one graph per window, no sub-panels).
%  Each window is listed in figure_handles / figure_names and all of them
%  are saved together at the end of this section.
%  ========================================================================

speed_axis_scale = linspace(0, 1.2, 200);                       % 0 to 120% of max speed
arm_axis_rpm     = speed_axis_scale * part_max_speed_rpm(arm_index);
plot_letters     = {'a', 'b', 'c', 'd', 'e', 'f'};
operating_color  = [0.4 0.4 0.4];
figure_handles   = {};
figure_names     = {};

% ------------------------------------------------------------------------
% PLOT 1: SPEED vs FORCE
%   1a, 1b, 1c: one rotor each, against its own speed (0 to 120% of its max)
%   1d:         all rotors together, against arm speed (wheels follow)
% ------------------------------------------------------------------------
if make_plot_1_speed_vs_force
    for p = 1:number_of_parts
        figure_name = sprintf('Plot 1%s - %s Force vs Speed', plot_letters{p}, part_names{p});
        figure_handles{end + 1} = figure('Name', figure_name, 'NumberTitle', 'off', 'Color', 'w');
        figure_names{end + 1}   = figure_name;
        hold on; grid on; box on;

        part_speed_rpm   = speed_axis_scale * part_max_speed_rpm(p);
        part_speed_rad_s = part_speed_rpm * 2 * pi / 60;
        part_shake_N     = static_imbalance_kgm(p) * part_speed_rad_s .^ 2;
        part_kicks_N     = part_shake_N * number_of_kicks * bearing_kick_ratio;
        part_total_N     = part_shake_N + part_kicks_N;

        plot(part_speed_rpm, part_shake_N, '-',  'LineWidth', 2,   'Color', part_colors(p, :));
        plot(part_speed_rpm, part_kicks_N, '--', 'LineWidth', 1.5, 'Color', part_colors(p, :));
        plot(part_speed_rpm, part_total_N, '-',  'LineWidth', 2.5, 'Color', total_color);
        plot([1 1] * part_max_speed_rpm(p), [0 max(part_total_N)], ':', 'Color', operating_color, 'LineWidth', 1.5);

        operating_total_N = peak_shake_N(p) + peak_kick_N(p);
        plot(part_max_speed_rpm(p), operating_total_N, 'o', 'MarkerSize', 8, ...
             'MarkerFaceColor', part_colors(p, :), 'MarkerEdgeColor', 'k');
        text(part_max_speed_rpm(p), operating_total_N, sprintf('  %.3g N', operating_total_N), 'FontSize', 9);

        xlim([0 part_speed_rpm(end)]);
        xlabel(sprintf('%s speed [RPM]', part_names{p})); ylabel('Force [N]');
        title(sprintf('%s: force vs its own speed (max %.0f RPM)', part_names{p}, part_max_speed_rpm(p)));
        legend({'Shake (imbalance)', 'Bearing kicks (all 4 lined up)', 'Total for this rotor', 'Operating speed'}, ...
               'Location', 'northwest');
    end

    figure_name = 'Plot 1d - All Rotors Force vs Speed';
    figure_handles{end + 1} = figure('Name', figure_name, 'NumberTitle', 'off', 'Color', 'w');
    figure_names{end + 1}   = figure_name;
    hold on; grid on; box on;

    worst_curve = zeros(size(speed_axis_scale));
    rss_curve   = zeros(size(speed_axis_scale));
    part_curves = zeros(number_of_parts, numel(speed_axis_scale));
    for i = 1:numel(speed_axis_scale)
        worst_curve(i)    = calc_worst_force_N(1, 1, bearing_kick_ratio, speed_axis_scale(i));
        rss_curve(i)      = calc_rss_force_N(1, 1, bearing_kick_ratio, speed_axis_scale(i));
        part_curves(:, i) = (calc_shake_N(1, 1, speed_axis_scale(i)) * (1 + number_of_kicks * bearing_kick_ratio))';
    end
    for p = 1:number_of_parts
        plot(arm_axis_rpm, part_curves(p, :), part_line_styles{p}, 'LineWidth', 1.5, 'Color', part_colors(p, :));
    end
    plot(arm_axis_rpm, worst_curve, 'k-',  'LineWidth', 2.5);
    plot(arm_axis_rpm, rss_curve,   'k--', 'LineWidth', 2);
    plot([1 1] * part_max_speed_rpm(arm_index), [0 max(worst_curve)], ':', 'Color', operating_color, 'LineWidth', 1.5);
    text(part_max_speed_rpm(arm_index), worst_force_N, sprintf('  worst %.3g N', worst_force_N), 'FontSize', 9);
    xlim([0 arm_axis_rpm(end)]);
    xlabel(sprintf('Arm speed [RPM]   (wheels at %.0f RPM when arm at %.0f RPM)', ...
        part_max_speed_rpm(1), part_max_speed_rpm(arm_index)));
    ylabel('Force [N]');
    title('All rotors together: system force vs arm speed');
    legend([part_names, {'Total worst case', 'Total random phase', 'Operating point'}], 'Location', 'northwest');
end

% ------------------------------------------------------------------------
% PLOT 2: SPEED vs TORQUE
%   2a, 2b, 2c: one rotor each, against its own speed (0 to 120% of its max)
%   2d:         all rotors together, against arm speed (wheels follow)
% ------------------------------------------------------------------------
if make_plot_2_speed_vs_torque
    for p = 1:number_of_parts
        figure_name = sprintf('Plot 2%s - %s Moment vs Speed', plot_letters{p}, part_names{p});
        figure_handles{end + 1} = figure('Name', figure_name, 'NumberTitle', 'off', 'Color', 'w');
        figure_names{end + 1}   = figure_name;
        hold on; grid on; box on;

        part_speed_rpm   = speed_axis_scale * part_max_speed_rpm(p);
        part_speed_rad_s = part_speed_rpm * 2 * pi / 60;
        part_shake_N     = static_imbalance_kgm(p) * part_speed_rad_s .^ 2;
        part_rock_Nm     = dynamic_imbalance_kgm2(p) * part_speed_rad_s .^ 2;
        part_lever_Nm    = abs(cm_distance_m(p)) * part_shake_N ...
                         + abs(bearing_mid_distance_m(p)) * part_shake_N * number_of_kicks * bearing_kick_ratio;
        part_total_Nm    = part_rock_Nm + part_lever_Nm;

        plot(part_speed_rpm, part_rock_Nm,  '-',  'LineWidth', 2,   'Color', part_colors(p, :));
        plot(part_speed_rpm, part_lever_Nm, '--', 'LineWidth', 1.5, 'Color', part_colors(p, :));
        plot(part_speed_rpm, part_total_Nm, '-',  'LineWidth', 2.5, 'Color', total_color);
        plot([1 1] * part_max_speed_rpm(p), [0 max(part_total_Nm)], ':', 'Color', operating_color, 'LineWidth', 1.5);

        operating_total_Nm = interp1(part_speed_rpm, part_total_Nm, part_max_speed_rpm(p));
        plot(part_max_speed_rpm(p), operating_total_Nm, 'o', 'MarkerSize', 8, ...
             'MarkerFaceColor', part_colors(p, :), 'MarkerEdgeColor', 'k');
        text(part_max_speed_rpm(p), operating_total_Nm, sprintf('  %.3g N*m', operating_total_Nm), 'FontSize', 9);

        xlim([0 part_speed_rpm(end)]);
        xlabel(sprintf('%s speed [RPM]', part_names{p})); ylabel('Moment [N*m]');
        title(sprintf('%s: moment vs its own speed   (ramp twist Mz: %.3g N*m)', part_names{p}, peak_twist_Nm(p)));
        legend({'Rock (tilt)', sprintf('Force x distance to reference (%.3f m)', abs(cm_distance_m(p))), ...
                'Total for this rotor', 'Operating speed'}, 'Location', 'northwest');
    end

    figure_name = 'Plot 2d - All Rotors Moment vs Speed';
    figure_handles{end + 1} = figure('Name', figure_name, 'NumberTitle', 'off', 'Color', 'w');
    figure_names{end + 1}   = figure_name;
    hold on; grid on; box on;

    worst_curve = zeros(size(speed_axis_scale));
    rss_curve   = zeros(size(speed_axis_scale));
    part_curves = zeros(number_of_parts, numel(speed_axis_scale));
    for i = 1:numel(speed_axis_scale)
        worst_curve(i)    = calc_worst_moment_Nm(1, 1, 1, bearing_kick_ratio, speed_axis_scale(i));
        rss_curve(i)      = calc_rss_moment_Nm(1, 1, 1, bearing_kick_ratio, speed_axis_scale(i));
        shake_now         = calc_shake_N(1, 1, speed_axis_scale(i));
        part_curves(:, i) = (calc_rock_Nm(1, 1, speed_axis_scale(i)) + abs(cm_distance_m) .* shake_now ...
                          + abs(bearing_mid_distance_m) .* shake_now * number_of_kicks * bearing_kick_ratio)';
    end
    for p = 1:number_of_parts
        plot(arm_axis_rpm, part_curves(p, :), part_line_styles{p}, 'LineWidth', 1.5, 'Color', part_colors(p, :));
    end
    plot(arm_axis_rpm, worst_curve, 'k-',  'LineWidth', 2.5);
    plot(arm_axis_rpm, rss_curve,   'k--', 'LineWidth', 2);
    plot([1 1] * part_max_speed_rpm(arm_index), [0 max(worst_curve)], ':', 'Color', operating_color, 'LineWidth', 1.5);
    text(part_max_speed_rpm(arm_index), worst_moment_Nm, sprintf('  worst %.3g N*m', worst_moment_Nm), 'FontSize', 9);
    xlim([0 arm_axis_rpm(end)]);
    xlabel('Arm speed [RPM]   (wheels follow)'); ylabel('Moment Mx / My at reference [N*m]');
    title(sprintf('All rotors together: moment vs arm speed   (net ramp twist Mz: %.3g N*m)', simulated_peak_twist_Nm));
    legend([part_names, {'Total worst case', 'Total random phase', 'Operating point'}], 'Location', 'northwest');
end

% ------------------------------------------------------------------------
% PLOT 3: FREQUENCY vs AMPLITUDE (spectrum during the hold)
%   3a: total force Fx      3b: total moment Mx
% ------------------------------------------------------------------------
if make_plot_3_spectrum
    hold_samples = find(in_hold);
    if numel(hold_samples) < 16
        warning('Hold time too short for a spectrum. Plot 3 skipped.');
    else
        n = numel(hold_samples);
        window = 0.5 - 0.5 * cos(2 * pi * (0:n-1) / (n - 1));   % Hann window
        fft_length = 8 * 2 ^ nextpow2(n);                       % zero padding: peaks read at their true height
        frequency_hz = (0:fft_length / 2) * sample_rate_hz / fft_length;

        signals       = {total_force_x_N(hold_samples), total_moment_x_Nm(hold_samples)};
        signal_names  = {'Total force Fx [N]', 'Total moment Mx [N*m]'};
        spectrum_tags = {'Force Spectrum', 'Moment Spectrum'};

        for s = 1:2
            figure_name = sprintf('Plot 3%s - %s', plot_letters{s}, spectrum_tags{s});
            figure_handles{end + 1} = figure('Name', figure_name, 'NumberTitle', 'off', 'Color', 'w');
            figure_names{end + 1}   = figure_name;
            hold on; grid on; box on;

            signal = signals{s} - mean(signals{s});
            spectrum = fft(signal .* window, fft_length);
            amplitude = 2 * abs(spectrum(1:fft_length / 2 + 1)) / sum(window);
            floor_level = max(amplitude) * 1e-6;
            semilogy(frequency_hz, max(amplitude, floor_level), 'k', 'LineWidth', 1);
            set(gca, 'YScale', 'log');

            % Label each expected source frequency (skip a label if another part already has one there)
            labeled_frequencies = [];
            for p = 1:number_of_parts
                source_freqs = [part_max_speed_hz(p), kick_frequency_hz(p, :)];
                source_labels = [{'1x'}, kick_names];
                for k = 1:numel(source_freqs)
                    [~, nearest] = min(abs(frequency_hz - source_freqs(k)));
                    search = max(1, nearest - 10):min(numel(amplitude), nearest + 10);
                    [~, offset] = max(amplitude(search));
                    nearest = search(offset);                  % snap to the top of the peak
                    if any(abs(labeled_frequencies - nearest) <= 10) || amplitude(nearest) < 100 * floor_level
                        continue;                              % already labeled, or no visible peak here
                    end
                    labeled_frequencies(end + 1) = nearest;
                    plot(frequency_hz(nearest), max(amplitude(nearest), floor_level), 'v', ...
                         'MarkerSize', 7, 'MarkerFaceColor', part_colors(p, :), 'MarkerEdgeColor', part_colors(p, :));
                    text(frequency_hz(nearest), max(amplitude(nearest), floor_level) * 1.6, ...
                         sprintf('%s %s', part_names{p}, source_labels{k}), ...
                         'FontSize', 8, 'Rotation', 90, 'Color', part_colors(p, :));
                end
            end
            xlim([0, 1.15 * fastest_frequency_hz]);
            ylim([floor_level, max(amplitude) * 50]);
            xlabel('Frequency [Hz]'); ylabel(['Amplitude: ', signal_names{s}]);
            title(sprintf('Spectrum during the hold: %s', signal_names{s}));
        end
    end
end

% ------------------------------------------------------------------------
% PLOT 4: CAMPBELL DIAGRAM (nutation / precession + source lines)
%   4a, 4b, 4c: one rotor each
% ------------------------------------------------------------------------
if make_plot_4_campbell
    source_styles = {'-', '--', '-.', ':', '--'};

    for p = 1:number_of_parts
        figure_name = sprintf('Plot 4%s - %s Campbell Diagram', plot_letters{p}, part_names{p});
        figure_handles{end + 1} = figure('Name', figure_name, 'NumberTitle', 'off', 'Color', 'w');
        figure_names{end + 1}   = figure_name;
        hold on; grid on; box on;

        speed_rpm = campbell_speed_rpm{p};
        modes     = campbell_modes_hz{p};
        sources   = campbell_sources_hz{p};
        names     = campbell_mode_names{p};

        % Mode lines with uncertainty band
        mode_colors = lines(size(modes, 1));
        legend_handles = [];
        legend_labels  = {};
        for m = 1:size(modes, 1)
            plot(speed_rpm, campbell_modes_low{p}(m, :),  ':', 'Color', mode_colors(m, :), 'LineWidth', 1);
            plot(speed_rpm, campbell_modes_high{p}(m, :), ':', 'Color', mode_colors(m, :), 'LineWidth', 1);
            legend_handles(end + 1) = plot(speed_rpm, modes(m, :), '-', 'Color', mode_colors(m, :), 'LineWidth', 2.5);
            legend_labels{end + 1}  = names{m};
        end

        % Source lines
        for s = 1:size(sources, 1)
            legend_handles(end + 1) = plot(speed_rpm, sources(s, :), source_styles{s}, ...
                'Color', part_colors(p, :), 'LineWidth', 1.3);
            legend_labels{end + 1} = campbell_source_names{s};
        end

        % Crossings
        for c = 1:size(crossing_list, 1)
            if strcmp(crossing_list{c, 1}, part_names{p})
                s = find(strcmp(campbell_source_names, crossing_list{c, 2}));
                crossing_rpm = crossing_list{c, 4};
                crossing_hz  = interp1(speed_rpm, sources(s, :), crossing_rpm);
                plot(crossing_rpm, crossing_hz, 'kx', 'MarkerSize', 12, 'LineWidth', 2.5);
            end
        end

        % Operating speed
        top_source_hz = max(sources(:, end));
        lowest_mode_hz = min(modes(:, 1));
        y_top = 1.15 * max(top_source_hz, lowest_mode_hz);
        plot([1 1] * part_max_speed_rpm(p), [0 y_top], 'k:', 'LineWidth', 1.5);

        ylim([0 y_top]);
        xlim([0 speed_rpm(end)]);
        xlabel(sprintf('%s speed [RPM]', part_names{p}));
        ylabel('Frequency [Hz]');
        title(sprintf('%s Campbell diagram  (x = crossing, dotted = +/-%.0f%% stiffness)', part_names{p}, 100 * stiffness_uncertainty));
        for m = 1:size(modes, 1)
            if min(modes(m, :)) > y_top
                legend_labels{m} = sprintf('%s (off chart, %.0f Hz)', legend_labels{m}, modes(m, 1));
            end
        end
        legend(legend_handles, legend_labels, 'Location', 'east', 'FontSize', 8);
    end
end

% ------------------------------------------------------------------------
% PLOT 5: WATERFALL (frequency vs speed, color = amplitude)
%   5a: force      5b: moment at reference
% ------------------------------------------------------------------------
if make_plot_5_waterfall
    waterfall_scales = linspace(0.02, 1.2, 70);

    point_freq_hz   = [];
    point_speed_rpm = [];
    point_force_N   = [];
    point_moment_Nm = [];
    for i = 1:numel(waterfall_scales)
        scale = waterfall_scales(i);
        for p = 1:number_of_parts
            spin_hz    = part_max_speed_hz(p) * scale;
            shake_now  = static_imbalance_kgm(p) * (part_max_speed_rad_s(p) * scale) ^ 2;
            rock_now   = dynamic_imbalance_kgm2(p) * (part_max_speed_rad_s(p) * scale) ^ 2;
            kick_now   = bearing_kick_ratio * shake_now;

            freqs   = [spin_hz, kick_harmonic(p, :) * spin_hz];
            forces  = [shake_now, kick_now * ones(1, number_of_kicks)];
            moments = [rock_now + abs(cm_distance_m(p)) * shake_now, ...
                       abs(bearing_mid_distance_m(p)) * kick_now * ones(1, number_of_kicks)];

            point_freq_hz   = [point_freq_hz,   freqs];
            point_speed_rpm = [point_speed_rpm, part_max_speed_rpm(arm_index) * scale * ones(1, numel(freqs))];
            point_force_N   = [point_force_N,   forces];
            point_moment_Nm = [point_moment_Nm, moments];
        end
    end

    waterfall_values = {point_force_N, point_moment_Nm};
    waterfall_labels = {'log10( force amplitude [N] )', 'log10( moment amplitude [N*m] )'};
    waterfall_tags   = {'Force Waterfall', 'Moment Waterfall'};
    waterfall_titles = {'Waterfall: force', 'Waterfall: moment at reference'};
    for s = 1:2
        figure_name = sprintf('Plot 5%s - %s', plot_letters{s}, waterfall_tags{s});
        figure_handles{end + 1} = figure('Name', figure_name, 'NumberTitle', 'off', 'Color', 'w');
        figure_names{end + 1}   = figure_name;
        hold on; grid on; box on;

        values = waterfall_values{s};
        keep = values > 0;
        log_values = log10(values(keep));
        scatter(point_freq_hz(keep), point_speed_rpm(keep), 18, log_values, 'filled');
        colormap(jet);
        caxis([min(log_values), max(log_values)]);
        color_bar = colorbar;
        ylabel(color_bar, waterfall_labels{s});
        plot([0 1.15 * fastest_frequency_hz * 1.2], [1 1] * part_max_speed_rpm(arm_index), 'k:', 'LineWidth', 1.5);
        xlim([0, 1.15 * fastest_frequency_hz * 1.2]);
        xlabel('Frequency [Hz]'); ylabel('Arm speed [RPM]  (wheels follow)');
        title(waterfall_titles{s});
    end
end

% ------------------------------------------------------------------------
% PLOT 6: LOADS OVER TIME
%   6a Fx, 6b Fy, 6c Mx, 6d My, 6e Mz, 6f zoom on the hold
% ------------------------------------------------------------------------
if make_plot_6_loads_over_time
    plot_step = max(1, floor(number_of_samples / 100000));
    plot_index = 1:plot_step:number_of_samples;

    time_signals = {total_force_x_N, total_force_y_N, total_moment_x_Nm, total_moment_y_Nm, total_moment_z_Nm};
    time_labels  = {'Fx [N]', 'Fy [N]', 'Mx [N*m]', 'My [N*m]', 'Mz net twist [N*m]'};
    time_tags    = {'Total Fx Over Time', 'Total Fy Over Time', 'Total Mx Over Time', 'Total My Over Time', 'Net Twist Mz Over Time'};

    for s = 1:numel(time_signals)
        figure_name = sprintf('Plot 6%s - %s', plot_letters{s}, time_tags{s});
        figure_handles{end + 1} = figure('Name', figure_name, 'NumberTitle', 'off', 'Color', 'w');
        figure_names{end + 1}   = figure_name;
        hold on; grid on; box on;

        signal = time_signals{s};
        plot(time_s(plot_index), signal(plot_index), 'Color', total_color, 'LineWidth', 0.8);
        [peak_value, peak_index] = max(abs(signal));
        plot(time_s(peak_index), signal(peak_index), 'ro', 'MarkerFaceColor', 'r');
        text(time_s(peak_index), signal(peak_index), sprintf('  peak %.3g', peak_value), 'FontSize', 9, 'Color', 'r');
        xlim([0 total_time_s]);
        xlabel('Time [s]'); ylabel(time_labels{s});
        title(sprintf('%s at the reference point over the full run', time_labels{s}));
    end

    % Zoomed view: two revolutions of the slowest part in the middle of the hold
    figure_name = 'Plot 6f - Zoom on the Hold';
    figure_handles{end + 1} = figure('Name', figure_name, 'NumberTitle', 'off', 'Color', 'w');
    figure_names{end + 1}   = figure_name;
    hold on; grid on; box on;

    zoom_length_s = 2 / max(min(part_max_speed_hz), eps);
    zoom_start_s  = spin_up_time_s + hold_time_s / 2;
    in_zoom = time_s >= zoom_start_s & time_s <= zoom_start_s + zoom_length_s;
    plot(time_s(in_zoom), total_force_x_N(in_zoom), 'Color', total_color, 'LineWidth', 1.2);
    plot(time_s(in_zoom), total_force_y_N(in_zoom), 'Color', [0.6 0.6 0.6], 'LineWidth', 1.2);
    legend({'Fx', 'Fy'}, 'Location', 'northeast');
    xlabel('Time [s]'); ylabel('Force [N]');
    title('Zoom: middle of the hold (two revolutions of the slowest part)');
end

% ------------------------------------------------------------------------
% PLOT 7: MOMENTUM CHECK
%   7a: spin momentum      7b: ramp twist
% ------------------------------------------------------------------------
if make_plot_7_momentum
    plot_step = max(1, floor(number_of_samples / 20000));
    plot_index = 1:plot_step:number_of_samples;

    figure_name = 'Plot 7a - Momentum';
    figure_handles{end + 1} = figure('Name', figure_name, 'NumberTitle', 'off', 'Color', 'w');
    figure_names{end + 1}   = figure_name;
    hold on; grid on; box on;
    for p = 1:number_of_parts
        plot(time_s(plot_index), part_momentum_Nms(p, plot_index), part_line_styles{p}, 'LineWidth', 2, 'Color', part_colors(p, :));
    end
    plot(time_s(plot_index), net_momentum_Nms(plot_index), 'k--', 'LineWidth', 2.5);
    xlabel('Time [s]'); ylabel('Spin momentum [N*m*s]');
    title(sprintf('Momentum: net at max speed = %.3g N*m*s  (%.2f%% of the arm)', net_momentum_at_max_Nms, leftover_percent));
    legend([part_names, {'Net (want about 0)'}], 'Location', 'northeast');

    figure_name = 'Plot 7b - Ramp Twist';
    figure_handles{end + 1} = figure('Name', figure_name, 'NumberTitle', 'off', 'Color', 'w');
    figure_names{end + 1}   = figure_name;
    hold on; grid on; box on;
    for p = 1:number_of_parts
        plot(time_s(plot_index), twist_z_Nm(p, plot_index), part_line_styles{p}, 'LineWidth', 2, 'Color', part_colors(p, :));
    end
    plot(time_s(plot_index), net_torque_Nm(plot_index), 'k--', 'LineWidth', 2.5);
    xlabel('Time [s]'); ylabel('Twist on structure [N*m]');
    title('Ramp twist: each part and net (net should be about 0 if momentum is balanced)');
    legend([part_names, {'Net'}], 'Location', 'northeast');
end

% ------------------------------------------------------------------------
% SAVE EVERY PLOT AS A PNG
%   File name = window name with spaces replaced, e.g. Plot_1a_Wheel_1_Force_vs_Speed.png
% ------------------------------------------------------------------------
if save_plots
    if ~exist(plot_folder, 'dir')
        mkdir(plot_folder);
    end
    drawnow;
    number_saved = 0;
    for i = 1:numel(figure_handles)
        this_figure = figure_handles{i};
        if ~ishghandle(this_figure)   % window was closed or replaced: look it up by name
            this_figure = findobj('Type', 'figure', 'Name', figure_names{i});
        end
        file_name = [strrep(strrep(figure_names{i}, ' - ', '_'), ' ', '_'), '.png'];
        if isempty(this_figure)
            warning('%s could not be saved (its window was closed).', figure_names{i});
        else
            print(this_figure(1), fullfile(plot_folder, file_name), '-dpng', '-r150');
            number_saved = number_saved + 1;
        end
    end
    fprintf('%d plots saved in folder: %s\n', number_saved, plot_folder);
end
