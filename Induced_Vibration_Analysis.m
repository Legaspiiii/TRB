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
%  Directions:
%    lateral = sideways (perpendicular to the shaft)
%    axial   = along the shaft (spin axis)
%
%  How to use:
%    1. Edit SECTION 1 only (your numbers).
%    2. Run the script.
%    3. Your answer is the "YOUR ANSWER" box at the very bottom of the
%       Command Window: Fx, Fy, Fz, Mx, My, Mz for each rotor and the total.
%       Everything above it is supporting detail. The same numbers are
%       saved as final_Fx_N ... final_Mz_Nm in the Workspace.
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
%  SECTION 6: TOTALS PER ROTOR AND FOR THE WHOLE SYSTEM  (worst case)
%  ------------------------------------------------------------------------
%  Worst case = every peak lines up at the same moment (safe design value).
%  ========================================================================

rotor_lateral_N = static_imbalance_force_N + bearing_lateral_N;   % lateral force of each rotor
rotor_axial_N   = bearing_axial_N;                             % axial force of each rotor (bearings only)
rotor_moment_Nm = dynamic_imbalance_moment_Nm + lever_arm_m .* rotor_lateral_N;   % dynamic imbalance + force x distance
rotor_twist_Nm  = Izz_kgm2 .* speed_rad_s / ramp_time_s;       % twist during spin-up / spin-down

total_lateral_N = sum(rotor_lateral_N);
total_axial_N   = sum(rotor_axial_N);
total_moment_Nm = sum(rotor_moment_Nm);
net_twist_Nm    = abs(sum(spin_direction .* Izz_kgm2 .* speed_rad_s)) / ramp_time_s;   % wheels cancel the arm


%% ========================================================================
%  SECTION 7: PRINTED RESULTS
%  ========================================================================

line_text = repmat('=', 1, 92);

% ---- Table 1: each rotor at max speed ----
fprintf('\n%s\n DETAIL 1: EACH ROTOR AT MAX SPEED\n%s\n', line_text, line_text);
fprintf('%-34s %14s %14s %14s\n', '', rotor_names{1}, rotor_names{2}, rotor_names{3});
fprintf('%-34s %14.0f %14.0f %14.0f\n', 'Speed [RPM]',                         max_speed_rpm .* spin_direction);
fprintf('%-34s %14.4g %14.4g %14.4g\n', 'Static imbalance force [N]',           static_imbalance_force_N);
fprintf('%-34s %14.4g %14.4g %14.4g\n', 'Bearing lateral force [N]',            bearing_lateral_N);
fprintf('%-34s %14.4g %14.4g %14.4g\n', 'Bearing axial force [N]',              bearing_axial_N);
fprintf('%-34s %14.4g %14.4g %14.4g\n', 'Dynamic imbalance moment [N*m]',       dynamic_imbalance_moment_Nm);
fprintf('%-34s %14.4g %14.4g %14.4g\n', 'Total moment at reference [N*m]',      rotor_moment_Nm);
fprintf('%-34s %14.4g %14.4g %14.4g\n', 'Twist during spin-up/down [N*m]',      rotor_twist_Nm);

% ---- Table 2: every disturbance with its frequency (harmonics of RPM) ----
fprintf('\n%s\n DETAIL 2: EVERY DISTURBANCE AT MAX SPEED  (frequency = harmonic x RPM / 60)\n%s\n', line_text, line_text);
fprintf('%-9s %-24s %10s %12s %14s %14s\n', 'Rotor', 'Source', 'Harmonic', 'Freq [Hz]', 'Lateral [N]', 'Axial [N]');
for r = 1:number_of_rotors
    fprintf('%-9s %-24s %10.3f %12.2f %14.4g %14.4g\n', rotor_names{r}, 'Static imbalance (1x)', 1, speed_hz(r), static_imbalance_force_N(r), 0);
    for k = 1:number_of_harmonics
        fprintf('%-9s %-24s %10.3f %12.2f %14.4g %14.4g\n', rotor_names{r}, ['Bearing ', harmonic_names{k}], ...
            bearing_harmonic(r, k), bearing_frequency_hz(r, k), bearing_lateral_each_N(r), bearing_axial_each_N(r));
    end
end

fprintf('(Details above. YOUR ANSWER is printed at the very bottom.)\n');


%% ========================================================================
%  SECTION 8: PLOTS
%  ------------------------------------------------------------------------
%  Plot 1: lateral force vs RPM      Plot 2: axial force vs RPM
%  Every load grows with speed squared: load(speed) = load(max) x (speed/max)^2
%  The wheels follow the arm, so the x-axis is arm speed.
%  ========================================================================

if make_plots
    speed_fraction = linspace(0, 1.2, 121)';          % 0 to 120% of max speed
    arm_axis_rpm   = speed_fraction * max_speed_rpm(arm_index);
    curve_shape    = speed_fraction .^ 2;
    x_label_text   = sprintf('Arm speed [RPM]    (wheel speed = %.2f x arm speed)', max_speed_rpm(1) / max_speed_rpm(arm_index));

    plot_rotor_values = {rotor_lateral_N, rotor_axial_N};
    plot_total_values = [total_lateral_N, total_axial_N];
    plot_titles       = {'Plot 1: Lateral force vs RPM', 'Plot 2: Axial force vs RPM'};
    plot_ylabels      = {'Lateral force [N]', 'Axial force [N]'};
    plot_files        = {'Plot_1_Lateral_Force_vs_RPM.png', 'Plot_2_Axial_Force_vs_RPM.png'};
    rotor_colors      = [0.00 0.45 0.74; 0.85 0.33 0.10; 0.47 0.67 0.19];
    rotor_lines       = {'-', '-.', '--'};

    for n = 1:2
        fprintf('Drawing plot %d...\n', n);
        try
            figure(n);
            clf;
            try
                set(gcf, 'Renderer', 'painters');
            catch
            end
            hold on;
            grid on;
            box on;
            rotor_values = plot_rotor_values{n};
            for r = 1:number_of_rotors
                plot(arm_axis_rpm, rotor_values(r) * curve_shape, rotor_lines{r}, ...
                     'Color', rotor_colors(r, :), 'LineWidth', 2);
            end
            plot(arm_axis_rpm, plot_total_values(n) * curve_shape, 'k-', 'LineWidth', 2.5);
            plot(max_speed_rpm(arm_index), plot_total_values(n), 'ko', 'MarkerFaceColor', 'k');
            text(max_speed_rpm(arm_index), plot_total_values(n), sprintf('%.3g N at max speed   ', plot_total_values(n)), ...
                 'HorizontalAlignment', 'right');
            xlabel(x_label_text);
            ylabel(plot_ylabels{n});
            title(plot_titles{n});
            legend([rotor_names, {'Total'}], 'Location', 'northwest');
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
%  SECTION 9: YOUR ANSWER  -  Fx, Fy, Fz, Mx, My, Mz
%  ------------------------------------------------------------------------
%  Axes:  z = along the shaft (spin axis),  x and y = sideways.
%
%  Fx, Fy : the lateral force ROTATES with each rotor, so it shows up in x
%           and y as two sine waves with the SAME peak, a quarter turn apart.
%           -> peak Fx = peak Fy = lateral force
%  Fz     : axial force along the shaft (bearing harmonics)
%  Mx, My : the tipping moment also rotates -> peak Mx = peak My = moment
%           (dynamic imbalance + lateral force x distance to the reference)
%  Mz     : twist about the shaft (only during spin-up / spin-down)
%
%  All values are PEAK amplitudes, worst case, at max speed.
%  Printed last so it is at the bottom of the Command Window, and saved
%  as final_ variables in the Workspace.
%  ========================================================================

final_Fx_N  = [rotor_lateral_N, total_lateral_N];      % [Wheel 1, Arm, Wheel 2, TOTAL]
final_Fy_N  = [rotor_lateral_N, total_lateral_N];
final_Fz_N  = [rotor_axial_N,   total_axial_N];
final_Mx_Nm = [rotor_moment_Nm, total_moment_Nm];
final_My_Nm = [rotor_moment_Nm, total_moment_Nm];
final_Mz_Nm = [rotor_twist_Nm,  net_twist_Nm];         % total Mz = net (wheels cancel the arm)

answer_rows   = {final_Fx_N, final_Fy_N, final_Fz_N, final_Mx_Nm, final_My_Nm, final_Mz_Nm};
answer_labels = {'Fx  [N]   (sideways)', 'Fy  [N]   (sideways)', 'Fz  [N]   (along shaft)', ...
                 'Mx  [N*m] (tipping)',  'My  [N*m] (tipping)',  'Mz  [N*m] (twist, ramps)'};

box_line = repmat('#', 1, 86);
fprintf('\n\n%s\n', box_line);
fprintf('#%84s#\n', '');
fprintf('#%-84s#\n', '   YOUR ANSWER: induced forces and moments  (peak, worst case, at max speed)');
fprintf('#%84s#\n', '');
fprintf('%s\n', box_line);
fprintf('   Speeds: %s %.0f RPM | %s %.0f RPM | %s %.0f RPM      z = along the shaft\n\n', ...
    rotor_names{1}, max_speed_rpm(1), rotor_names{2}, max_speed_rpm(2), rotor_names{3}, max_speed_rpm(3));
fprintf('   %-26s %12s %12s %12s %14s\n', '', rotor_names{1}, rotor_names{2}, rotor_names{3}, 'TOTAL');
for k = 1:6
    row = answer_rows{k};
    fprintf('   %-26s %12.4g %12.4g %12.4g %14.4g\n', answer_labels{k}, row);
end
fprintf('\n   Fx = Fy and Mx = My because the loads rotate: same peak, a quarter turn apart.\n');
fprintf('   Fz comes only from the bearings. Mz only acts during spin-up / spin-down;\n');
fprintf('   its TOTAL is small because the wheels cancel the arm.\n');

fprintf('\n   WHERE IT COMES FROM (TOTAL)\n');
fprintf('        static imbalance   -> Fx, Fy   %12.4g  N\n',   sum(static_imbalance_force_N));
fprintf('        bearing harmonics  -> Fx, Fy   %12.4g  N\n',   sum(bearing_lateral_N));
fprintf('        bearing harmonics  -> Fz       %12.4g  N\n',   sum(bearing_axial_N));
fprintf('        dynamic imbalance  -> Mx, My   %12.4g  N*m\n', sum(dynamic_imbalance_moment_Nm));
fprintf('        force x distance   -> Mx, My   %12.4g  N*m\n', sum(lever_arm_m .* rotor_lateral_N));

if ~isempty(report_arm_rpm)
    fprintf('\n   AT OTHER SPEEDS  (TOTAL, wheels follow the arm; every value grows with speed squared)\n');
    fprintf('   %10s %12s %14s %14s %16s\n', 'Arm RPM', 'Wheel RPM', 'Fx = Fy [N]', 'Fz [N]', 'Mx = My [N*m]');
    for i = 1:numel(report_arm_rpm)
        speed_ratio = report_arm_rpm(i) / max_speed_rpm(arm_index);
        fprintf('   %10.1f %12.1f %14.4g %14.4g %16.4g\n', report_arm_rpm(i), max_speed_rpm(1) * speed_ratio, ...
            total_lateral_N * speed_ratio ^ 2, total_axial_N * speed_ratio ^ 2, total_moment_Nm * speed_ratio ^ 2);
    end
end
fprintf('\n%s\n', box_line);
