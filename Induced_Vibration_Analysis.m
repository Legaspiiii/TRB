%% ========================================================================
%  INDUCED VIBRATION ANALYSIS
%  Forces and moments induced by two momentum wheels, one rotating arm,
%  and their bearings
%
%  Sources (each one has its equations here and its inputs in SECTION 1):
%    1. STATIC IMBALANCE   Us -> rotating lateral force  F = Us x sqrt(speed^4 + accel^2)
%                          (radial Us x speed^2, plus tangential Us x accel during ramps)
%    2. COUPLE IMBALANCE   Ud -> rotating rocking moment M = Ud x sqrt(speed^4 + accel^2)
%                          (often called dynamic imbalance)
%       Us from: Us directly, mass x eccentricity, or ISO 21940 grade (Us = mass x G / speed)
%       Ud from: Ud directly, or static tolerance in two planes b apart, opposed: Ud = (Us/2) x b
%    3. MOTOR REACTION TORQUE during spin-up / spin-down: Mz = -Izz x accel
%    4. TAPERED ROLLER BEARINGS (bearing A and B of each rotor), from geometry, preload,
%       stiffness and specified waviness:
%       - Q0 = preload / (Z sin(angle)); Palmgren line contact -> roller stiffness k
%       - bearing stiffness: radial (Z/2) k cos^2(angle), axial Z k sin^2(angle)
%       - waviness (out-of-roundness) of races and rollers: orders q = kZ +/- 1 give
%         lateral force, q = kZ axial force (races); even orders q = 2k for rollers
%       - force at the shaft = blocked force x transmissibility of the rotor on its bearings
%    5. MOTOR TORQUE RIPPLE AND COGGING (when motor data is entered):
%       Mz on the mount = ripple + cogging (datasheet peak values, worst case)
%    6. GYROSCOPIC MOMENT from motion of the supporting spacecraft:
%       M = -(spacecraft rate) x (total stored momentum H), H = sum(direction x Izz x speed)
%
%  Assumptions (state these in your report):
%    - Hard-mounted: loads are calculated as if bolted to a rigid base.
%    - WORST CASE: all peaks are added as if they line up at the same
%      moment (phases are unknown). Real loads are equal or lower.
%    - Fx and Fy (and Mx and My) are peak values reached a quarter turn
%      apart, NOT at the same time: the sideways resultant = Fx, not 1.41 x Fx.
%    - If Ud is not entered, it is the static tolerance in two planes, opposed
%      (worst case): Ud = (Us / 2) x plane spacing.
%    - Bearing loads come from the SPECIFIED waviness; they are
%      only as good as those numbers (use drawing / bearing class tolerances).
%    - Bearing model is linear about the preload (rollers stay loaded) and uses
%      the rotor's lateral and axial modes only (no rocking mode).
%    - Imbalance values are the SPECIFIED limits (balance spec), not measurements.
%    - Not included: structural resonance, broadband noise, rotor flexibility,
%      localized bearing defects, imbalance harmonics above 1x RPM, bearing
%      varying-compliance excitation (small: bearings are preloaded and the
%      sideways loads are small compared with the preload).
%
%  Directions:
%    lateral = sideways (perpendicular to the shaft)
%    axial   = along the shaft (spin axis)
%
%  How to use:
%    1. Edit SECTION 1 only (your numbers).
%    2. Run the script.
%    3. The Command Window prints Fx, Fy, Fz, Mx, My, Mz in five parts:
%         PART 1  ARM               (imbalance + ramp twist)
%         PART 2  MOMENTUM WHEELS   (imbalance + ramp twist)
%         PART 3  BEARINGS
%         PART 4  MOTORS (ripple, cogging) AND GYROSCOPIC
%         PART 5  ALL TOGETHER      <- the final answer, at the very bottom
%       The final answer is also saved as final_Fx_N ... final_Mz_Nm.
%    4. ARITHMETIC CHECKS print first: every line should say PASS.
%       (They prove the code does its math right, not that the inputs
%        or estimates are physically right.)
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
spacecraft_rate_rad_s  = [0 0 0];  % [rad/s] angular rate of the supporting spacecraft about x, y, z
                                  %         (gyroscopic moment). [0 0 0] = fixed base
report_arm_rpm       = [50 200];  % [RPM] extra arm speeds to get results at (wheels follow at the same
                                  %       ratio as the max speeds). Use [] for none.

% ------------------------------------------------------------------------
% IMBALANCE INPUTS -- fill in ONE way for each (the first one filled in is used)
%   STATIC imbalance Us (lateral force):
%     way 1  *_static_imbalance_kgm   Us directly, from a balance spec/report
%     way 2  *_eccentricity_um        center of mass offset   -> Us = mass x e
%     way 3  *_balance_grade_mm_s     ISO 21940 grade G        -> Us = mass x G / speed
%   COUPLE imbalance Ud (rocking moment, often called dynamic imbalance):
%     way 1  *_couple_imbalance_kgm2  Ud directly, from a balance spec/report
%     way 2  *_plane_spacing_m        distance b between the 2 balancing planes
%                                     -> Ud = (Us / 2) x b  (static tolerance in each
%                                        plane, opposed = worst case)
%   Unit help:  1 g*mm = 1e-6 kg*m      1 g*mm^2 = 1e-9 kg*m^2
%   Leave the ways you don't use as [].
%
% SPIN INERTIA: enter Izz (from CAD), OR the stored momentum H (Izz = H / speed).
%
% MOTOR DATA (leave [] until known; the output then says "not included"):
%   *_motor_ripple_Nm  : torque ripple, peak [N*m]   (motor datasheet)
%   *_motor_cogging_Nm : cogging torque, peak [N*m]  (motor datasheet)
%
% Anything required that is left [] stops the script with a list of what to fill in.
% ------------------------------------------------------------------------

% ------------------------------------------------------------------------
% BEARING INPUTS (each rotor has a back-to-back PAIR: bearing A and bearing B)
%   Geometry (catalog): rollers, roller diameter, pitch diameter, cup angle,
%                       effective roller contact length
%   Preload           : axial preload on the pair [N] (from assembly spec)
%   Waviness          : how out-of-round the bearing surfaces are [um], one value for the pair
%                       (small bumps on the races and rollers - this is what makes a bearing
%                       vibrate). Ask the bearing supplier. Until then 0.1 um is used: the upper
%                       end of the range typical for precision bearings (0.01 - 0.1 um).
% ------------------------------------------------------------------------

% ------------------------------------------------------------------------
% 1B. WHEEL 1
% ------------------------------------------------------------------------
wheel1_name                   = 'Wheel 1';
wheel1_mass_kg                = 3.0;      % [kg]     rotating mass only
wheel1_Izz_kgm2               = 0.034;    % [kg*m^2] spin-axis inertia, from CAD (or use momentum below)
wheel1_momentum_Nms           = [];       % [N*m*s]  OPTIONAL instead of Izz: stored momentum at max speed (Izz = H / speed)
wheel1_static_imbalance_kgm   = [];       % [kg*m]   STATIC way 1: Us directly (balance spec / report)
wheel1_eccentricity_um        = [];       % [um]     STATIC way 2: center of mass offset (Us = mass x e)
wheel1_balance_grade_mm_s     = 2.5;      % [mm/s]   STATIC way 3: ISO 21940 balance grade G (Us = mass x G / speed)
wheel1_couple_imbalance_kgm2  = [];       % [kg*m^2] COUPLE way 1: Ud directly (balance spec / report)
wheel1_plane_spacing_m        = 0.030;    % [m]      COUPLE way 2: distance between the 2 balancing planes (Ud = Us/2 x b)
wheel1_max_speed_rpm          = 515;      % [RPM]
wheel1_spin_direction         = -1;       % +1 or -1
wheel1_position_m             = -0.200;   % [m]      center of mass position along the shaft
wheel1_motor_ripple_Nm        = [];       % [N*m]    torque ripple, peak         (motor datasheet)
wheel1_motor_cogging_Nm       = [];       % [N*m]    cogging torque, peak        (motor datasheet)

wheel1_bearing_rollers        = 15;   % [-]   number of rollers      (bearing catalog)
wheel1_bearing_roller_dia_mm  = 7;    % [mm]  mean roller diameter   (bearing catalog)
wheel1_bearing_pitch_dia_mm   = 39;   % [mm]  pitch diameter         (bearing catalog)
wheel1_bearing_cup_angle_deg  = 14;   % [deg] cup (contact) angle    (bearing catalog)
wheel1_bearing_roller_length_mm   = 8 ;    % [mm]  effective roller contact length (catalog / drawing)
wheel1_bearing_preload_N          = 500;  % [N]   axial preload on the bearing pair
wheel1_bearing_waviness_um        = 0.1;   % [um] waviness (out-of-roundness), assumed until supplier data

% ------------------------------------------------------------------------
% 1C. ARM
% ------------------------------------------------------------------------
arm_name                      = 'Arm';
arm_mass_kg                   = 4.0;      % [kg]     rotating mass only
arm_Izz_kgm2                  = 0.350;    % [kg*m^2] spin-axis inertia, from CAD (or use momentum below)
arm_momentum_Nms              = [];       % [N*m*s]  OPTIONAL instead of Izz: stored momentum at max speed (Izz = H / speed)
arm_static_imbalance_kgm      = [];       % [kg*m]   STATIC way 1: Us directly (balance spec / report)
arm_eccentricity_um           = 20;       % [um]     STATIC way 2: center of mass offset (Us = mass x e)
arm_balance_grade_mm_s        = [];       % [mm/s]   STATIC way 3: ISO 21940 balance grade G (Us = mass x G / speed)
arm_couple_imbalance_kgm2     = [];       % [kg*m^2] COUPLE way 1: Ud directly (balance spec / report)
arm_plane_spacing_m           = 0.040;    % [m]      COUPLE way 2: distance between the 2 balancing planes (Ud = Us/2 x b)
arm_max_speed_rpm             = 100;      % [RPM]
arm_spin_direction            = +1;       % +1 or -1
arm_position_m                = 0.000;    % [m]      center of mass position along the shaft
arm_motor_ripple_Nm           = [];       % [N*m]    torque ripple, peak         (motor datasheet)
arm_motor_cogging_Nm          = [];       % [N*m]    cogging torque, peak        (motor datasheet)

arm_bearing_rollers           = 20;   % [-]   number of rollers      (bearing catalog)
arm_bearing_roller_dia_mm     = 8;    % [mm]  mean roller diameter   (bearing catalog)
arm_bearing_pitch_dia_mm      = 55;   % [mm]  pitch diameter         (bearing catalog)
arm_bearing_cup_angle_deg     = 15;   % [deg] cup (contact) angle    (bearing catalog)
arm_bearing_roller_length_mm   = 10;    % [mm]  effective roller contact length (catalog / drawing)
arm_bearing_preload_N          = 900;  % [N]   axial preload on the bearing pair
arm_bearing_waviness_um        = 0.1;   % [um] waviness (out-of-roundness), assumed until supplier data

% ------------------------------------------------------------------------
% 1D. WHEEL 2
% ------------------------------------------------------------------------
wheel2_name                   = 'Wheel 2';
wheel2_mass_kg                = 3.0;      % [kg]     rotating mass only
wheel2_Izz_kgm2               = 0.034;    % [kg*m^2] spin-axis inertia, from CAD (or use momentum below)
wheel2_momentum_Nms           = [];       % [N*m*s]  OPTIONAL instead of Izz: stored momentum at max speed (Izz = H / speed)
wheel2_static_imbalance_kgm   = [];       % [kg*m]   STATIC way 1: Us directly (balance spec / report)
wheel2_eccentricity_um        = [];       % [um]     STATIC way 2: center of mass offset (Us = mass x e)
wheel2_balance_grade_mm_s     = 2.5;      % [mm/s]   STATIC way 3: ISO 21940 balance grade G (Us = mass x G / speed)
wheel2_couple_imbalance_kgm2  = [];       % [kg*m^2] COUPLE way 1: Ud directly (balance spec / report)
wheel2_plane_spacing_m        = 0.030;    % [m]      COUPLE way 2: distance between the 2 balancing planes (Ud = Us/2 x b)
wheel2_max_speed_rpm          = 515;      % [RPM]
wheel2_spin_direction         = -1;       % +1 or -1
wheel2_position_m             = 0.200;    % [m]      center of mass position along the shaft
wheel2_motor_ripple_Nm        = [];       % [N*m]    torque ripple, peak         (motor datasheet)
wheel2_motor_cogging_Nm       = [];       % [N*m]    cogging torque, peak        (motor datasheet)

wheel2_bearing_rollers        = 15;   % [-]   number of rollers      (bearing catalog)
wheel2_bearing_roller_dia_mm  = 7;    % [mm]  mean roller diameter   (bearing catalog)
wheel2_bearing_pitch_dia_mm   = 39;   % [mm]  pitch diameter         (bearing catalog)
wheel2_bearing_cup_angle_deg  = 14;   % [deg] cup (contact) angle    (bearing catalog)
wheel2_bearing_roller_length_mm   = 8 ;    % [mm]  effective roller contact length (catalog / drawing)
wheel2_bearing_preload_N          = 500;  % [N]   axial preload on the bearing pair
wheel2_bearing_waviness_um        = 0.1;   % [um] waviness (out-of-roundness), assumed until supplier data

% ------------------------------------------------------------------------
% 1E. PLOTS
% ------------------------------------------------------------------------
make_plots  = true;    % true = draw the 2 plots
save_plots  = false;   % true = also save them as PNG files
plot_folder = 'Induced_Vibration_Plots';


%% ========================================================================
%  SECTION 1F: INPUT CHECK  (stops with a list if anything required is missing)
%  ========================================================================

rotor_prefixes = {'wheel1', 'arm', 'wheel2'};
required_each  = {'mass_kg', 'max_speed_rpm', 'spin_direction', 'position_m', ...
                  'bearing_rollers', 'bearing_roller_dia_mm', 'bearing_pitch_dia_mm', 'bearing_cup_angle_deg', ...
                  'bearing_roller_length_mm', 'bearing_preload_N', 'bearing_waviness_um'};
required_general = {'spin_up_time_s', 'spin_down_time_s', 'reference_position_m', 'spacecraft_rate_rad_s'};
missing_inputs = {};
for i = 1:numel(required_general)
    if isempty(eval(required_general{i}))
        missing_inputs{end+1} = required_general{i};
    end
end
for p = 1:numel(rotor_prefixes)
    pre = rotor_prefixes{p};
    for i = 1:numel(required_each)
        if isempty(eval([pre, '_', required_each{i}]))
            missing_inputs{end+1} = [pre, '_', required_each{i}];
        end
    end
    if isempty(eval([pre, '_Izz_kgm2'])) && isempty(eval([pre, '_momentum_Nms']))
        missing_inputs{end+1} = [pre, '_Izz_kgm2  (or ', pre, '_momentum_Nms)'];
    end
    if isempty(eval([pre, '_static_imbalance_kgm'])) && isempty(eval([pre, '_eccentricity_um'])) ...
            && isempty(eval([pre, '_balance_grade_mm_s']))
        missing_inputs{end+1} = [pre, ' static imbalance: fill ONE of _static_imbalance_kgm, _eccentricity_um, _balance_grade_mm_s'];
    end
    if isempty(eval([pre, '_couple_imbalance_kgm2'])) && isempty(eval([pre, '_plane_spacing_m']))
        missing_inputs{end+1} = [pre, ' couple imbalance: fill ONE of _couple_imbalance_kgm2, _plane_spacing_m'];
    end
end
if numel(spacecraft_rate_rad_s) ~= 3
    missing_inputs{end+1} = 'spacecraft_rate_rad_s must have 3 values [x y z]';
end
if ~isempty(missing_inputs)
    fprintf('\n*** MISSING INPUTS - fill these in SECTION 1, then run again ***\n');
    fprintf('   %s\n', missing_inputs{:});
    error('Missing inputs (listed above).');
end


%% ========================================================================
%  SECTION 2: PUT THE THREE ROTORS SIDE BY SIDE
%  Every quantity is a row of 3 numbers: [Wheel 1, Arm, Wheel 2]
%  ========================================================================

rotor_names     = {wheel1_name, arm_name, wheel2_name};
number_of_rotors = 3;
arm_index                     = 2;

mass_kg          = [wheel1_mass_kg,         arm_mass_kg,         wheel2_mass_kg];
max_speed_rpm    = [wheel1_max_speed_rpm,   arm_max_speed_rpm,   wheel2_max_speed_rpm];
spin_direction   = [wheel1_spin_direction,  arm_spin_direction,  wheel2_spin_direction];
position_m       = [wheel1_position_m,      arm_position_m,      wheel2_position_m];

% Inputs that can be filled in more than one way are kept as cells ([] = not given)
Izz_entered               = {wheel1_Izz_kgm2,               arm_Izz_kgm2,               wheel2_Izz_kgm2};
momentum_entered          = {wheel1_momentum_Nms,           arm_momentum_Nms,           wheel2_momentum_Nms};
static_imbalance_entered  = {wheel1_static_imbalance_kgm,   arm_static_imbalance_kgm,   wheel2_static_imbalance_kgm};
eccentricity_entered      = {wheel1_eccentricity_um,        arm_eccentricity_um,        wheel2_eccentricity_um};
balance_grade_entered     = {wheel1_balance_grade_mm_s,     arm_balance_grade_mm_s,     wheel2_balance_grade_mm_s};
couple_imbalance_entered  = {wheel1_couple_imbalance_kgm2,  arm_couple_imbalance_kgm2,  wheel2_couple_imbalance_kgm2};
plane_spacing_entered     = {wheel1_plane_spacing_m,        arm_plane_spacing_m,        wheel2_plane_spacing_m};
motor_ripple_entered      = {wheel1_motor_ripple_Nm,        arm_motor_ripple_Nm,        wheel2_motor_ripple_Nm};
motor_cogging_entered     = {wheel1_motor_cogging_Nm,       arm_motor_cogging_Nm,       wheel2_motor_cogging_Nm};

bearing_rollers       = [wheel1_bearing_rollers,       arm_bearing_rollers,       wheel2_bearing_rollers];
bearing_roller_dia_mm = [wheel1_bearing_roller_dia_mm, arm_bearing_roller_dia_mm, wheel2_bearing_roller_dia_mm];
bearing_pitch_dia_mm  = [wheel1_bearing_pitch_dia_mm,  arm_bearing_pitch_dia_mm,  wheel2_bearing_pitch_dia_mm];
bearing_cup_angle_deg = [wheel1_bearing_cup_angle_deg, arm_bearing_cup_angle_deg, wheel2_bearing_cup_angle_deg];
bearing_roller_length_mm = [wheel1_bearing_roller_length_mm, arm_bearing_roller_length_mm, wheel2_bearing_roller_length_mm];
bearing_preload_N        = [wheel1_bearing_preload_N,        arm_bearing_preload_N,        wheel2_bearing_preload_N];

% Waviness: rows = [Wheel 1; Arm; Wheel 2], columns = [bearing A, bearing B]   [m]
% (one value per pair, used for both bearings and for the outer race, inner race and rollers)
waviness_m = 1e-6 * [wheel1_bearing_waviness_um; arm_bearing_waviness_um; wheel2_bearing_waviness_um] * [1 1];


%% ========================================================================
%  SECTION 3: UNIT CONVERSIONS
%  ========================================================================

speed_rad_s      = max_speed_rpm * 2 * pi / 60;     % [rad/s]
speed_hz         = max_speed_rpm / 60;              % [Hz]  revolutions per second (1x RPM)
lever_arm_m      = abs(position_m - reference_position_m);   % [m] distance to the reference point
ramp_time_s      = max(min(spin_up_time_s, spin_down_time_s), eps);   % shorter ramp = worst case (used in tables)
spin_accel_rad_s2 = speed_rad_s / ramp_time_s;      % [rad/s^2] angular acceleration of the worst ramp

% Spin inertia: Izz from CAD, or from stored momentum H:  Izz = H / speed
Izz_kgm2       = zeros(1, number_of_rotors);
inertia_source = cell(1, number_of_rotors);
for r = 1:number_of_rotors
    if ~isempty(Izz_entered{r})
        Izz_kgm2(r) = Izz_entered{r};
        inertia_source{r} = 'CAD';
    else
        Izz_kgm2(r) = momentum_entered{r} / speed_rad_s(r);
        inertia_source{r} = 'from H';
    end
end

% Twist on the mount (reaction torque = -Izz x acceleration), spin-up and spin-down separately.
% Sign shows direction: spin-up and spin-down twist the mount opposite ways.
twist_spin_up_Nm   = -spin_direction .* Izz_kgm2 .* speed_rad_s / max(spin_up_time_s, eps);
twist_spin_down_Nm =  spin_direction .* Izz_kgm2 .* speed_rad_s / max(spin_down_time_s, eps);


%% ========================================================================
%  SECTION 4: IMBALANCE, MOTOR RIPPLE/COGGING, GYROSCOPIC
%  ========================================================================

% Static imbalance Us and couple imbalance Ud: the first way filled in is used
static_imbalance_kgm   = zeros(1, number_of_rotors);
dynamic_imbalance_kgm2 = zeros(1, number_of_rotors);     % couple imbalance Ud
static_source  = cell(1, number_of_rotors);
dynamic_source = cell(1, number_of_rotors);
for r = 1:number_of_rotors
    if ~isempty(static_imbalance_entered{r})
        static_imbalance_kgm(r) = static_imbalance_entered{r};
        static_source{r} = 'entered';
    elseif ~isempty(eccentricity_entered{r})
        static_imbalance_kgm(r) = mass_kg(r) * eccentricity_entered{r} * 1e-6;                  % Us = m e
        static_source{r} = sprintf('e = %g um', eccentricity_entered{r});
    else
        static_imbalance_kgm(r) = mass_kg(r) * balance_grade_entered{r} * 1e-3 / speed_rad_s(r);  % Us = m G / speed
        static_source{r} = sprintf('grade G%g', balance_grade_entered{r});
    end
    if ~isempty(couple_imbalance_entered{r})
        dynamic_imbalance_kgm2(r) = couple_imbalance_entered{r};
        dynamic_source{r} = 'entered';
    else
        dynamic_imbalance_kgm2(r) = static_imbalance_kgm(r) / 2 * plane_spacing_entered{r};      % Ud = (Us/2) b
        dynamic_source{r} = sprintf('b = %g m', plane_spacing_entered{r});
    end
end

% ---- 4A. Static and couple imbalance (1x RPM) ----
% At constant speed: radial force Us x speed^2 and moment Ud x speed^2.
% While the speed changes there is also a tangential part Us x accel (Ud x accel)
% at right angles, so the peak at the top of a ramp is Us x sqrt(speed^4 + accel^2).
static_force_steady_N       = static_imbalance_kgm   .* speed_rad_s .^ 2;
dynamic_moment_steady_Nm    = dynamic_imbalance_kgm2 .* speed_rad_s .^ 2;
static_imbalance_force_N    = static_imbalance_kgm   .* sqrt(speed_rad_s .^ 4 + spin_accel_rad_s2 .^ 2);
dynamic_imbalance_moment_Nm = dynamic_imbalance_kgm2 .* sqrt(speed_rad_s .^ 4 + spin_accel_rad_s2 .^ 2);

% ---- 4B. Motor torque ripple and cogging (when motor data is entered) ----
% The stator reaction passes straight into the structure, so ripple and cogging are a
% twist Mz on the mount. Both are added (worst case, no speed-loop attenuation).
motor_torque_Nm = zeros(1, number_of_rotors);
for r = 1:number_of_rotors
    motor_torque_Nm(r) = sum([motor_ripple_entered{r}, 0]) + sum([motor_cogging_entered{r}, 0]);
end

% ---- 4C. Gyroscopic moment from motion of the supporting spacecraft ----
% Stored momentum of each rotor along z (sign = spin direction), moment M = -(rate x H)
momentum_Nms = spin_direction .* Izz_kgm2 .* speed_rad_s;
spacecraft_rate = reshape(spacecraft_rate_rad_s, 1, 3);
gyro_moment_Nm = zeros(3, number_of_rotors);                 % rows = [Mx; My; Mz]
for r = 1:number_of_rotors
    gyro_moment_Nm(:, r) = -cross(spacecraft_rate, [0 0 momentum_Nms(r)])';
end


%% ========================================================================
%  SECTION 5: BEARING DISTURBANCE MODEL
%  ------------------------------------------------------------------------
%  5A. Characteristic frequencies (pure geometry)
%      Cup (outer race) turns with the rotor at fr, cone (inner race) is on
%      the stationary shaft. Harmonic number h = events per revolution.
%  5B. Stiffness from geometry and preload
%  5C. Disturbance lines from the specified waviness
%  5D. Force that reaches the stationary shaft (rotor-on-bearing dynamics)
%  ========================================================================

% Fixed model settings
bearing_harmonic_count = 2;      % harmonics k = 1, 2 of each bearing disturbance (they carry most of the energy)
bearing_damping_ratio  = 0.03;   % damping of the rotor-on-bearing modes (typical; matters only near resonance)

% ---- 5A. Characteristic frequencies ----
harmonic_names   = {'Cage', 'Outer race pass', 'Inner race pass', 'Roller spin'};
number_of_harmonics = 4;
bearing_harmonic = zeros(number_of_rotors, number_of_harmonics);   % rows = rotors

for r = 1:number_of_rotors
    g = bearing_roller_dia_mm(r) * cosd(bearing_cup_angle_deg(r)) / bearing_pitch_dia_mm(r);
    bearing_harmonic(r, 1) = (1 + g) / 2;                                                   % cage
    bearing_harmonic(r, 2) = bearing_rollers(r) * (1 - g) / 2;                              % outer race (cup) pass
    bearing_harmonic(r, 3) = bearing_rollers(r) * (1 + g) / 2;                              % inner race (cone) pass
    bearing_harmonic(r, 4) = bearing_pitch_dia_mm(r) / (2 * bearing_roller_dia_mm(r)) * (1 - g ^ 2);  % roller spin
end
bearing_frequency_hz = bearing_harmonic .* (speed_hz');            % [Hz] rows = rotors

% ---- 5B. Stiffness from geometry and preload ----
cup_angle_rad    = bearing_cup_angle_deg * pi / 180;
roller_load_N    = bearing_preload_N ./ (bearing_rollers .* sin(cup_angle_rad));                 % Q0, load on each roller
contact_defl_m   = 3.84e-5 * roller_load_N .^ 0.9 ./ bearing_roller_length_mm .^ 0.8 * 1e-3;      % Palmgren, one line contact [m]
roller_defl_m    = 2 * contact_defl_m;                                                            % inner + outer contact
roller_stiffness_Npm = (10 / 9) * roller_load_N ./ roller_defl_m;                                 % k = dQ/d(deflection)
bearing_radial_stiffness_Npm = bearing_rollers / 2 .* roller_stiffness_Npm .* cos(cup_angle_rad) .^ 2;   % one bearing
bearing_axial_stiffness_Npm  = bearing_rollers     .* roller_stiffness_Npm .* sin(cup_angle_rad) .^ 2;   % one bearing
natural_freq_radial_hz = sqrt(2 * bearing_radial_stiffness_Npm ./ mass_kg) / (2 * pi);   % rotor on its 2 bearings, lateral
natural_freq_axial_hz  = sqrt(2 * bearing_axial_stiffness_Npm  ./ mass_kg) / (2 * pi);   % rotor on its 2 bearings, axial

% ---- 5C. Disturbance lines from the specified waviness ----
%  Each line: rotor, source, waviness order, direction (1 = lateral, 3 = axial),
%  frequency at max speed [Hz], and blocked force from bearing A and bearing B [N].
%  Race waviness of order q, summed over the Z rollers, gives a net force only when
%    lateral: q = kZ +/- 1  -> magnitude (Z/2) k A cos(angle) per order
%    axial  : q = kZ        -> magnitude  Z    k A sin(angle)
%  Inner race (fixed)  : lateral and axial lines at k x (inner race pass)
%  Outer race (turning): lateral at k x (outer race pass) +/- fr, axial at k x (outer race pass)
%  Roller waviness, even order q = 2k (changes the roller diameter by 2A):
%    lateral at 2k x (roller spin) +/- cage, axial at 2k x (roller spin)
%  All rollers' waviness is taken in phase (worst case, x Z).
line_rotor = [];  line_source = {};  line_order = {};  line_dir = [];  line_freq_hz = [];  line_blocked_N = zeros(0, 2);

for r = 1:number_of_rotors
    Z  = bearing_rollers(r);
    kr = roller_stiffness_Npm(r);
    ca = cos(cup_angle_rad(r));
    sa = sin(cup_angle_rad(r));
    fr = speed_hz(r);
    fc = bearing_frequency_hz(r, 1);
    fo = bearing_frequency_hz(r, 2);
    fi = bearing_frequency_hz(r, 3);
    fb = bearing_frequency_hz(r, 4);
    roller_count_factor = Z;               % all rollers in phase (worst case)
    A_out = waviness_m(r, :);
    A_in  = waviness_m(r, :);
    A_rol = waviness_m(r, :);

    for k = 1:bearing_harmonic_count
        % inner race (fixed): orders kZ-1 and kZ+1 land on the same frequency, so they add
        line_rotor(end+1) = r; line_source{end+1} = 'Inner race waviness'; line_order{end+1} = sprintf('%dZ-1, %dZ+1', k, k);
        line_dir(end+1) = 1;   line_freq_hz(end+1) = k * fi;               line_blocked_N(end+1, :) = 2 * (Z / 2) * kr * ca * A_in;
        line_rotor(end+1) = r; line_source{end+1} = 'Inner race waviness'; line_order{end+1} = sprintf('%dZ', k);
        line_dir(end+1) = 3;   line_freq_hz(end+1) = k * fi;               line_blocked_N(end+1, :) = Z * kr * sa * A_in;
        % outer race (turning with the rotor)
        line_rotor(end+1) = r; line_source{end+1} = 'Outer race waviness'; line_order{end+1} = sprintf('%dZ+1', k);
        line_dir(end+1) = 1;   line_freq_hz(end+1) = k * fo + fr;          line_blocked_N(end+1, :) = (Z / 2) * kr * ca * A_out;
        line_rotor(end+1) = r; line_source{end+1} = 'Outer race waviness'; line_order{end+1} = sprintf('%dZ-1', k);
        line_dir(end+1) = 1;   line_freq_hz(end+1) = abs(k * fo - fr);     line_blocked_N(end+1, :) = (Z / 2) * kr * ca * A_out;
        line_rotor(end+1) = r; line_source{end+1} = 'Outer race waviness'; line_order{end+1} = sprintf('%dZ', k);
        line_dir(end+1) = 3;   line_freq_hz(end+1) = k * fo;               line_blocked_N(end+1, :) = Z * kr * sa * A_out;
        % rollers (even waviness order 2k)
        line_rotor(end+1) = r; line_source{end+1} = 'Roller waviness';     line_order{end+1} = sprintf('%d', 2 * k);
        line_dir(end+1) = 1;   line_freq_hz(end+1) = 2 * k * fb + fc;      line_blocked_N(end+1, :) = roller_count_factor * kr * ca * A_rol;
        line_rotor(end+1) = r; line_source{end+1} = 'Roller waviness';     line_order{end+1} = sprintf('%d', 2 * k);
        line_dir(end+1) = 1;   line_freq_hz(end+1) = abs(2 * k * fb - fc); line_blocked_N(end+1, :) = roller_count_factor * kr * ca * A_rol;
        line_rotor(end+1) = r; line_source{end+1} = 'Roller waviness';     line_order{end+1} = sprintf('%d', 2 * k);
        line_dir(end+1) = 3;   line_freq_hz(end+1) = 2 * k * fb;           line_blocked_N(end+1, :) = roller_count_factor * 2 * kr * sa * A_rol;
    end
end
line_blocked_total_N = sum(line_blocked_N, 2)';          % bearing A + bearing B (worst case: in phase)

% Linear model check: waviness must be small next to the preload deflection
largest_imposed_m = max(4 * waviness_m, [], 2)';        % outer + inner race + roller (x2, both contacts)
linearity_ratio   = largest_imposed_m ./ roller_defl_m;   % keep well below 1 (rollers stay loaded)

% ---- 5D. Force that reaches the stationary shaft ----
%  The waviness pushes the rotor on its bearings. Below the rotor-bearing
%  natural frequency the rotor follows and only its inertia force reaches the
%  shaft; far above it the full blocked force does. Transmissibility:
transmissibility = @(ratio) ratio .^ 2 ./ sqrt((1 - ratio .^ 2) .^ 2 + (2 * bearing_damping_ratio * ratio) .^ 2);
line_natural_hz  = natural_freq_radial_hz(line_rotor);
is_axial_line    = (line_dir == 3);
line_natural_hz(is_axial_line) = natural_freq_axial_hz(line_rotor(is_axial_line));
% Transmitted force of every line at a speed factor s (1 = max speed): frequencies scale with s
bearing_line_force = @(speed_factor) line_blocked_total_N .* transmissibility(speed_factor * line_freq_hz ./ line_natural_hz);
line_force_N = bearing_line_force(1);

% Totals per rotor (worst case: all lines added)
bearing_lateral_N = zeros(1, number_of_rotors);
bearing_axial_N   = zeros(1, number_of_rotors);
for r = 1:number_of_rotors
    bearing_lateral_N(r) = sum(line_force_N(line_rotor == r & line_dir == 1));
    bearing_axial_N(r)   = sum(line_force_N(line_rotor == r & line_dir == 3));
end


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
%  Mz     : twist about the shaft (motor reaction during ramps, plus motor ripple/cogging).
%  Gyroscopic moments are not rotating: they are kept with their signs and added at the end.
%
%  Each source is stored as 6 numbers: [Fx; Fy; Fz; Mx; My; Mz]
%  Worst case = every peak lines up at the same moment (safe design value).
%  ========================================================================

% ---- Imbalance of each rotor (wheels and arm) ----
imbalance_loads = zeros(6, number_of_rotors);          % columns = Wheel 1, Arm, Wheel 2
for r = 1:number_of_rotors
    imbalance_force  = static_imbalance_force_N(r);
    imbalance_moment = dynamic_imbalance_moment_Nm(r) + lever_arm_m(r) * imbalance_force;
    imbalance_loads(:, r) = [imbalance_force;                 % Fx
                             imbalance_force;                 % Fy
                             0;                               % Fz (imbalance does not push along the shaft)
                             imbalance_moment;                % Mx
                             imbalance_moment;                % My
                             spin_direction(r) * Izz_kgm2(r) * speed_rad_s(r) / ramp_time_s];  % Mz (signed twist)
end

% Steady values (constant speed, no ramp) for the plots and the loads at other speeds
steady_force_N   = static_force_steady_N;
steady_moment_Nm = dynamic_moment_steady_Nm + lever_arm_m .* steady_force_N;

% ---- Motor ripple / cogging and gyroscopic moment of each rotor ----
motor_loads = zeros(6, number_of_rotors);
motor_loads(6, :) = motor_torque_Nm;
gyro_loads = zeros(6, number_of_rotors);
gyro_loads(4:6, :) = gyro_moment_Nm;

% ---- Bearings of each rotor (every bearing line, bearing A + B) ----
bearing_loads = zeros(6, number_of_rotors);            % columns = Wheel 1 bearings, Arm bearings, Wheel 2 bearings
for r = 1:number_of_rotors
    bearing_moment = lever_arm_m(r) * bearing_lateral_N(r);
    bearing_loads(:, r) = [bearing_lateral_N(r);    % Fx
                           bearing_lateral_N(r);    % Fy
                           bearing_axial_N(r);      % Fz
                           bearing_moment;          % Mx
                           bearing_moment;          % My
                           0];                      % Mz (bearing drag is internal: motor and bearings cancel)
    % Note: the bearing force reaches the shaft as the rotor's reaction, so it acts at the rotor center of mass.
end

% ---- Group the columns ----
wheel_columns = [1 3];
arm_loads                     = imbalance_loads(:, arm_index);
wheel_loads   = imbalance_loads(:, wheel_columns);
wheels_total  = sum(wheel_loads, 2);                   % Mz adds with sign (both wheels spin the same way)
bearings_total = sum(bearing_loads, 2);
motors_total  = sum(motor_loads, 2);
gyro_total    = abs(sum(gyro_loads, 2));               % net gyroscopic moment (signed sum, then size)
all_total     = arm_loads + wheels_total + bearings_total;
all_total(6)  = abs(all_total(6)) + motors_total(6);   % net twist (wheels cancel the arm) + motor ripple/cogging
all_total(4:5) = all_total(4:5) + gyro_total(4:5);     % gyroscopic moment added at full value

% Show twist as a size (positive) everywhere except where noted
arm_loads(6)     = abs(arm_loads(6));
wheel_loads(6,:) = abs(wheel_loads(6,:));
wheels_total(6)  = abs(wheels_total(6));

% ---- Final answer saved in the Workspace: [Fx Fy Fz Mx My Mz] ----
final_arm_loads      = arm_loads';
final_wheel_1_loads  = wheel_loads(:, 1)';
final_wheel_2_loads  = wheel_loads(:, 2)';
final_bearing_loads  = bearings_total';
final_motor_loads    = motors_total';
final_gyro_loads     = gyro_total';
final_Fx_N  = all_total(1);
final_Fy_N  = all_total(2);
final_Fz_N  = all_total(3);
final_Mx_Nm = all_total(4);
final_My_Nm = all_total(5);
final_Mz_Nm = all_total(6);


%% ========================================================================
%  SECTION 6B: ARITHMETIC CHECKS  (the script checks its own math every run)
%  NOTE: these prove the code calculates its equations correctly. They do
%  NOT prove the inputs (balance limits, bearing waviness) are right.
%  ------------------------------------------------------------------------
%  Each check redoes a result a DIFFERENT way (by hand formula or a known
%  identity) and compares it with what the script calculated above.
%  PASS = they agree. Any FAIL means do not trust the results.
%  ========================================================================

check_names  = {};
check_passed = [];
check_tol    = 1e-9;      % relative tolerance

% 1. Known textbook answer: 1 kg, 1 mm offset, 60 RPM (1 rev/s) -> F = 0.001 x (2*pi)^2 = 0.0394784 N
hand_value = 1 * 0.001 * (2 * pi * 60 / 60) ^ 2;
check_names{end+1}  = '1. Textbook case: 1 kg, 1 mm, 60 RPM gives 0.0394784 N';
check_passed(end+1) = abs(hand_value - 0.0394784176) <= 1e-9;

% 2. Static force by hand, using speed in Hz:  F = Us x sqrt((2*pi*f)^4 + accel^2)
hand_value = static_imbalance_kgm .* sqrt((2 * pi * speed_hz) .^ 4 + (2 * pi * speed_hz / ramp_time_s) .^ 2);
check_names{end+1}  = '2. Static imbalance force   F = Us x sqrt(speed^4 + accel^2)';
check_passed(end+1) = max(abs(hand_value - static_imbalance_force_N)) <= check_tol * max(abs(hand_value));

% 3. Us recomputed from whichever way it was given
hand_value = zeros(1, number_of_rotors);
for r = 1:number_of_rotors
    if ~isempty(static_imbalance_entered{r})
        hand_value(r) = static_imbalance_entered{r};
    elseif ~isempty(eccentricity_entered{r})
        hand_value(r) = mass_kg(r) * eccentricity_entered{r} / 1e6;
    else
        hand_value(r) = mass_kg(r) * (balance_grade_entered{r} / 1000) / (2 * pi * max_speed_rpm(r) / 60);
    end
end
check_names{end+1}  = '3. Us = entered, or mass x e, or mass x G / speed';
check_passed(end+1) = max(abs(hand_value - static_imbalance_kgm)) <= check_tol * max([abs(hand_value), eps]);

% 4. Dynamic moment by hand:  M = Ud x (2*pi*f)^2
hand_value = dynamic_imbalance_kgm2 .* sqrt((2 * pi * speed_hz) .^ 4 + (2 * pi * speed_hz / ramp_time_s) .^ 2);
check_names{end+1}  = '4. Couple imbalance moment M = Ud x sqrt(speed^4 + accel^2)';
check_passed(end+1) = max(abs(hand_value - dynamic_imbalance_moment_Nm)) <= check_tol * max([abs(hand_value), eps]);

% 5. Ud recomputed: entered, or the static tolerance in two planes b apart, opposed
hand_value = zeros(1, number_of_rotors);
for r = 1:number_of_rotors
    if ~isempty(couple_imbalance_entered{r})
        hand_value(r) = couple_imbalance_entered{r};
    else
        hand_value(r) = 0.5 * static_imbalance_kgm(r) * plane_spacing_entered{r};
    end
end
check_names{end+1}  = '5. Ud = entered, or (Us / 2) x plane spacing';
check_passed(end+1) = max(abs(hand_value - dynamic_imbalance_kgm2)) <= check_tol * max([abs(hand_value), eps]);

% 6. Bearing identities (exact for any bearing):
%    cup pass + cone pass = number of rollers,  cup pass = rollers x (1 - cage)
identity_1 = bearing_harmonic(:, 2) + bearing_harmonic(:, 3) - bearing_rollers';
identity_2 = bearing_harmonic(:, 2) - bearing_rollers' .* (1 - bearing_harmonic(:, 1));
check_names{end+1}  = '6. Bearing identities: cup + cone = rollers, cup = rollers x (1 - cage)';
check_passed(end+1) = max(abs([identity_1; identity_2])) <= 1e-9;

% 7. Bearing radial stiffness (Z/2) k cos^2 = sum over rollers of k cos^2(angle) cos^2(roller position)
stiffness_error = 0;
for r = 1:number_of_rotors
    roller_angle = 2 * pi * (0:bearing_rollers(r) - 1) / bearing_rollers(r);
    hand_value = sum(roller_stiffness_Npm(r) * cos(cup_angle_rad(r)) ^ 2 * cos(roller_angle) .^ 2);
    stiffness_error = max(stiffness_error, abs(hand_value - bearing_radial_stiffness_Npm(r)) / hand_value);
end
check_names{end+1}  = '7. Bearing radial stiffness = sum over every roller';
check_passed(end+1) = stiffness_error <= 1e-9;

% 8. Twist by hand:  Mz = Izz x speed / ramp time
hand_value = Izz_kgm2 .* (2 * pi * max_speed_rpm / 60) / ramp_time_s;
check_names{end+1}  = '8. Ramp twist Mz = Izz x speed / ramp time';
check_passed(end+1) = max(abs(hand_value - abs(imbalance_loads(6, :)))) <= check_tol * max(hand_value);

% 9. Tipping moment = dynamic imbalance + sideways force x distance
hand_value = dynamic_imbalance_moment_Nm + static_imbalance_force_N .* abs(position_m - reference_position_m);
check_names{end+1}  = '9. Tipping moment = couple + force x distance';
check_passed(end+1) = max(abs(hand_value - imbalance_loads(4, :))) <= check_tol * max([hand_value, eps]);

% 10. Rotating loads: peak Fx = peak Fy and peak Mx = peak My, everywhere
check_names{end+1}  = '10. Fx = Fy and Mx = My for every part';
check_passed(end+1) = isequal(imbalance_loads(1, :), imbalance_loads(2, :)) && isequal(imbalance_loads(4, :), imbalance_loads(5, :)) ...
                   && isequal(bearing_loads(1, :), bearing_loads(2, :)) && isequal(bearing_loads(4, :), bearing_loads(5, :));

% 11. TOTAL = arm + wheel 1 + wheel 2 + all bearings  (Mz: net of the signed twists)
hand_total    = sum(imbalance_loads, 2) + sum(bearing_loads, 2);
hand_total(6) = abs(hand_total(6)) + sum(motor_torque_Nm);
hand_total(4:5) = hand_total(4:5) + abs(sum(gyro_moment_Nm(1:2, :), 2));
check_names{end+1}  = '11. TOTAL = arm + wheels + bearings + motors + gyroscopic';
check_passed(end+1) = max(abs(hand_total - all_total)) <= check_tol * max(abs(hand_total));

% 12. Waviness selection rule: summed over Z rollers, order kZ +/- 1 gives Z/2 (lateral),
%     order kZ gives Z (axial), any other order gives zero
rule_error = 0;
for r = 1:number_of_rotors
    Z = bearing_rollers(r);
    roller_angle = 2 * pi * (0:Z - 1) / Z;
    rule_error = max([rule_error, ...
        abs(sum(cos((Z + 1) * roller_angle) .* cos(roller_angle)) - Z / 2), ...
        abs(sum(cos((Z - 1) * roller_angle) .* cos(roller_angle)) - Z / 2), ...
        abs(sum(cos(Z * roller_angle)) - Z), ...
        abs(sum(cos((Z + 2) * roller_angle) .* cos(roller_angle)))]);
end
check_names{end+1}  = '12. Waviness rule: only orders kZ +/- 1 (lateral) and kZ (axial) give net force';
check_passed(end+1) = rule_error <= 1e-9;

% 13. Bearing totals = sum of every bearing line (bearing A + bearing B, through the transmissibility)
hand_lateral = zeros(1, number_of_rotors);
for i = 1:numel(line_dir)
    if line_dir(i) == 1
        ratio = line_freq_hz(i) / natural_freq_radial_hz(line_rotor(i));
        hand_lateral(line_rotor(i)) = hand_lateral(line_rotor(i)) + sum(line_blocked_N(i, :)) * ratio ^ 2 / sqrt((1 - ratio ^ 2) ^ 2 + (2 * bearing_damping_ratio * ratio) ^ 2);
    end
end
check_names{end+1}  = '13. Bearing lateral total = sum of all bearing lines';
check_passed(end+1) = max(abs(hand_lateral - bearing_lateral_N)) <= check_tol * max([hand_lateral, eps]);

% 14. Gyroscopic moment of the whole assembly = -(rate x total momentum), written out by hand
total_H = sum(spin_direction .* Izz_kgm2 .* (2 * pi * max_speed_rpm / 60));
w = reshape(spacecraft_rate_rad_s, 1, 3);
hand_value = [-w(2) * total_H; w(1) * total_H; 0];          % -(w x [0 0 H]) expanded
check_names{end+1}  = '14. Gyroscopic moment = -(spacecraft rate x total momentum)';
check_passed(end+1) = max(abs(hand_value - sum(gyro_moment_Nm, 2))) <= check_tol * max([abs(hand_value); 1e-12]);

% 15. Motor twist = ripple + cogging (blank = 0)
hand_value = zeros(1, number_of_rotors);
for r = 1:number_of_rotors
    if ~isempty(motor_ripple_entered{r}),  hand_value(r) = hand_value(r) + motor_ripple_entered{r};  end
    if ~isempty(motor_cogging_entered{r}), hand_value(r) = hand_value(r) + motor_cogging_entered{r}; end
end
check_names{end+1}  = '15. Motor twist Mz = ripple + cogging';
check_passed(end+1) = max(abs(hand_value - motor_torque_Nm)) <= 1e-12;

number_of_checks = numel(check_passed);
all_checks_ok    = all(check_passed);


%% ========================================================================
%  SECTION 7: PLOTS
%  ------------------------------------------------------------------------
%  Plot 1: sideways force (Fx = Fy) vs RPM    Plot 2: axial force (Fz) vs RPM
%  Curves are at constant speed (no ramp). Imbalance loads grow with speed squared. Bearing loads are recalculated at
%  every speed (their frequencies move against the rotor-bearing resonance).
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

    % bearing totals at every plotted speed: column 1 = lateral, column 2 = axial
    bearing_curve = zeros(numel(speed_fraction), 2);
    for i = 1:numel(speed_fraction)
        speed_line_force = bearing_line_force(speed_fraction(i));
        bearing_curve(i, 1) = sum(speed_line_force(line_dir == 1));
        bearing_curve(i, 2) = sum(speed_line_force(line_dir == 3));
    end

    for n = 1:2
        fprintf('Drawing plot %d...\n', n);
        try
            row = plot_row(n);
            if row == 1
                steady_row = steady_force_N;                 % imbalance lateral force, constant speed
            else
                steady_row = zeros(1, number_of_rotors);     % imbalance gives no axial force
            end
            curve_values = [steady_row(arm_index) * curve_shape, steady_row(1) * curve_shape, ...
                            steady_row(3) * curve_shape, bearing_curve(:, n)];
            curve_values = [curve_values, sum(curve_values, 2)];      % last column = TOTAL
            figure(n);
            clf;
            try
                set(gcf, 'Renderer', 'painters');
            catch
            end
            hold on;
            grid on;
            box on;
            for c = 1:size(curve_values, 2)
                line_width = 2;
                if c == size(curve_values, 2)
                    line_width = 3;
                end
                plot(arm_axis_rpm, curve_values(:, c), curve_lines{c}, ...
                     'Color', curve_colors(c, :), 'LineWidth', line_width);
            end
            [~, max_index] = min(abs(speed_fraction - 1));
            plot(max_speed_rpm(arm_index), curve_values(max_index, end), 'ko', 'MarkerFaceColor', 'k');
            text(max_speed_rpm(arm_index), curve_values(max_index, end), sprintf('%.3g N at max speed (steady)   ', curve_values(max_index, end)), ...
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
              'Mx  [N*m] tipping',  'My  [N*m] tipping',  'Mz  [N*m] twist'};
part_line  = repmat('=', 1, 90);

% ---- Arithmetic check results (from SECTION 6B) ----
fprintf('\n%s\n ARITHMETIC CHECKS  -  script math vs hand calculations\n', part_line);
fprintf(' (proves the math is done right, not that the inputs/estimates are right)\n%s\n', part_line);
for k = 1:number_of_checks
    if check_passed(k)
        fprintf('   PASS   %s\n', check_names{k});
    else
        fprintf('   FAIL   %s\n', check_names{k});
    end
end
if all_checks_ok
    fprintf('   ALL %d CHECKS PASSED\n', number_of_checks);
else
    fprintf('   *** %d OF %d CHECKS FAILED - DO NOT TRUST THE RESULTS BELOW ***\n', sum(~check_passed), number_of_checks);
end

% ---- Where Us and Ud came from ----
fprintf('\n   Imbalance used:\n');
for r = 1:number_of_rotors
    fprintf('   %-8s  Us = %-10.4g kg*m (%-12s)   Ud (couple) = %-10.4g kg*m^2 (%-10s)   Izz = %-8.4g kg*m^2 (%s)\n', rotor_names{r}, ...
        static_imbalance_kgm(r), static_source{r}, dynamic_imbalance_kgm2(r), dynamic_source{r}, Izz_kgm2(r), inertia_source{r});
end

part_titles  = {sprintf('PART 1: ARM  -  static + couple imbalance  (%.0f RPM)', max_speed_rpm(arm_index)), ...
                sprintf('PART 2: MOMENTUM WHEELS  -  static + couple imbalance  (%.0f RPM)', max_speed_rpm(1)), ...
                'PART 3: BEARINGS  -  geometry, stiffness, preload and specified waviness', ...
                'PART 4: MOTORS (torque ripple, cogging) AND GYROSCOPIC MOMENT', ...
                'PART 5: ALL TOGETHER  -  FINAL ANSWER, WORST-CASE ESTIMATE'};
part_columns = {{'Arm'}, ...
                {rotor_names{1}, rotor_names{3}, 'Wheels total'}, ...
                {[rotor_names{1}, ' brg'], [rotor_names{2}, ' brg'], [rotor_names{3}, ' brg'], 'Bearings total'}, ...
                {[rotor_names{1}, ' mot'], [rotor_names{2}, ' mot'], [rotor_names{3}, ' mot'], 'Motors total', 'Gyroscopic'}, ...
                {'Arm', 'Wheels', 'Bearings', 'Motors', 'Gyroscopic', 'TOTAL'}};
part_values  = {arm_loads, ...
                [wheel_loads, wheels_total], ...
                [bearing_loads, bearings_total], ...
                [motor_loads, motors_total, gyro_total], ...
                [arm_loads, wheels_total, bearings_total, motors_total, gyro_total, all_total]};

for part = 1:5
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
        fprintf('\n   Fx = Fy: static imbalance %.4g N at the top of the ramp (%.4g N at constant speed)\n', ...
            static_imbalance_force_N(arm_index), static_force_steady_N(arm_index));
        fprintf('   Mx = My: couple imbalance %.4g N*m + force x distance %.4g N*m\n', ...
            dynamic_imbalance_moment_Nm(arm_index), lever_arm_m(arm_index) * static_imbalance_force_N(arm_index));
        fprintf('   Frequency: 1x RPM = %.2f Hz\n', speed_hz(arm_index));
        fprintf('   Mz twist on mount: spin-up %+.4g N*m, spin-down %+.4g N*m  (table shows the larger)\n', ...
            twist_spin_up_Nm(arm_index), twist_spin_down_Nm(arm_index));
    elseif part == 2
        for r = wheel_columns
            fprintf('\n   %s:  Fx = Fy static imbalance %.4g N  |  Mx = My: couple imbalance %.4g + force x distance %.4g N*m  |  1x = %.2f Hz', ...
                rotor_names{r}, static_imbalance_force_N(r), dynamic_imbalance_moment_Nm(r), ...
                lever_arm_m(r) * static_imbalance_force_N(r), speed_hz(r));
            fprintf('\n   %s:  Mz twist on mount: spin-up %+.4g N*m, spin-down %+.4g N*m', ...
                rotor_names{r}, twist_spin_up_Nm(r), twist_spin_down_Nm(r));
        end
        fprintf('\n');
    end
    if part == 3
        for r = 1:number_of_rotors
            if linearity_ratio(r) > 0.5
                fprintf('   WARNING %s: waviness is %.0f%% of the preload deflection - rollers may unload,\n', ...
                    rotor_names{r}, 100 * linearity_ratio(r));
                fprintf('           the linear bearing model is not reliable. Raise the preload or check the inputs.\n');
            end
        end

        fprintf('\n   Characteristic frequencies at max speed [Hz]:\n');
        fprintf('   %-8s %12s %16s %16s %13s\n', 'Rotor', harmonic_names{:});
        for r = 1:number_of_rotors
            fprintf('   %-8s %12.2f %16.2f %16.2f %13.2f\n', rotor_names{r}, bearing_frequency_hz(r, :));
        end

        fprintf('\n   Each bearing force above is the sum of its lines (frequencies listed here and\n');
        fprintf('   their multiples). Line-by-line values are in line_freq_hz and line_force_N.\n');
    elseif part == 4
        fprintf('\n   Motor torque ripple and cogging (Mz on the mount):\n');
        for r = 1:number_of_rotors
            if isempty(motor_ripple_entered{r}) && isempty(motor_cogging_entered{r})
                fprintf('   %-8s not included (motor data not entered)\n', rotor_names{r});
            else
                fprintf('   %-8s ripple %.4g N*m + cogging %.4g N*m = %.4g N*m\n', rotor_names{r}, ...
                    sum([motor_ripple_entered{r}, 0]), sum([motor_cogging_entered{r}, 0]), motor_torque_Nm(r));
            end
        end
        fprintf('\n   Gyroscopic moment: spacecraft rate [%g %g %g] rad/s\n', spacecraft_rate_rad_s);
        fprintf('   Stored momentum H [N*m*s]: %s %+.4g, %s %+.4g, %s %+.4g   ->  total %+.4g\n', ...
            rotor_names{1}, momentum_Nms(1), rotor_names{2}, momentum_Nms(2), rotor_names{3}, momentum_Nms(3), sum(momentum_Nms));
        fprintf('   Gyroscopic column = net moment of the assembly, -(rate x total H).\n');
        if all(spacecraft_rate_rad_s == 0)
            fprintf('   Spacecraft rate is zero (fixed base), so there is no gyroscopic moment.\n');
        end
    elseif part == 5
        fprintf('\n   Mz TOTAL is the NET twist: the wheels twist opposite to the arm, so they cancel.\n');
        fprintf('   Each motor mount still feels its own twist (see PART 1 and PART 2).\n');
        fprintf('   Net Mz: spin-up %+.4g N*m, spin-down %+.4g N*m  (table shows the larger)\n', ...
            sum(twist_spin_up_Nm), sum(twist_spin_down_Nm));
        fprintf('\n   HOW TO READ THIS TABLE:\n');
        fprintf('   - Fx and Fy are peak values reached a quarter turn apart, NOT at the same time.\n');
        fprintf('     The sideways resultant = Fx (not 1.41 x Fx). Same for Mx and My.\n');
        fprintf('   - WORST CASE: every source and every moment part is added as if their peaks\n');
        fprintf('     line up (phases are unknown). Real loads are equal or lower.\n');
        fprintf('   - Imbalance values are the specified balance limits. If Ud was not entered it is the static\n');
        fprintf('     tolerance in two planes, opposed (worst case). Bearing loads follow from the specified waviness.\n');
        fprintf('   - Mz TOTAL = net ramp twist + motor ripple and cogging. Gyroscopic moment is added to Mx, My.\n');
        fprintf('\n   ASSUMPTIONS: hard-mounted (rigid base). Imbalance and bearing waviness at their specified limits.\n');
        fprintf('   NOT INCLUDED: structural resonance, broadband noise, rotor flexibility, localized bearing defects, harmonics above 1x RPM,\n');
        fprintf('                 bearing varying compliance (small: bearings preloaded).\n');
        if all(cellfun(@isempty, [motor_ripple_entered, motor_cogging_entered]))
            fprintf('   NOT YET ENTERED: motor ripple/cogging data (that source is zero until entered).\n');
        end
        if ~isempty(report_arm_rpm)
            fprintf('\n   AT OTHER SPEEDS  (constant speed, no ramp; wheels follow the arm; Fx, Fz, Mx totals)\n');
            fprintf('   %10s %12s %14s %14s %16s\n', 'Arm RPM', 'Wheel RPM', 'Fx = Fy [N]', 'Fz [N]', 'Mx = My [N*m]');
            for i = 1:numel(report_arm_rpm)
                speed_ratio = report_arm_rpm(i) / max_speed_rpm(arm_index);
                speed_line_force = bearing_line_force(speed_ratio);
                lateral_lines = (line_dir == 1);
                speed_Fx = sum(steady_force_N) * speed_ratio ^ 2 + sum(speed_line_force(lateral_lines));
                speed_Fz = sum(speed_line_force(line_dir == 3));
                speed_gyro = abs(-spacecraft_rate(2) * sum(momentum_Nms)) * speed_ratio;   % H grows with speed
                speed_Mx = sum(steady_moment_Nm) * speed_ratio ^ 2 + ...
                           sum(speed_line_force(lateral_lines) .* lever_arm_m(line_rotor(lateral_lines))) + speed_gyro;
                fprintf('   %10.1f %12.1f %14.4g %14.4g %16.4g\n', report_arm_rpm(i), max_speed_rpm(1) * speed_ratio, ...
                    speed_Fx, speed_Fz, speed_Mx);
            end
        end
    end
end
fprintf('%s\n', part_line);
