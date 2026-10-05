%% =========================================================================
%  ArmLoads.m
%  Loads and moments induced on the structure by the ROTATING ARM.
%
%  WHY THE ARM NEEDS ITS OWN SCRIPT
%    A momentum wheel is a disk: axisymmetric, compact, imbalance only.
%    The arm is a long symmetric bar spinning about the shaft, which
%    changes what matters:
%      1. SYMMETRIC means the nominal imbalance is zero.  The loads come
%         from TOLERANCES: a CG offset (one end heavier) and a TILT of the
%         arm away from perpendicular to the shaft.  Because the arm is
%         long, its spin inertia is large and a tilt of a few hundredths
%         of a degree already gives a large 1x moment.
%      2. A bar is NOT axisymmetric (its inertia about its own length is
%         tiny).  If the shaft axis is ever rotated, the gyroscopic
%         moment pulses at TWICE the spin frequency (2x).
%      3. CENTRIFUGAL tension in the arm root is usually the largest load
%         of all.  It cancels between the two halves, so the shaft does
%         not see it, but the hub and root must carry it.
%      4. AIR DRAG on a long arm at high speed can need a large drive
%         torque, which the structure reacts.  Zero in vacuum.
%    The script uses the full rigid-body (Newton-Euler) equations of a
%    spinning body with the arm's CAD inertia, so every one of these
%    effects comes out of the same calculation.
%
%  QUICK START
%    1. Edit section 1 "USER INPUTS" only.  Follow STEP 1 to STEP 4 and
%       fill in every blank:  [ ] takes one number,  [ ; ; ] takes x; y; z.
%       Units are written on every line.  Optional items can stay NaN.
%       If a required value is empty or has the wrong shape the script
%       stops and names it.
%    2. Press Run.
%    3. Read the SUMMARY block printed last in the Command Window (also
%       saved to ArmLoads_summary.txt).  Those are the numbers to quote.
%
%  WHAT YOU GET FROM CAD  (mass properties of everything that spins
%  with the arm: arm, hub, tip parts, motor rotor)
%    - mass
%    - moments of inertia TAKEN AT THE CENTRE OF MASS and ALIGNED WITH a
%      coordinate system whose x is the shaft axis and y runs along the
%      arm  ->  Ixx (about the shaft), Iyy (about the arm's length), Izz
%    - the position of the arm hub on the shaft
%    - optional: mass and CG radius of ONE half of the arm (for the root
%      tension), arm width and tip radius (for air drag)
%    CAD often reports g*mm^2: divide by 1e9 for kg*m^2.
%  WHAT CAD CANNOT GIVE YOU
%    The imbalance.  A CAD model is perfectly balanced.  Use your
%    balancing spec (ISO 21940 grade, e.g. G2.5) or a mass / position
%    tolerance, and a tilt tolerance from the drawing.
%
%  THE LAYOUT
%
%        MW-1                    ARM                     MW-2
%      +------+          <-- long, symmetric -->       +------+
%      | wheel|   bearing    ====|=|====    bearing    | wheel|
%      |      |------------------+-+--------------------|      |-----> x
%      +------+                  |                      +------+
%                         x = 0 (reference point)
%     - shaft axis = structure x axis, arm spins about +x at 3000 rpm
%     - the arm lies along y at the start of the revolution
%     - loads are reported at the reference point and, optionally, split
%       into the two shaft bearings
%
%  WHICH OUTPUT IS WHICH
%    STEADY  : constant part of the load (mean over one revolution):
%              weight / base acceleration, gyroscopic moment if the shaft
%              axis rotates, spin-up torque, drag torque.
%    1x      : once-per-revolution part: CG offset and tilt (imbalance).
%    2x      : twice-per-revolution part: arm not axisymmetric, appears
%              when the shaft axis rotates.
%    PEAK    : largest value over one revolution = steady + all harmonics
%              with their real phases.  THIS IS THE DESIGN LOAD.
%    BEARINGS: the same loads split into the two shaft bearings.
%    INTERNAL: centrifugal tension in the arm root (not seen by the shaft).
%
%  REFERENCES
%    Rigid-body Newton-Euler equations in a rotating frame (any dynamics
%    text, e.g. Greenwood, "Principles of Dynamics").
%    ISO 21940-11 balance quality grades:  e = G / omega.
%    Southwell coefficient for rotating-beam stiffening:
%      f(Omega)^2 = f0^2 + K * (Omega/2pi)^2.
%    Liu, Maghami, Blaurock (2008) and Zhang et al. (2014) for the
%    imbalance-driven 1x disturbance model also used in
%    MomentumWheelLoads.m.
%
%  MATLAB R2016b or newer.  (Under GNU Octave move the LOCAL FUNCTIONS
%  block to the top of the file, right after a line containing only "1;".)
%% =========================================================================
clear; clc; close all;

%% ========================================================================
%% 1. USER INPUTS      <<< the only section you need to edit >>>
%% ========================================================================
%
%   STEP 1  reference point
%   STEP 2  arm - required (CAD + tolerances)
%   STEP 3  optional data (leave as is if you don't have it)
%   STEP 4  analysis settings
%
%   The blank shows the shape of what goes in it:
%        [ ]        one number            e.g.  4.2
%        [ ; ; ]    three numbers x; y; z  e.g.  [0; 0; 0]
%   Vectors are [x; y; z] in the structure frame.  Units are in brackets.

%% ---- STEP 1: reference point -------------------------------------------------
reference_point       = [0; 0; 0];            % [m]  loads are reported here.
                                              %      [0;0;0] = shaft centre

%% ---- STEP 2: ARM, required ------------------------------------------------------
arm_name              = 'ARM';
%  position and orientation (CAD)
arm_position          = [ ; ; ];              % [m]  [x; y; z] centre of the arm hub
                                              %      on the shaft axis, e.g. [0; 0; 0]
arm_spin_axis         = [1; 0; 0];            % [-]  direction of the arm's angular
                                              %      momentum (right-hand rule): +x,
                                              %      opposite to the wheels (-x)
arm_direction         = [0; 1; 0];            % [-]  direction the arm points at the
                                              %      start of a revolution.  Must be
                                              %      perpendicular to arm_spin_axis
arm_rpm               = 3000;                 % [rpm] operating speed
%  mass properties (CAD, everything that spins with the arm)
arm_mass              = [ ];                  % [kg]
arm_Ixx               = [ ];                  % [kg*m^2] about the SHAFT axis, through CG
arm_Iyy               = [ ];                  % [kg*m^2] about the arm's LENGTH, through CG
                                              %          (small for a slender arm)
arm_Izz               = [ ];                  % [kg*m^2] about the third axis, through CG
%  static imbalance (tolerance, NOT from CAD).  Give ONE of:
%     arm_static_imbalance (here)  or  arm_cg_offset_mm  or  arm_balance_grade
%     (both in STEP 3).  Enter 0 here for a perfectly balanced arm.
%     Units:  'g*mm'  'g*cm'  'kg*m'  'oz*in'
%     For "one tip may be dm grams heavier at radius r mm":  dm*r  in g*mm
arm_static_imbalance  = [ ];    arm_static_unit  = 'g*mm';
%  dynamic imbalance (tolerance, NOT from CAD).  Give ONE of:
%     arm_dynamic_imbalance (here)  or  arm_tilt_deg (STEP 3).
%     Enter 0 here for a perfectly straight arm.
%     Units:  'g*mm^2'  'g*cm^2'  'kg*m^2'  'oz*in^2'
arm_dynamic_imbalance = [ ];    arm_dynamic_unit = 'g*mm^2';

%% ---- STEP 3: optional data.  Leave as NaN / 0 if unknown ----------------------
%  Imbalance alternatives
arm_cg_offset_mm      = NaN;                  % [mm]   CG offset from the shaft axis.
                                              %        Used if arm_static_imbalance = [ ]
arm_balance_grade     = NaN;                  % [mm/s] ISO 21940 balance grade G, e.g.
                                              %        2.5 for G2.5.  e = G/omega at
                                              %        arm_rpm.  Used if both of the
                                              %        above are empty
arm_tilt_deg          = NaN;                  % [deg]  tilt of the arm away from
                                              %        perpendicular to the shaft (drawing
                                              %        tolerance).  Used if
                                              %        arm_dynamic_imbalance = [ ]
arm_imbalance_phase   = 0;                    % [rad]  angle between CG offset and tilt.
                                              %        Unknown -> 0
%  Products of inertia from CAD (about CG, same axes).  ~0 for a symmetric arm
arm_Ixy               = 0;                    % [kg*m^2]
arm_Ixz               = 0;                    % [kg*m^2]
arm_Iyz               = 0;                    % [kg*m^2]
arm_cad_products      = 'integral';           % 'integral': CAD lists +integral(xy dm)
                                              %   (e.g. SolidWorks Lxy).  'tensor': CAD
                                              %   lists the inertia tensor with negative
                                              %   off-diagonal terms.  Irrelevant if 0
%  Root (internal) load
arm_half_mass         = NaN;                  % [kg]   mass of ONE half of the arm (CAD:
                                              %        split at the hub)
arm_half_cg_radius    = NaN;                  % [m]    distance from the shaft axis to that
                                              %        half's CG
arm_root_area_mm2     = NaN;                  % [mm^2] cross-section area at the root ->
                                              %        root tensile stress
%  Arm bending modes (non-rotating, from FE or tap test) for the Campbell diagram
arm_flap_freq         = NaN;                  % [Hz] out-of-plane (along shaft) bending
arm_edge_freq         = NaN;                  % [Hz] in-plane (in the spin plane) bending
arm_flap_southwell    = 1.17;                 % [-]  centrifugal stiffening coefficient
arm_edge_southwell    = 0.17;                 % [-]  (uniform cantilever values)
%  Air drag.  0 density = vacuum
air_density           = 0;                    % [kg/m^3] 1.225 at sea level
arm_drag_coeff        = 1.2;                  % [-]  ~1.2 flat bar, ~1.0 round bar
arm_width             = NaN;                  % [m]  frontal width of the arm facing the
                                              %      direction of motion
arm_tip_radius        = NaN;                  % [m]  shaft axis to arm tip
%  Motion of the structure that carries the shaft bearings
base_rate             = [0; 0; 0];            % [rad/s] angular rate of the structure.
                                              %   0 when the shaft axis is fixed.  If the
                                              %   shaft axis is turned: gyroscopic loads
spin_accel            = 0;                    % [rad/s^2] arm spin-up (+) / down (-) rate
base_accel            = [0; 0; 0];            % [m/s^2] quasi-static acceleration.  On the
                                              %   ground with z up use [0; 0; 9.81] to get
                                              %   the arm weight
%  Shaft bearings: loads are also split into the two bearings if given
bearing_A_position    = [NaN; NaN; NaN];      % [m] [x; y; z] bearing A on the shaft axis
bearing_B_position    = [NaN; NaN; NaN];      % [m] [x; y; z] bearing B on the shaft axis
axial_bearing         = 'A';                  % 'A' or 'B': bearing taking axial load

%% ---- STEP 4: analysis settings --------------------------------------------------
assembly_mass         = [ ];                  % [kg] mass of the whole assembly the arm
                                              %      loads shake (shaft + arm + wheels).
                                              %      For the acceleration plot: a = F/m
rpm_sweep             = 0:50:4500;            % [rpm] speed sweep
structure_modes_Hz    = [];                   % [Hz] known structure modes for the
                                              %      Campbell diagram, e.g. [35 60 110]
accel_limit_g         = NaN;                  % [g]  limit line on the acceleration plot
n_per_rev             = 360;                  % samples per revolution
makePlots             = true;                 % figures on/off
saveOutputs           = true;                 % write .mat, .csv and summary .txt on/off

%% ========================================================================
%%  (end of user inputs - nothing below needs editing)
%% ========================================================================

%% ========================================================================
%% 2. COLLECT AND CHECK INPUTS
%% ========================================================================
notes = {};
P_ref = need(reference_point, 'reference_point', 3);
m_struct = need(assembly_mass, 'assembly_mass', 1);

arm = struct();
arm.name = arm_name;
arm.p    = need(arm_position,  'arm_position',  3);
s        = need(arm_spin_axis, 'arm_spin_axis', 3);
assert(norm(s) > 0, 'arm_spin_axis must not be zero');
arm.s    = s / norm(s);
d        = need(arm_direction, 'arm_direction', 3);
d        = d - (d.' * arm.s) * arm.s;
if norm(d) < 1e-9
    error('arm_direction must not be parallel to arm_spin_axis.');
end
arm.d    = d / norm(d);
arm.n    = cross3(arm.s, arm.d);
arm.B0   = [arm.s, arm.d, arm.n];            % body axes x' (spin), y' (arm), z'
arm.rpm  = need(arm_rpm,  'arm_rpm',  1);
arm.Om   = arm.rpm * 2*pi/60;
arm.m    = need(arm_mass, 'arm_mass', 1);
Ixx = need(arm_Ixx, 'arm_Ixx', 1);
Iyy = need(arm_Iyy, 'arm_Iyy', 1);
Izz = need(arm_Izz, 'arm_Izz', 1);
switch lower(arm_cad_products)
    case 'integral', sg = -1;
    case 'tensor',   sg =  1;
    otherwise, error('arm_cad_products must be ''integral'' or ''tensor''.');
end
Ib = [Ixx,         sg*arm_Ixy, sg*arm_Ixz;
      sg*arm_Ixy,  Iyy,        sg*arm_Iyz;
      sg*arm_Ixz,  sg*arm_Iyz, Izz];
arm.Ib_cad = Ib;
ph = arm_imbalance_phase;

% --- static imbalance -> CG offset in the body frame ---
Us = empty_to_nan(arm_static_imbalance);
if ~isnan(Us)
    arm.U = imbalance_to_SI(Us, arm_static_unit, 1);
    notes{end+1} = sprintf('static imbalance from arm_static_imbalance = %g %s', Us, arm_static_unit);
elseif ~isnan(arm_cg_offset_mm)
    arm.U = arm.m * arm_cg_offset_mm * 1e-3;
    notes{end+1} = sprintf('static imbalance from CG offset %g mm', arm_cg_offset_mm);
elseif ~isnan(arm_balance_grade)
    assert(arm.Om > 0, 'arm_balance_grade needs a non-zero arm_rpm');
    e = arm_balance_grade*1e-3 / arm.Om;
    arm.U = arm.m * e;
    notes{end+1} = sprintf('static imbalance from balance grade G%g at %g rpm: CG offset %.4g mm', ...
        arm_balance_grade, arm.rpm, e*1e3);
else
    error(['Enter arm_static_imbalance (or arm_cg_offset_mm, or arm_balance_grade) ' ...
           'in section 1.  Enter 0 for a perfectly balanced arm.']);
end
arm.e   = arm.U / arm.m;                                 % [m] CG offset
arm.c_b = arm.e * [0; 1; 0];                             % along the arm

% --- dynamic imbalance -> product of inertia / tilted inertia tensor ---
Ud = empty_to_nan(arm_dynamic_imbalance);
if ~isnan(Ud)
    Ud_SI = imbalance_to_SI(Ud, arm_dynamic_unit, 2);
    Ib(1,2) = Ib(1,2) - Ud_SI*cos(ph);   Ib(2,1) = Ib(1,2);
    Ib(1,3) = Ib(1,3) - Ud_SI*sin(ph);   Ib(3,1) = Ib(1,3);
    notes{end+1} = sprintf('dynamic imbalance from arm_dynamic_imbalance = %g %s', Ud, arm_dynamic_unit);
elseif ~isnan(arm_tilt_deg)
    tilt_axis = [0; -sin(ph); cos(ph)];                  % tilt in the spin/arm plane
    Rt = rodrigues(tilt_axis, arm_tilt_deg*pi/180);
    Ib = Rt * Ib * Rt.';
    notes{end+1} = sprintf('dynamic imbalance from a %g deg arm tilt', arm_tilt_deg);
else
    error(['Enter arm_dynamic_imbalance (or arm_tilt_deg) in section 1.  ' ...
           'Enter 0 for a perfectly straight arm.']);
end
arm.Ib  = Ib;
arm.Ud_total = norm(Ib(1,2:3));                          % |product of inertia| with spin axis
arm.H   = arm.Ib(1,1) * arm.Om * arm.s;                  % nominal momentum (axis part)

% --- air drag ---
drag_on = air_density > 0;
if drag_on
    need(arm_width, 'arm_width', 1);  need(arm_tip_radius, 'arm_tip_radius', 1);
    drag_k = air_density * arm_drag_coeff * arm_width * arm_tip_radius^4 / 4;   % T = k*Om^2
    notes{end+1} = sprintf('air drag on: rho = %g kg/m^3, Cd = %g, width %g m, tip radius %g m', ...
        air_density, arm_drag_coeff, arm_width, arm_tip_radius);
else
    drag_k = 0;
    notes{end+1} = 'air drag off (air_density = 0, vacuum)';
end

% --- bearings ---
bear_on = all(~isnan(bearing_A_position)) && all(~isnan(bearing_B_position));
if bear_on
    pA = need(bearing_A_position, 'bearing_A_position', 3);
    pB = need(bearing_B_position, 'bearing_B_position', 3);
    xa = arm.s.' * (pA - P_ref);   xb = arm.s.' * (pB - P_ref);
    if abs(xa - xb) < 1e-9, error('Bearing A and B are at the same axial position.'); end
    offA = norm((pA - P_ref) - xa*arm.s);  offB = norm((pB - P_ref) - xb*arm.s);
    ref_off = norm((P_ref - arm.p) - (arm.s.'*(P_ref - arm.p))*arm.s);
    if max([offA offB ref_off]) > 1e-3
        notes{end+1} = 'WARNING: bearings or reference point are not on the shaft axis; bearing split assumes they are';
    end
    axial_to = upper(axial_bearing);
    assert(any(strcmp(axial_to, {'A','B'})), 'axial_bearing must be ''A'' or ''B''');
else
    notes{end+1} = 'bearing positions not given: no bearing split';
end

omega_b = base_rate(:);  a_b = base_accel(:);  alpha = spin_accel;
if abs(Iyy - Izz) > 1e-3*max(Iyy, Izz) && norm(omega_b) == 0
    notes{end+1} = 'arm is not axisymmetric (Iyy ~= Izz): a 2x load appears if base_rate is non-zero';
end
if isnan(arm_half_mass) || isnan(arm_half_cg_radius)
    notes{end+1} = 'arm_half_mass / arm_half_cg_radius not given: root tension not computed';
end

%% ========================================================================
%% 3. NOMINAL SPEED: ONE REVOLUTION
%% ========================================================================
nom = arm_one_rev(arm, arm.Om, alpha, omega_b, a_b, P_ref, drag_k*arm.Om^2, n_per_rev);
nom.theta_deg = (0:n_per_rev-1) * 360/n_per_rev;
nom.F = tidy(nom.F);  nom.M = tidy(nom.M);       % round-off -> exact 0
nom.harm = harmonics_of([nom.F; nom.M], 3);       % steady, 1x, 2x, 3x
nom.peak = max(abs([nom.F; nom.M]), [], 2);
[nom.F1_lat, nom.F1_ax] = lateral_axial_peak(nom.harm.p(1:3,1), arm.s);
[nom.F2_lat, nom.F2_ax] = lateral_axial_peak(nom.harm.p(1:3,2), arm.s);
nom.drag_T = drag_k * arm.Om^2;
nom.drag_P = nom.drag_T * arm.Om;
if bear_on
    [nom.RA, nom.RB, nom.Tmotor] = bearing_split(nom.F, nom.M, arm.s, xa, xb, axial_to);
    tolT = 1e-9 * max(abs([nom.F(:); nom.M(:)]));
    nom.Tmotor(abs(nom.Tmotor) < tolT) = 0;
    nom.RA_rad_peak = max(vnorm(nom.RA - arm.s*(arm.s.'*nom.RA)));
    nom.RB_rad_peak = max(vnorm(nom.RB - arm.s*(arm.s.'*nom.RB)));
    nom.RA_ax_peak  = max(abs(arm.s.'*nom.RA));
    nom.RB_ax_peak  = max(abs(arm.s.'*nom.RB));
end
% internal root load
root = struct('on', ~isnan(arm_half_mass) && ~isnan(arm_half_cg_radius));
if root.on
    root.F = arm_half_mass * arm_half_cg_radius * arm.Om^2;            % [N]
    if ~isnan(arm_root_area_mm2), root.sigma_MPa = root.F / arm_root_area_mm2;
    else, root.sigma_MPa = NaN; end
end

%% ========================================================================
%% 4. SPEED SWEEP
%% ========================================================================
nS = numel(rpm_sweep);
nps = max(36, min(n_per_rev, 72));                 % enough samples for 3x
sweep = struct('rpm', rpm_sweep(:));
sweep.steady = zeros(nS,6); sweep.A1 = zeros(nS,6); sweep.A2 = zeros(nS,6);
sweep.peak = zeros(nS,6);
sweep.F1_lat = zeros(nS,1); sweep.F1_ax = zeros(nS,1);
sweep.F2_lat = zeros(nS,1); sweep.F2_ax = zeros(nS,1);
sweep.drag_T = zeros(nS,1); sweep.root_F = NaN(nS,1);
for k = 1:nS
    Om = rpm_sweep(k)*2*pi/60;
    r1 = arm_one_rev(arm, Om, alpha, omega_b, a_b, P_ref, drag_k*Om^2, nps);
    r1.F = tidy(r1.F);  r1.M = tidy(r1.M);
    hk = harmonics_of([r1.F; r1.M], 2);
    sweep.steady(k,:) = hk.mean.';
    sweep.A1(k,:) = abs(hk.p(:,1)).';
    sweep.A2(k,:) = abs(hk.p(:,2)).';
    sweep.peak(k,:) = max(abs([r1.F; r1.M]), [], 2).';
    [sweep.F1_lat(k), sweep.F1_ax(k)] = lateral_axial_peak(hk.p(1:3,1), arm.s);
    [sweep.F2_lat(k), sweep.F2_ax(k)] = lateral_axial_peak(hk.p(1:3,2), arm.s);
    sweep.drag_T(k) = drag_k*Om^2;
    if root.on, sweep.root_F(k) = arm_half_mass*arm_half_cg_radius*Om^2; end
end
g0 = 9.80665;
accel = struct();
accel.f1 = sweep.rpm/60;          accel.f2 = 2*sweep.rpm/60;
accel.lat1_g = sweep.F1_lat/m_struct/g0;  accel.ax1_g = sweep.F1_ax/m_struct/g0;
accel.lat2_g = sweep.F2_lat/m_struct/g0;  accel.ax2_g = sweep.F2_ax/m_struct/g0;

%% ========================================================================
%% 5. PRINTED REPORT
%% ========================================================================
lbl = {'Fx [N]','Fy [N]','Fz [N]','Mx [N*m]','My [N*m]','Mz [N*m]'};
fprintf('=====================================================================\n');
fprintf('  ARM INDUCED LOADS  -  loads applied by the arm on the structure,\n');
fprintf('  structure frame, reduced to P_ref = [%g %g %g] m\n', P_ref);
fprintf('=====================================================================\n\n');
fprintf('--- Arm data as used ------------------------------------------------\n');
fprintf('   hub position     [%7.3f %7.3f %7.3f] m\n', arm.p);
fprintf('   spin axis        [%7.3f %7.3f %7.3f]   arm direction at start [%7.3f %7.3f %7.3f]\n', arm.s, arm.d);
fprintf('   speed            %g rpm = %.4g Hz = %.4g rad/s\n', arm.rpm, arm.Om/(2*pi), arm.Om);
fprintf('   mass             %.4g kg\n', arm.m);
fprintf('   inertia (CAD)    Ixx %.4g  Iyy %.4g  Izz %.4g kg*m^2\n', diag(arm.Ib_cad));
fprintf('   static imbalance %.4g kg*m  = CG offset %.4g mm  -> 1x force U*W^2 = %.4g N\n', ...
    arm.U, arm.e*1e3, arm.U*arm.Om^2);
fprintf('   product of inertia with spin axis %.4g kg*m^2  -> 1x moment P*W^2 = %.4g N*m\n', ...
    arm.Ud_total, arm.Ud_total*arm.Om^2);
fprintf('   angular momentum %.4g N*m*s along [%g %g %g]\n', norm(arm.H), arm.s);
fprintf('\n');

fprintf('--- Loads on the structure at %g rpm, at P_ref ---------------------\n', arm.rpm);
fprintf('%-12s%12s%12s%12s%12s\n', '', 'steady', '1x amp', '2x amp', 'PEAK');
for r = 1:6
    fprintf('%-12s%12.4g%12.4g%12.4g%12.4g\n', lbl{r}, nom.harm.mean(r)+0, ...
        abs(nom.harm.p(r,1)), abs(nom.harm.p(r,2)), nom.peak(r));
end
fprintf('   steady = mean over one revolution, 1x/2x = amplitude at %.4g / %.4g Hz,\n', arm.Om/(2*pi), arm.Om/pi);
fprintf('   PEAK = largest |value| over one revolution (all parts with their phases)\n\n');

if bear_on
fprintf('--- Bearing loads (A at %g m, B at %g m along the shaft, axial on %s) ---\n', xa, xb, axial_to);
fprintf('   bearing A: peak radial %.4g N, peak axial %.4g N\n', nom.RA_rad_peak, nom.RA_ax_peak);
fprintf('   bearing B: peak radial %.4g N, peak axial %.4g N\n', nom.RB_rad_peak, nom.RB_ax_peak);
fprintf('   torque about the shaft axis (to the drive / motor mount): steady %.4g N*m\n\n', mean(nom.Tmotor));
end

fprintf('--- Drive torque and drag ------------------------------------------\n');
fprintf('   spin-up torque Ixx*alpha        %.4g N*m\n', arm.Ib(1,1)*alpha);
fprintf('   air drag torque at %g rpm      %.4g N*m  (power %.4g W)\n', arm.rpm, nom.drag_T, nom.drag_P);
fprintf('\n');

fprintf('--- Internal load: centrifugal tension at the arm root -------------\n');
if root.on
    fprintf('   per half: m_half*r_cg*W^2 = %.4g N', root.F);
    if ~isnan(root.sigma_MPa), fprintf('   ->  root stress %.4g MPa', root.sigma_MPa); end
    fprintf('\n   (the two halves cancel at the shaft; the hub and root carry this)\n\n');
else
    fprintf('   not computed (give arm_half_mass and arm_half_cg_radius)\n\n');
end

fprintf('--- Rigid-body acceleration of the %g kg assembly at %g rpm ---------\n', m_struct, arm.rpm);
fprintf('   1x: lateral %.4g N -> %.4g g,   axial %.4g N -> %.4g g\n', ...
    nom.F1_lat, nom.F1_lat/m_struct/g0, nom.F1_ax, nom.F1_ax/m_struct/g0);
fprintf('   2x: lateral %.4g N -> %.4g g,   axial %.4g N -> %.4g g\n\n', ...
    nom.F2_lat, nom.F2_lat/m_struct/g0, nom.F2_ax, nom.F2_ax/m_struct/g0);

fprintf('--- Speed sweep %g..%g rpm ---------------------------------------\n', rpm_sweep(1), rpm_sweep(end));
[mx, r] = max(max(sweep.peak(:,1:3), [], 1));  [~, k] = max(sweep.peak(:,r));
fprintf('   largest force  %.4g N   (%s) at %g rpm\n', mx, lbl{r}, rpm_sweep(k));
[mx, r] = max(max(sweep.peak(:,4:6), [], 1));  [~, k] = max(sweep.peak(:,r+3));
fprintf('   largest moment %.4g N*m (%s) at %g rpm\n\n', mx, lbl{r+3}, rpm_sweep(k));

fprintf('--- Momentum to cancel with the wheels -----------------------------\n');
fprintf('   arm momentum H = [%.4g %.4g %.4g] N*m*s\n', arm.H + 0);
fprintf('   -> paste into MomentumWheelLoads.m:  arm_momentum = [%.6g; %.6g; %.6g];\n', arm.H + 0);
fprintf('   -> each of two equal wheels must store %.4g N*m*s the other way\n\n', norm(arm.H)/2);

fprintf('--- Notes and assumptions ----------------------------------------\n');
for k = 1:numel(notes), fprintf('   * %s\n', notes{k}); end
fprintf('   * rigid arm: bending of the arm is not in the loads; check the\n');
fprintf('     Campbell diagram for arm modes near 1x or 2x.\n');
fprintf('   * speed is held constant at each point (quasi-steady); spin_accel\n');
fprintf('     adds the constant spin-up torque.\n');
fprintf('=====================================================================\n');

%% ========================================================================
%% 6. PLOTS
%% ========================================================================
if makePlots
    c = lines(4);
    % --- loads vs speed ---
    figure('Name','Arm loads vs speed','NumberTitle','off');
    for r = 1:6
        subplot(2,3,r); hold on; grid on;
        plot(sweep.rpm, abs(sweep.steady(:,r)), '-', 'Color', c(1,:), 'LineWidth', 1.2);
        plot(sweep.rpm, sweep.A1(:,r), '-', 'Color', c(2,:), 'LineWidth', 1.2);
        plot(sweep.rpm, sweep.A2(:,r), '-', 'Color', c(3,:), 'LineWidth', 1.2);
        plot(sweep.rpm, sweep.peak(:,r), 'k-', 'LineWidth', 1.8);
        yl = ylim; plot([arm.rpm arm.rpm], yl, 'k--');
        xlabel('arm speed [rpm]'); ylabel(lbl{r}); title(lbl{r});
        if r == 1, legend({'|steady|','1x','2x','PEAK'}, 'Location', 'northwest'); end
    end

    % --- Campbell diagram ---
    figure('Name','Arm Campbell diagram','NumberTitle','off'); hold on; grid on;
    for h = 1:3
        plot(sweep.rpm, h*sweep.rpm/60, 'Color', [0.4 0.4 0.4]);
        text(sweep.rpm(end), h*sweep.rpm(end)/60, sprintf(' %gx', h));
    end
    fspin = sweep.rpm/60;
    if ~isnan(arm_flap_freq)
        ff = sqrt(arm_flap_freq^2 + arm_flap_southwell*fspin.^2);
        plot(sweep.rpm, ff, '-', 'Color', c(1,:), 'LineWidth', 1.5);
        text(sweep.rpm(end), ff(end), ' arm flap');
    end
    if ~isnan(arm_edge_freq)
        fe = sqrt(arm_edge_freq^2 + arm_edge_southwell*fspin.^2);
        plot(sweep.rpm, fe, '-', 'Color', c(2,:), 'LineWidth', 1.5);
        text(sweep.rpm(end), fe(end), ' arm edge');
    end
    for m = 1:numel(structure_modes_Hz)
        plot(sweep.rpm([1 end]), [1 1]*structure_modes_Hz(m), 'r-');
        text(sweep.rpm(1), structure_modes_Hz(m), sprintf(' structure %.4g Hz', structure_modes_Hz(m)), 'Color', 'r');
    end
    yl = ylim; plot([arm.rpm arm.rpm], yl, 'k--');
    xlabel('arm speed [rpm]'); ylabel('frequency [Hz]');
    title('Campbell diagram: excitation lines vs. modes (crossings = resonance risk)');

    % --- one revolution at nominal speed ---
    figure('Name','Arm loads over one revolution','NumberTitle','off');
    LL = [nom.F; nom.M];
    for r = 1:6
        subplot(2,3,r); hold on; grid on;
        plot(nom.theta_deg, LL(r,:), 'k-', 'LineWidth', 1.4);
        xlim([0 360]); xlabel('arm angle [deg]'); ylabel(lbl{r}); title(lbl{r});
    end

    % --- acceleration vs frequency ---
    figure('Name','Rigid-body acceleration vs excitation frequency','NumberTitle','off');
    ttl = {'LATERAL (perpendicular to shaft axis)', 'AXIAL (along shaft axis)'};
    dat = {[accel.lat1_g, accel.lat2_g], [accel.ax1_g, accel.ax2_g]};
    for sp = 1:2
        subplot(1,2,sp); hold on; grid on;
        y = dat{sp};
        if max(y(:)) <= 0
            text(0.5, 0.5, {'no load in this direction'}, 'Units', 'normalized', 'HorizontalAlignment', 'center');
            xlabel('excitation frequency [Hz]'); ylabel('peak acceleration [g]'); title(ttl{sp});
            continue
        end
        ok1 = accel.f1 > 0 & y(:,1) > 0;  ok2 = accel.f2 > 0 & y(:,2) > 0;
        lg = {};
        if any(ok1), plot(accel.f1(ok1), y(ok1,1), '-', 'Color', c(2,:), 'LineWidth', 1.5); lg{end+1} = '1x'; end
        if any(ok2), plot(accel.f2(ok2), y(ok2,2), '-', 'Color', c(3,:), 'LineWidth', 1.5); lg{end+1} = '2x'; end
        set(gca, 'XScale', 'log', 'YScale', 'log');
        yl = ylim; fn = arm.rpm/60;
        plot([fn fn], yl, 'k--'); text(fn, yl(2), ' nominal 1x', 'VerticalAlignment', 'top');
        if ~isnan(accel_limit_g)
            plot([min(accel.f1(accel.f1>0)) max(accel.f2)], [1 1]*accel_limit_g, 'r-', 'LineWidth', 1.2);
        end
        xlabel('excitation frequency [Hz]'); ylabel('peak acceleration [g]');
        title(sprintf('%s, m = %g kg', ttl{sp}, m_struct));
        legend(lg, 'Location', 'northwest');
    end
end

%% ========================================================================
%% 7. SAVE AND SUMMARY
%% ========================================================================
summary = struct();
summary.what       = 'Loads applied by the rotating arm on the structure at operating speed';
summary.point_m    = P_ref(:).';
summary.rpm        = arm.rpm;
summary.F_peak_N   = nom.peak(1:3).';
summary.M_peak_Nm  = nom.peak(4:6).';
summary.F_steady_N = nom.harm.mean(1:3).' + 0;
summary.M_steady_Nm= nom.harm.mean(4:6).' + 0;
summary.F_1x_N     = abs(nom.harm.p(1:3,1)).';
summary.M_1x_Nm    = abs(nom.harm.p(4:6,1)).';
summary.F_2x_N     = abs(nom.harm.p(1:3,2)).';
summary.M_2x_Nm    = abs(nom.harm.p(4:6,2)).';
summary.H_Nms      = arm.H.' + 0;
summary.drag_T_Nm  = nom.drag_T;
results = struct('arm', arm, 'P_ref', P_ref, 'nominal', nom, 'sweep', sweep, ...
                 'accel', accel, 'root', root, 'summary', summary, 'notes', {notes});
if saveOutputs
    save('ArmLoads_results.mat', 'results');
    fid = fopen('ArmLoads_sweep.csv', 'w');
    fprintf(fid, 'rpm');
    for nm = {'steady','amp1x','amp2x','peak'}
        for r = 1:6, fprintf(fid, ',%s %s', nm{1}, strtok(lbl{r})); end
    end
    fprintf(fid, ',drag torque Nm,root tension N\n');
    for k = 1:nS
        fprintf(fid, '%g', sweep.rpm(k));
        fprintf(fid, ',%.6g', sweep.steady(k,:), sweep.A1(k,:), sweep.A2(k,:), sweep.peak(k,:));
        fprintf(fid, ',%.6g,%.6g\n', sweep.drag_T(k), sweep.root_F(k));
    end
    fclose(fid);
    fid = fopen('ArmLoads_summary.txt', 'w');
    print_summary(fid, summary, nom, root, bear_on, m_struct);
    fclose(fid);
    fprintf('Saved ArmLoads_results.mat, ArmLoads_sweep.csv, ArmLoads_summary.txt\n');
end
print_summary(1, summary, nom, root, bear_on, m_struct);

%% ========================================================================
%% LOCAL FUNCTIONS
%% ========================================================================
function out = arm_one_rev(arm, Om, alpha, wb, ab, P_ref, Tdrag, N)
% Loads applied BY the arm ON the structure over one revolution, from the
% rigid-body Newton-Euler equations, written in the structure frame.
%   structure frame rotates at constant wb about P_ref and accelerates at ab
%   arm spins at Om (rad/s) relative to it, with spin acceleration alpha
%   F, M : 3 x N   (M about P_ref)
    th = 2*pi*(0:N-1)/N;
    s = arm.s;  S = skew(s);  m = arm.m;
    w = wb + Om*s;                                 % arm angular velocity
    F = zeros(3,N);  M = zeros(3,N);
    for k = 1:N
        Q   = rodrigues(s, th(k)) * arm.B0;        % body -> structure
        I   = Q * arm.Ib * Q.';                    % inertia about CG
        rr  = Q * arm.c_b;                         % CG relative to hub
        r   = arm.p + rr;
        rd  = Om * cross3(s, rr);
        rdd = Om^2 * cross3(s, cross3(s, rr)) + alpha * cross3(s, rr);
        a   = ab + rdd + 2*cross3(wb, rd) + cross3(wb, cross3(wb, r - P_ref));
        Fk  = -m * a;                              % reaction of the arm's inertia
        Idot = Om * (S*I - I*S);
        H   = I * w;
        tau = Idot*w + I*(alpha*s) + cross3(wb, H);   % dH/dt (inertial) about CG
        Tair = -Tdrag * s;                         % air drag torque on the arm
        F(:,k) = Fk;
        M(:,k) = -tau + Tair + cross3(r - P_ref, Fk);
    end
    out = struct('F', F, 'M', M);
end

function h = harmonics_of(X, nh)
% Mean and harmonic phasors of each row of X sampled over one revolution.
% Row signal = mean + sum_k Re( p(:,k) * exp(i*k*theta) ).
    N  = size(X, 2);
    Y  = fft(X, [], 2);
    tol = 1e-9 * max(1e-12, max(abs(X(:))));
    h.mean = real(Y(:,1)) / N;
    h.mean(abs(h.mean) < tol) = 0;
    h.p = zeros(size(X,1), nh);
    for k = 1:nh
        if k+1 <= N/2, h.p(:,k) = 2*Y(:,k+1)/N; end
    end
    h.p(abs(h.p) < tol) = 0;
end

function X = tidy(X)
% Set numerical round-off (relative to the largest entry) to exactly zero.
    tol = 1e-9 * max(abs(X(:)));
    X(abs(X) < tol) = 0;
end

function [RA, RB, T] = bearing_split(F, M, s, xa, xb, axial_to)
% Split loads (F, M about P_ref) acting on a shaft along s into the
% radial reactions of two bearings at axial positions xa, xb (relative to
% P_ref).  Axial force goes to bearing axial_to; torque about the shaft
% goes to the drive (T).
    N   = size(F, 2);
    Fa  = s.' * F;           Fp = F - s*Fa;
    T   = s.' * M;           Mp = M - s*T;
    G   = cross3N(Mp, repmat(s, 1, N));          % = xa*RA + xb*RB
    RA  = (xb*Fp - G) / (xb - xa);
    RB  = (G - xa*Fp) / (xb - xa);
    if strcmp(axial_to, 'A'), RA = RA + s*Fa; else, RB = RB + s*Fa; end
end

function [f_lat, f_ax] = lateral_axial_peak(p, axial_dir)
% Peak axial and lateral force of a harmonic force phasor p.  Exact for
% rotating as well as straight-line oscillation.
    ax    = axial_dir(:) / norm(axial_dir);
    pa    = ax.' * p(:);
    f_ax  = abs(pa);
    pl    = p(:) - pa*ax;
    f_lat = norm([real(pl), imag(pl)]);
end

function print_summary(fid, S, nom, root, bear_on, m_struct)
% The block of numbers to quote.  fid = 1 prints to screen.
    L = '#####################################################################';
    fprintf(fid, '\n%s\n', L);
    fprintf(fid, '#   SUMMARY  -  ARM LOADS, THE NUMBERS TO QUOTE                     #\n');
    fprintf(fid, '%s\n', L);
    fprintf(fid, ' WHAT      : %s\n', S.what);
    fprintf(fid, ' WHERE     : at point [%g %g %g] m, structure frame [x y z]\n', S.point_m);
    fprintf(fid, ' SPEED     : %g rpm (%.4g Hz)\n', S.rpm, S.rpm/60);
    fprintf(fid, '\n');
    fprintf(fid, ' >>> PEAK OVER ONE REVOLUTION (design load) <<<                <- quote these\n');
    fprintf(fid, '     Fx = %10.4g N        Mx = %10.4g N*m\n', S.F_peak_N(1), S.M_peak_Nm(1));
    fprintf(fid, '     Fy = %10.4g N        My = %10.4g N*m\n', S.F_peak_N(2), S.M_peak_Nm(2));
    fprintf(fid, '     Fz = %10.4g N        Mz = %10.4g N*m\n', S.F_peak_N(3), S.M_peak_Nm(3));
    fprintf(fid, '\n');
    fprintf(fid, '     made of:      steady            1x amplitude      2x amplitude\n');
    nm = {'Fx','Fy','Fz'};
    for r = 1:3
        fprintf(fid, '     %s  %12.4g N    %12.4g N    %12.4g N\n', nm{r}, S.F_steady_N(r), S.F_1x_N(r), S.F_2x_N(r));
    end
    nm = {'Mx','My','Mz'};
    for r = 1:3
        fprintf(fid, '     %s  %12.4g N*m  %12.4g N*m  %12.4g N*m\n', nm{r}, S.M_steady_Nm(r), S.M_1x_Nm(r), S.M_2x_Nm(r));
    end
    fprintf(fid, '     1x at %.4g Hz, 2x at %.4g Hz\n', S.rpm/60, S.rpm/30);
    fprintf(fid, '\n');
    if bear_on
        fprintf(fid, ' BEARINGS (peak over one revolution):\n');
        fprintf(fid, '     bearing A: radial %10.4g N   axial %10.4g N\n', nom.RA_rad_peak, nom.RA_ax_peak);
        fprintf(fid, '     bearing B: radial %10.4g N   axial %10.4g N\n', nom.RB_rad_peak, nom.RB_ax_peak);
        fprintf(fid, '     drive torque about shaft: %10.4g N*m\n', mean(nom.Tmotor));
        fprintf(fid, '\n');
    end
    fprintf(fid, ' ACCELERATION of the %g kg assembly (1x): lateral %.4g g, axial %.4g g\n', ...
        m_struct, nom.F1_lat/m_struct/9.80665, nom.F1_ax/m_struct/9.80665);
    if root.on
        fprintf(fid, ' ROOT TENSION (internal, each half): %.4g N', root.F);
        if ~isnan(root.sigma_MPa), fprintf(fid, '  ->  %.4g MPa', root.sigma_MPa); end
        fprintf(fid, '\n');
    end
    fprintf(fid, ' DRAG TORQUE: %.4g N*m (%.4g W)\n', nom.drag_T, nom.drag_P);
    fprintf(fid, ' ARM MOMENTUM: [%g %g %g] N*m*s  (the wheels must cancel this)\n', S.H_Nms);
    fprintf(fid, '%s\n', L);
end

function R = rodrigues(axis, ang)
% Rotation matrix for a rotation of ang (rad) about unit vector axis.
    k = axis(:) / norm(axis);  K = skew(k);
    R = eye(3) + sin(ang)*K + (1 - cos(ang))*(K*K);
end

function S = skew(v)
% Cross-product matrix: skew(a)*b = cross(a,b).
    S = [   0   -v(3)  v(2);
          v(3)    0   -v(1);
         -v(2)  v(1)    0  ];
end

function c = cross3(a, b)
% 3-vector cross product (works for complex vectors too).
    c = [a(2)*b(3) - a(3)*b(2);
         a(3)*b(1) - a(1)*b(3);
         a(1)*b(2) - a(2)*b(1)];
end

function C = cross3N(A, B)
% Column-wise cross product of two 3 x N arrays.
    C = [A(2,:).*B(3,:) - A(3,:).*B(2,:);
         A(3,:).*B(1,:) - A(1,:).*B(3,:);
         A(1,:).*B(2,:) - A(2,:).*B(1,:)];
end

function n = vnorm(A)
% Column-wise Euclidean norm of a 3 x N array.
    n = sqrt(sum(A.^2, 1));
end

function v = need(v, name, n)
% Stop with a clear message if a required input was left empty or has the
% wrong shape.  n = number of values expected (1 = single number, 3 = x;y;z).
    if nargin < 3, n = 1; end
    if n == 3, shape = 'three numbers [x; y; z]'; else, shape = 'one number'; end
    if isempty(v) || any(isnan(v(:)))
        error('Input  %s  is empty. Enter %s for it in section 1 (USER INPUTS).', name, shape);
    end
    if numel(v) ~= n
        error('Input  %s  has %d value(s) but needs %s.', name, numel(v), shape);
    end
    v = v(:);
end

function v = empty_to_nan(v)
% Optional input: an empty [ ] means "not given".
    if isempty(v), v = NaN; end
end

function v = imbalance_to_SI(val, unit, order)
% Convert static (order 1, -> kg*m) or dynamic (order 2, -> kg*m^2)
% imbalance from common units to SI.
    u = lower(strrep(strrep(unit, ' ', ''), '.', '*'));
    switch order
        case 1
            switch u
                case 'g*cm',  f = 1e-3 * 1e-2;
                case 'g*mm',  f = 1e-3 * 1e-3;
                case 'kg*m',  f = 1;
                case 'oz*in', f = 0.0283495 * 0.0254;
                otherwise, error('Unknown static imbalance unit ''%s''', unit);
            end
        case 2
            switch u
                case 'g*cm^2',  f = 1e-3 * 1e-4;
                case 'g*mm^2',  f = 1e-3 * 1e-6;
                case 'kg*m^2',  f = 1;
                case 'oz*in^2', f = 0.0283495 * 0.0254^2;
                otherwise, error('Unknown dynamic imbalance unit ''%s''', unit);
            end
    end
    v = val * f;
end
