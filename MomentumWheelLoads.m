%% =========================================================================
%  MomentumWheelLoads.m
%  Loads and moments induced on a structure by TWO momentum storage wheels.
%
%  WHAT IT COMPUTES   (every load is the load applied BY the wheels ON the
%  structure, written in the structure frame and reduced to one reference
%  point that you choose)
%
%    A. STEADY, RIGID-BODY LOADS
%         - stored angular momentum of each wheel and of the pair
%         - gyroscopic torque  H x omega  when the mounting structure
%           rotates (e.g. when the wheels ride on a rotating arm)
%         - spin-up / spin-down reaction torque   -Izz * alpha
%         - inertial reaction to a quasi-static base acceleration  -m * a
%
%    B. HARMONIC (VIBRATION) DISTURBANCE LOADS
%         - 1x-speed radial force  from STATIC  imbalance :  Us * W^2
%         - 1x-speed radial moment from DYNAMIC imbalance :  Ud * W^2
%         - optional higher harmonics and axial force (if you have data)
%         - optional wheel structural modes (rocking mode with gyroscopic
%           nutation/precession split, axial mode, radial mode) that can
%           amplify the disturbance   [Liu et al. 2008, Eq. 2-6]
%         - lever arm of the rotor CG above the mounting plane, rotation
%           into the structure frame, transfer to the reference point and
%           combination of the two wheels (worst case and RSS)
%
%    C. SPEED SWEEP, CAMPBELL DIAGRAM and TIME HISTORIES at nominal speed
%
%  QUICK START
%    1. Edit section 1 "USER INPUTS" only.  Follow STEP 1 to STEP 5 and
%       fill in every blank:  [ ] takes one number,  [ ; ; ] takes x; y; z.
%       Units are written on every line.  Anything
%       marked optional can stay NaN; the script then uses a rigid /
%       conservative fallback and lists it under "Notes and assumptions".
%       If a required value is left empty the script stops and names it.
%    2. Press Run.
%    3. Read the printed report (Command Window) and the three figures.
%       Everything is also kept in the struct  results  and written to
%           MomentumWheelLoads_results.mat
%           MomentumWheelLoads_sweep.csv
%
%  WHICH OUTPUT IS WHICH
%    A. Steady loads     : constant forces/moments (momentum, gyroscopic,
%                          spin-up, quasi-static).  Zero if the shaft axis
%                          is fixed, the speed is constant and there is no
%                          base acceleration.
%    B. Harmonic loads   : vibration at the spin frequency (and harmonics)
%                          caused by imbalance.  Peak amplitudes per wheel
%                          and for the pair.  THIS IS THE MAIN RESULT.
%    C. Speed sweep      : how B changes with speed, and the Campbell
%                          diagram for resonance crossings.
%    D. Time history     : B written out in time with both wheels running.
%    E. Acceleration     : lateral and axial force divided by the mass of
%                          the assembly, plotted against frequency
%                          (rigid-body estimate, no flexible modes).
%    SUMMARY             : printed LAST and saved to
%                          MomentumWheelLoads_summary.txt.  These are the
%                          numbers to quote when someone asks for "the
%                          loads".  Every value says what it is, where it
%                          acts and in which units.
%    Add A and B for a combined design load.
%
%  WHAT YOU NEED FROM THE WHEEL DATASHEET
%       rotor mass, angular momentum (or spin inertia Izz), nominal speed,
%       static imbalance, dynamic imbalance, rotor CG height above the
%       mounting plane (from the outline drawing).  Transverse inertia and
%       structural frequencies are a bonus.
%  WHAT YOU NEED FROM YOUR OWN LAYOUT
%       mounting point of each wheel, spin axis direction of each wheel,
%       the reference point, and the angular rate of whatever the wheels
%       are bolted to.
%
%  REFERENCES
%    Liu K-C, Maghami P, Blaurock C (2008) "Reaction wheel disturbance
%      modeling, jitter analysis and validation tests for Solar Dynamics
%      Observatory", AIAA GNC Conf.   (Eq. 1 tonal model, Eq. 2-6 wheel
%      structural model, Eq. 7 x/y phasing, RSS combination of wheels)
%    Zhang Z, Aglietti G S, Ren W, Addari D (2014) "Microvibration analysis
%      of a cantilever configured reaction wheel assembly", Adv. Aircraft
%      and Spacecraft Science 1(4) 379-398.
%    Bialke B (1997) "A compilation of reaction wheel induced spacecraft
%      disturbances", AAS Guidance and Control Conf.
%
%  CONVENTIONS
%    - SI internally: N, N*m, kg, m, rad, rad/s.  Datasheet imbalance units
%      (g*cm, g*cm^2, g*mm, g*mm^2, oz*in ...) are converted for you.
%    - "Structure frame" = whatever frame you give positions and axes in.
%    - spin_axis = direction of the wheel's ANGULAR MOMENTUM vector, right-
%      hand rule.  Two wheels that spin the same way get the same vector.
%      A wheel spinning the opposite way gets the negated vector.
%    - Wheel local frame: z_w = spin_axis, x_w and y_w are two radial
%      directions built automatically.  The imbalance loads rotate in the
%      x_w-y_w plane at the spin frequency, y lagging x by 90 deg.
%    - Harmonic amplitudes are PEAK (zero-to-peak) values.
%    - The relative phase of the two wheels is unknown.  'worst' adds the
%      amplitudes, 'rss' root-sum-squares them (the SDO approach).
%
%  THE LAYOUT THE INPUT TEMPLATE DESCRIBES
%
%        MW-1                    ARM                     MW-2
%      +------+              (spins about x)           +------+
%      | wheel|   bearing    ====|=|====    bearing    | wheel|
%      |  <-- |------------------+-+--------------------| <--  |-----> x
%      +------+                  |                      +------+
%        x = -d           x = 0 (P_ref)                  x = +d
%
%     - the shaft axis is the structure x axis
%     - the arm spins about +x; the wheels spin about -x (opposite
%       sense) so that their stored momentum cancels the arm's
%     - the wheels sit at x = -d and x = +d, the arm at x = 0
%     - loads are reduced to the shaft centre.  Move P_ref to a bearing
%       location to get the loads at that bearing instead.
%     Only the wheels are modelled.  The arm enters only through
%     H_other (its momentum, for the momentum balance printout) and,
%     if the whole shaft is ever slewed, through omega_base.
%
%  MATLAB R2016b or newer.  (Under GNU Octave move the LOCAL FUNCTIONS
%  block to the top of the file, right after a line containing only "1;",
%  because Octave needs script-local functions defined before use.)
%% =========================================================================
clear; clc; close all;

%% ========================================================================
%% 1. USER INPUTS      <<< the only section you need to edit >>>
%% ========================================================================
%
%   STEP 1  reference point            (where do you want the loads?)
%   STEP 2  wheel 1  - required data   (datasheet + drawing)
%   STEP 3  wheel 2  - required data
%   STEP 4  optional data              (leave as is if you don't have it)
%   STEP 5  analysis settings
%
%   Fill in every blank.  The blank shows the shape of what goes in it:
%        [ ]        one number            e.g.  1.5
%        [ ; ; ]    three numbers x; y; z  e.g.  [-0.25; 0; 0]
%   The script stops and tells you which one is missing (or has the wrong
%   shape) if you leave a required value empty.
%   Vectors are [x; y; z] in the structure frame.  Units are in brackets.

%% ---- STEP 1: reference point -------------------------------------------------
%  Loads are reported at this point.  [0;0;0] = shaft centre where the arm
%  sits.  Put a bearing position here to get the loads at that bearing.
reference_point = [0; 0; 0];                  % [m]

%% ---- STEP 2: WHEEL 1, required --------------------------------------------------
MW1_name              = 'MW-1';
%  position and orientation (drawing)
MW1_position          = [ ; ; ];              % [m]  [x; y; z] centre of the mounting
                                              %      face, e.g. [-0.25; 0; 0] for a
                                              %      wheel 0.25 m left of centre
MW1_spin_axis         = [-1; 0; 0];           % [-]  direction of the wheel's angular
                                              %      momentum (right-hand rule).
                                              %      -x = opposite to the arm (+x)
MW1_cg_height         = [ ];                  % [m]  rotor CG height above the mounting
                                              %      face, along the spin axis.
                                              %      0 if MW1_position is the rotor CG
%  mass, momentum, speed (datasheet)
MW1_mass              = [ ];                  % [kg]     rotor (spinning part)
MW1_rpm               = [ ];                  % [rpm]    nominal operating speed
MW1_momentum          = [ ];                  % [N*m*s]  angular momentum at MW1_rpm
                                              %          (leave empty if you give Izz)
MW1_Izz               = [ ];                  % [kg*m^2] spin inertia
                                              %          (leave empty if you give momentum)
%  imbalance (datasheet).  Units:  static  'g*cm' 'g*mm' 'kg*m' 'oz*in'
%                                  dynamic 'g*cm^2' 'g*mm^2' 'kg*m^2' 'oz*in^2'
MW1_static_imbalance  = [ ];    MW1_static_unit  = 'g*cm';
MW1_dynamic_imbalance = [ ];    MW1_dynamic_unit = 'g*cm^2';

%% ---- STEP 3: WHEEL 2, required --------------------------------------------------
MW2_name              = 'MW-2';
MW2_position          = [ ; ; ];              % [m]  [x; y; z] e.g. [0.25; 0; 0]
MW2_spin_axis         = [-1; 0; 0];           % [-]  same as wheel 1 (spins the same way)
MW2_cg_height         = [ ];                  % [m]
MW2_mass              = [ ];                  % [kg]
MW2_rpm               = [ ];                  % [rpm]
MW2_momentum          = [ ];                  % [N*m*s]  or leave empty and give Izz
MW2_Izz               = [ ];                  % [kg*m^2] or leave empty and give momentum
MW2_static_imbalance  = [ ];    MW2_static_unit  = 'g*cm';
MW2_dynamic_imbalance = [ ];    MW2_dynamic_unit = 'g*cm^2';

%% ---- STEP 4: optional data.  Leave as NaN / 0 if unknown ----------------------
%  Wheel 1
MW1_Irr               = NaN;                  % [kg*m^2] transverse inertia about rotor
                                              %          CG.  Needed only for the rocking
                                              %          mode or the tilt option
MW1_cg_offset_mm      = NaN;                  % [mm]  rotor CG offset from the spin axis.
                                              %       Used instead of the static imbalance
                                              %       if that is left empty
MW1_tilt_deg          = NaN;                  % [deg] principal-axis tilt.  Used instead of
                                              %       the dynamic imbalance if that is left
                                              %       empty (needs Irr)
MW1_imbalance_phase   = 0;                    % [rad] angle between static and dynamic
                                              %       imbalance.  Unknown -> 0
MW1_harmonics         = [1 1 1];              % rows [h, force ratio, moment ratio].  No
                                              % test data -> keep [1 1 1].  A 2x line at
                                              % 20 % of the 1x -> [1 1 1; 2 0.2 0.2]
MW1_axial_coeff       = 0;                    % [kg*m] axial 1x force coefficient,
                                              %        F = coeff*omega^2.  0 if unknown
MW1_rock_freq         = NaN;  MW1_rock_damp   = 0.02;   % [Hz, -] rocking mode at 0 rpm
MW1_axial_freq        = NaN;  MW1_axial_damp  = 0.02;   % [Hz, -] axial mode
MW1_radial_freq       = NaN;  MW1_radial_damp = 0.02;   % [Hz, -] radial mode
                                              % NaN frequency = rigid wheel
%  Wheel 2
MW2_Irr               = NaN;
MW2_cg_offset_mm      = NaN;
MW2_tilt_deg          = NaN;
MW2_imbalance_phase   = 0;
MW2_harmonics         = [1 1 1];
MW2_axial_coeff       = 0;
MW2_rock_freq         = NaN;  MW2_rock_damp   = 0.02;
MW2_axial_freq        = NaN;  MW2_axial_damp  = 0.02;
MW2_radial_freq       = NaN;  MW2_radial_damp = 0.02;

%  Motion of the structure that carries the wheel bearings
base_rate             = [0; 0; 0];            % [rad/s] angular rate of the mounting
                                              %   structure.  0 when the shaft axis is
                                              %   fixed in space (the arm spinning ABOUT
                                              %   the shaft does not count).  Non-zero
                                              %   only if the whole shaft is slewed ->
                                              %   gyroscopic torque
spin_accel            = 0;                    % [rad/s^2] wheel spin-up (+) / spin-down
                                              %   (-) rate.  0 at steady speed
base_accel            = [0; 0; 0];            % [m/s^2] quasi-static acceleration of the
                                              %   structure.  0 on orbit; [0;0;9.81*8]
                                              %   for an 8 g case
arm_momentum          = [0; 0; 0];            % [N*m*s] momentum of the ARM, only to
                                              %   print the net momentum of arm + wheels.
                                              %   I_arm*omega_arm along +x.  0 to ignore

%% ---- STEP 5: analysis settings --------------------------------------------------
assembly_mass         = [ ];                  % [kg]  mass of the whole assembly the wheel
                                              %       loads shake (shaft + arm + wheels).
                                              %       For the acceleration plot: a = F/m
combine_method        = 'worst';              % 'worst' = add amplitudes (structural sizing)
                                              % 'rss'   = root-sum-square (jitter, SDO)
rpm_sweep             = 0:50:8000;            % [rpm] speed sweep applied to both wheels
structure_modes_Hz    = [];                   % [Hz]  known structure modes to draw on the
                                              %       Campbell diagram, e.g. [35 60 110]
accel_limit_g         = NaN;                  % [g]   acceleration limit line on the plot
time_length           = 0.05;                 % [s]   time-history length
sample_rate           = 20e3;                 % [Hz]  time-history sample rate
wheel2_phase          = 0;                    % [rad] phase of wheel 2 relative to wheel 1,
                                              %       time history only
makePlots             = true;                 % figures on/off
saveOutputs           = true;                 % write .mat, .csv and summary .txt on/off

%% ========================================================================
%%  (end of user inputs - nothing below needs editing)
%% ========================================================================

%% ---- collect the inputs into the internal wheel structures -------------------
wheel = make_wheel('MW1_', MW1_name, MW1_position, MW1_spin_axis, MW1_cg_height, ...
    MW1_mass, MW1_rpm, MW1_momentum, MW1_Izz, ...
    MW1_static_imbalance, MW1_static_unit, MW1_dynamic_imbalance, MW1_dynamic_unit, ...
    MW1_Irr, MW1_cg_offset_mm, MW1_tilt_deg, MW1_imbalance_phase, MW1_harmonics, MW1_axial_coeff, ...
    MW1_rock_freq, MW1_rock_damp, MW1_axial_freq, MW1_axial_damp, MW1_radial_freq, MW1_radial_damp);
wheel(2) = make_wheel('MW2_', MW2_name, MW2_position, MW2_spin_axis, MW2_cg_height, ...
    MW2_mass, MW2_rpm, MW2_momentum, MW2_Izz, ...
    MW2_static_imbalance, MW2_static_unit, MW2_dynamic_imbalance, MW2_dynamic_unit, ...
    MW2_Irr, MW2_cg_offset_mm, MW2_tilt_deg, MW2_imbalance_phase, MW2_harmonics, MW2_axial_coeff, ...
    MW2_rock_freq, MW2_rock_damp, MW2_axial_freq, MW2_axial_damp, MW2_radial_freq, MW2_radial_damp);
P_ref        = need(reference_point, 'reference_point', 3);
m_struct     = need(assembly_mass, 'assembly_mass', 1);
omega_base   = base_rate(:);   alpha_spin = spin_accel;   a_base = base_accel(:);
H_other      = arm_momentum(:);
sc_modes_Hz  = structure_modes_Hz;   t_end = time_length;   fs = sample_rate;
phase_wheel2 = wheel2_phase;

%% ========================================================================
%% 2. DERIVED QUANTITIES AND INPUT CHECKS
%% ========================================================================
nW = numel(wheel);
notes = {};
wheel_in = wheel;                       % user inputs; derived fields are added below
for i = 1:nW
    wi = wheel_in(i);
    assert(numel(wi.spin_axis) == 3 && norm(wi.spin_axis) > 0, ...
        '%s: spin_axis must be a non-zero 3-vector', wi.name);
    wi.spin_axis = wi.spin_axis(:) / norm(wi.spin_axis);
    wi.r_mount   = wi.r_mount(:);
    wi.omega_nom = wi.rpm_nom * 2*pi/60;                       % [rad/s]
    % spin inertia
    if isnan(wi.Izz)
        assert(~isnan(wi.H_nom) && wi.omega_nom > 0, ...
            '%s: give Izz, or H_nom together with a non-zero rpm_nom', wi.name);
        wi.Izz = wi.H_nom / wi.omega_nom;
        notes{end+1} = sprintf('%s: Izz derived from H_nom/omega_nom = %.4g kg*m^2', ...
            wi.name, wi.Izz); %#ok<SAGROW>
    end
    % static imbalance
    if isnan(wi.Us)
        assert(~isnan(wi.e_cg_mm), '%s: give Us or e_cg_mm', wi.name);
        wi.Us_SI = wi.mass * wi.e_cg_mm * 1e-3;
        notes{end+1} = sprintf('%s: Us derived from mass*e_cg = %.4g kg*m', ...
            wi.name, wi.Us_SI); %#ok<SAGROW>
    else
        wi.Us_SI = imbalance_to_SI(wi.Us, wi.Us_unit, 1);
    end
    % dynamic imbalance
    if isnan(wi.Ud)
        assert(~isnan(wi.tilt_deg) && ~isnan(wi.Irr), ...
            '%s: give Ud, or tilt_deg together with Irr', wi.name);
        wi.Ud_SI = abs(wi.Izz - wi.Irr) * wi.tilt_deg*pi/180;
        notes{end+1} = sprintf('%s: Ud derived from (Izz-Irr)*tilt = %.4g kg*m^2', ...
            wi.name, wi.Ud_SI); %#ok<SAGROW>
    else
        wi.Ud_SI = imbalance_to_SI(wi.Ud, wi.Ud_unit, 2);
    end
    % structural mode options
    if ~isnan(wi.f_rock) && isnan(wi.Irr)
        error('%s: the rocking-mode option needs Irr', wi.name);
    end
    if isnan(wi.f_rock), notes{end+1} = sprintf('%s: rocking mode not given -> rigid pass-through of moments', wi.name); end %#ok<SAGROW>
    if isnan(wi.f_ax),   notes{end+1} = sprintf('%s: axial mode not given -> rigid pass-through of axial force', wi.name); end %#ok<SAGROW>
    if isnan(wi.f_rad),  notes{end+1} = sprintf('%s: radial mode not given -> rigid pass-through of radial force', wi.name); end %#ok<SAGROW>
    if size(wi.harmonics,1) == 1
        notes{end+1} = sprintf('%s: fundamental (1x) only; higher harmonics and broadband noise need test data', wi.name); %#ok<SAGROW>
    end
    % local frame and derived positions
    wi.R      = wheel_frame(wi.spin_axis);        % columns x_w y_w z_w
    wi.r_cg   = wi.r_mount + wi.L_cg * wi.spin_axis;
    wi.H_vec  = wi.Izz * wi.omega_nom * wi.spin_axis;   % [N*m*s]
    if i == 1, wheel = wi; else, wheel(i) = wi; end     % rebuild with the new fields
end
clear wheel_in
if nW == 2 && dot(wheel(1).spin_axis, wheel(2).spin_axis) < 0
    notes{end+1} = 'The two spin axes point opposite ways: the wheels counter-rotate'; %#ok<SAGROW>
end

%% ========================================================================
%% 3. STEADY (RIGID-BODY) LOADS
%% ========================================================================
steady = struct();
steady.H_total = [0;0;0];
for i = 1:nW
    wi = wheel(i);
    s = struct();
    s.H           = wi.H_vec;                                   % stored momentum
    s.T_gyro      = cross3(wi.H_vec, omega_base);               % torque ON structure
    s.T_spin      = -wi.Izz * alpha_spin * wi.spin_axis;        % reaction to spin-up
    s.F_qs        = -wi.mass * a_base;                          % inertial reaction force
    s.M_qs        = cross3(wi.r_cg - P_ref, s.F_qs);            % its moment about P_ref
    s.M_total     = s.T_gyro + s.T_spin + s.M_qs;               % total steady moment
    s.F_total     = s.F_qs;                                     % total steady force
    steady.wheel(i) = s;
    steady.H_total  = steady.H_total + s.H;
end
steady.H_net    = steady.H_total + H_other;
steady.F_total  = steady.wheel(1).F_total;
steady.M_total  = steady.wheel(1).M_total;
for i = 2:nW
    steady.F_total = steady.F_total + steady.wheel(i).F_total;
    steady.M_total = steady.M_total + steady.wheel(i).M_total;
end

%% ========================================================================
%% 4. HARMONIC LOADS AT NOMINAL SPEED
%% ========================================================================
nominal = struct();
amp_all = [];                       % every (wheel, harmonic) amplitude, 6 x N
for i = 1:nW
    [Fb, Mb, fHz] = wheel_phasors(wheel(i), wheel(i).rpm_nom, P_ref);
    nominal.wheel(i).F_phasor = Fb;         % 3 x nHarm  complex, structure frame
    nominal.wheel(i).M_phasor = Mb;
    nominal.wheel(i).freq_Hz  = fHz;
    nominal.wheel(i).amp      = [abs(Fb); abs(Mb)];            % 6 x nHarm peak
    nominal.wheel(i).amp_combined = combine_amp(nominal.wheel(i).amp, combine_method);
    amp_all = [amp_all, nominal.wheel(i).amp]; %#ok<AGROW>
end
nominal.combined_worst = combine_amp(amp_all, 'worst');
nominal.combined_rss   = combine_amp(amp_all, 'rss');

%% ========================================================================
%% 5. SPEED SWEEP  (same rpm applied to both wheels)
%% ========================================================================
%  5a. amplitude of each load component vs speed
%  5b. lateral / axial force per harmonic vs excitation frequency, and the
%      rigid-body acceleration  a = F / m_struct
axial_dir = wheel(1).spin_axis;             % axial = shaft / spin axis
hlist = [];
for i = 1:nW, hlist = [hlist; wheel(i).harmonics(:,1)]; end %#ok<AGROW>
hlist = unique(hlist);  nH = numel(hlist);

nS = numel(rpm_sweep);
sweep = struct();
sweep.rpm = rpm_sweep(:);
for i = 1:nW, sweep.wheel(i).amp = zeros(nS, 6); end
sweep.combined = zeros(nS, 6);
accel = struct('h', hlist, 'axial_dir', axial_dir, 'm_struct', m_struct);
accel.freq_Hz = sweep.rpm * (hlist.' / 60);          % nS x nH
accel.F_lat   = zeros(nS, nH);                        % peak lateral force  [N]
accel.F_ax    = zeros(nS, nH);                        % peak axial force    [N]
for k = 1:nS
    amp_k = [];
    for i = 1:nW
        [Fb, Mb] = wheel_phasors(wheel(i), rpm_sweep(k), P_ref);
        a = [abs(Fb); abs(Mb)];
        sweep.wheel(i).amp(k,:) = combine_amp(a, combine_method).';
        amp_k = [amp_k, a]; %#ok<AGROW>
        for q = 1:size(wheel(i).harmonics, 1)
            j = find(hlist == wheel(i).harmonics(q,1), 1);
            [fl, fa] = lateral_axial_peak(Fb(:,q), axial_dir);
            accel.F_lat(k,j) = combine_amp([accel.F_lat(k,j), fl], combine_method);
            accel.F_ax(k,j)  = combine_amp([accel.F_ax(k,j),  fa], combine_method);
        end
    end
    sweep.combined(k,:) = combine_amp(amp_k, combine_method).';
end
g0 = 9.80665;
accel.a_lat_mps2 = accel.F_lat / m_struct;   accel.a_lat_g = accel.a_lat_mps2 / g0;
accel.a_ax_mps2  = accel.F_ax  / m_struct;   accel.a_ax_g  = accel.a_ax_mps2  / g0;
[~, kmax] = max(max(sweep.combined(:,1:3), [], 2));
sweep.rpm_maxForce = rpm_sweep(kmax);
[~, kmax] = max(max(sweep.combined(:,4:6), [], 2));
sweep.rpm_maxMoment = rpm_sweep(kmax);

% lateral / axial force and acceleration at nominal speed (for the summary)
nominal.F_lat = struct('worst', 0, 'rss', 0);
nominal.F_ax  = struct('worst', 0, 'rss', 0);
for i = 1:nW
    for q = 1:size(wheel(i).harmonics, 1)
        [fl, fa] = lateral_axial_peak(nominal.wheel(i).F_phasor(:,q), axial_dir);
        nominal.F_lat.worst = nominal.F_lat.worst + fl;
        nominal.F_ax.worst  = nominal.F_ax.worst  + fa;
        nominal.F_lat.rss   = sqrt(nominal.F_lat.rss^2 + fl^2);
        nominal.F_ax.rss    = sqrt(nominal.F_ax.rss^2  + fa^2);
    end
end
nominal.a_lat_g = struct('worst', nominal.F_lat.worst/m_struct/g0, 'rss', nominal.F_lat.rss/m_struct/g0);
nominal.a_ax_g  = struct('worst', nominal.F_ax.worst /m_struct/g0, 'rss', nominal.F_ax.rss /m_struct/g0);

%% ========================================================================
%% 6. TIME HISTORY AT NOMINAL SPEED
%% ========================================================================
t = (0:1/fs:t_end).';
timeh = struct('t', t);
timeh.total = zeros(numel(t), 6);
for i = 1:nW
    ph = 0; if i == 2, ph = phase_wheel2; end
    Fb = nominal.wheel(i).F_phasor;  Mb = nominal.wheel(i).M_phasor;
    om = 2*pi*nominal.wheel(i).freq_Hz;
    sig = zeros(numel(t), 6);
    for k = 1:numel(om)
        e = exp(1i*(om(k)*t + ph));                 % nt x 1
        sig = sig + real(e * [Fb(:,k).', Mb(:,k).']); % nt x 6
    end
    timeh.wheel(i).load = sig;
    timeh.total = timeh.total + sig;
end
timeh.peak = max(abs(timeh.total), [], 1);

%% ========================================================================
%% 7. PRINTED REPORT
%% ========================================================================
lbl = {'Fx [N]','Fy [N]','Fz [N]','Mx [N*m]','My [N*m]','Mz [N*m]'};
fprintf('=====================================================================\n');
fprintf('  MOMENTUM WHEEL INDUCED LOADS  -  loads applied by the wheels on the\n');
fprintf('  structure, structure frame, reduced to P_ref = [%g %g %g] m\n', P_ref);
fprintf('=====================================================================\n\n');

fprintf('--- Wheel data as used -------------------------------------------\n');
for i = 1:nW
    wi = wheel(i);
    fprintf('%s\n', wi.name);
    fprintf('   mount point      [%7.3f %7.3f %7.3f] m   rotor CG [%7.3f %7.3f %7.3f] m\n', wi.r_mount, wi.r_cg);
    fprintf('   spin axis        [%7.3f %7.3f %7.3f]     nominal %g rpm = %.4g Hz\n', wi.spin_axis, wi.rpm_nom, wi.omega_nom/(2*pi));
    fprintf('   mass %.4g kg   Izz %.4g kg*m^2   Irr %.4g kg*m^2   H = %.4g N*m*s\n', wi.mass, wi.Izz, wi.Irr, norm(wi.H_vec));
    fprintf('   static imbalance  %.4g kg*m  (%.4g g*cm)\n', wi.Us_SI, wi.Us_SI/1e-5);
    fprintf('   dynamic imbalance %.4g kg*m^2 (%.4g g*cm^2)\n', wi.Ud_SI, wi.Ud_SI/1e-7);
    fprintf('   1x radial force  Us*W^2 = %.4g N,  1x radial moment Ud*W^2 = %.4g N*m (at rotor CG)\n', ...
        wi.Us_SI*wi.omega_nom^2, wi.Ud_SI*wi.omega_nom^2);
    if ~isnan(wi.f_rock)
        [fn, fp] = whirl_freqs(wi, wi.omega_nom);
        fprintf('   rocking mode %.4g Hz at 0 rpm -> nutation %.4g Hz, precession %.4g Hz at nominal\n', wi.f_rock, fn, fp);
    end
end
fprintf('\n');

fprintf('--- A. Steady (rigid-body) loads ---------------------------------\n');
fprintf('   base rate omega = [%g %g %g] rad/s,  spin accel = %g rad/s^2,  base accel = [%g %g %g] m/s^2\n', ...
    omega_base, alpha_spin, a_base);
for i = 1:nW
    s = steady.wheel(i);
    fprintf('%s\n', wheel(i).name);
    fprintf('   momentum H        [%10.4g %10.4g %10.4g] N*m*s\n', s.H + 0);
    fprintf('   gyroscopic torque [%10.4g %10.4g %10.4g] N*m   (H x omega_base)\n', s.T_gyro + 0);
    fprintf('   spin-up torque    [%10.4g %10.4g %10.4g] N*m   (-Izz*alpha)\n', s.T_spin + 0);
    fprintf('   quasi-static F    [%10.4g %10.4g %10.4g] N     (-m*a)\n', s.F_qs + 0);
    fprintf('   quasi-static M    [%10.4g %10.4g %10.4g] N*m   (about P_ref)\n', s.M_qs + 0);
end
fprintf('BOTH WHEELS\n');
fprintf('   total momentum    [%10.4g %10.4g %10.4g] N*m*s  |H| = %.4g\n', steady.H_total + 0, norm(steady.H_total));
if any(H_other)
fprintf('   net with H_other  [%10.4g %10.4g %10.4g] N*m*s  |H| = %.4g\n', steady.H_net + 0, norm(steady.H_net));
end
fprintf('   steady force      [%10.4g %10.4g %10.4g] N\n', steady.F_total + 0);   % '+ 0' turns -0 into 0
fprintf('   steady moment     [%10.4g %10.4g %10.4g] N*m\n', steady.M_total + 0);
fprintf('\n');

fprintf('--- B. Harmonic loads at nominal speed (peak amplitudes) ---------\n');
fprintf('%-26s', ' ');
for i = 1:nW, fprintf('%12s', wheel(i).name); end
fprintf('%14s%14s\n', 'BOTH worst', 'BOTH rss');
for r = 1:6
    fprintf('%-26s', lbl{r});
    for i = 1:nW, fprintf('%12.4g', nominal.wheel(i).amp_combined(r)); end
    fprintf('%14.4g%14.4g\n', nominal.combined_worst(r), nominal.combined_rss(r));
end
fprintf('   (per-wheel column combines its harmonics with ''%s''; frequencies:', combine_method);
for i = 1:nW, fprintf(' %s %s Hz;', wheel(i).name, mat2str(round(nominal.wheel(i).freq_Hz*100)/100)); end
fprintf(')\n\n');

fprintf('--- C. Speed sweep %g..%g rpm, both wheels, ''%s'' combination ---\n', rpm_sweep(1), rpm_sweep(end), combine_method);
[mx, r] = max(max(sweep.combined(:,1:3), [], 1));
fprintf('   largest force  %.4g N   (%s) at %g rpm\n', mx, lbl{r}, sweep.rpm_maxForce);
[mx, r] = max(max(sweep.combined(:,4:6), [], 1));
fprintf('   largest moment %.4g N*m (%s) at %g rpm\n', mx, lbl{r+3}, sweep.rpm_maxMoment);
fprintf('\n');

fprintf('--- D. Time history at nominal speed, phase of wheel 2 = %g rad -----\n', phase_wheel2);
fprintf('   peak |total| over %g s: ', t_end);
for r = 1:6, fprintf('%s %.4g  ', strtok(lbl{r}), timeh.peak(r)); end
fprintf('\n\n');

fprintf('--- E. Rigid-body acceleration of the %g kg assembly at nominal speed ---\n', m_struct);
fprintf('   lateral (perp. to shaft axis): force %.4g N -> %.4g g  (worst), %.4g N -> %.4g g (rss)\n', ...
    nominal.F_lat.worst, nominal.a_lat_g.worst, nominal.F_lat.rss, nominal.a_lat_g.rss);
fprintf('   axial   (along shaft axis)   : force %.4g N -> %.4g g  (worst), %.4g N -> %.4g g (rss)\n', ...
    nominal.F_ax.worst, nominal.a_ax_g.worst, nominal.F_ax.rss, nominal.a_ax_g.rss);
if nominal.F_ax.worst == 0
fprintf('   (axial is zero because imbalance only produces radial loads; set C_ax if the vendor gives an axial force)\n');
end
fprintf('\n');

fprintf('--- Notes and assumptions ----------------------------------------\n');
for k = 1:numel(notes), fprintf('   * %s\n', notes{k}); end
fprintf('   * Harmonic loads scale with speed^2; the 1x force and moment come from\n');
fprintf('     the datasheet imbalance limits and are therefore upper bounds for a\n');
fprintf('     wheel that meets its specification.\n');
fprintf('   * Steady and harmonic loads are reported separately; add them for a\n');
fprintf('     combined design load.\n');
fprintf('=====================================================================\n');

%% ========================================================================
%% 8. PLOTS
%% ========================================================================
if makePlots
    cols = lines(nW+1);
    % --- amplitude vs speed ---
    figure('Name','Harmonic load amplitude vs wheel speed','NumberTitle','off');
    for r = 1:6
        subplot(2,3,r); hold on; grid on;
        for i = 1:nW
            plot(sweep.rpm, sweep.wheel(i).amp(:,r), '-', 'Color', cols(i,:), 'LineWidth', 1.2);
        end
        plot(sweep.rpm, sweep.combined(:,r), 'k-', 'LineWidth', 1.8);
        yl = ylim;
        for i = 1:nW
            plot([wheel(i).rpm_nom wheel(i).rpm_nom], yl, '--', 'Color', cols(i,:));
        end
        xlabel('wheel speed [rpm]'); ylabel(lbl{r}); title(lbl{r});
        if r == 1
            lg = cell(1,nW+1); for i = 1:nW, lg{i} = wheel(i).name; end
            lg{end} = ['both (' combine_method ')'];
            legend(lg, 'Location', 'northwest');
        end
    end

    % --- Campbell diagram ---
    figure('Name','Campbell diagram','NumberTitle','off'); hold on; grid on;
    hmax = max(wheel(1).harmonics(:,1));
    for i = 1:nW, hmax = max(hmax, max(wheel(i).harmonics(:,1))); end
    for h = 1:max(hmax,3)
        plot(sweep.rpm, h*sweep.rpm/60, 'Color', [0.4 0.4 0.4]);
        text(sweep.rpm(end), h*sweep.rpm(end)/60, sprintf(' %gx', h));
    end
    for i = 1:nW
        if ~isnan(wheel(i).f_rock)
            fn = zeros(nS,1); fp = zeros(nS,1);
            for k = 1:nS, [fn(k), fp(k)] = whirl_freqs(wheel(i), rpm_sweep(k)*2*pi/60); end
            plot(sweep.rpm, fn, '-', 'Color', cols(i,:), 'LineWidth', 1.5);
            plot(sweep.rpm, fp, '--', 'Color', cols(i,:), 'LineWidth', 1.5);
            text(sweep.rpm(end), fn(end), [' ' wheel(i).name ' nutation']);
            text(sweep.rpm(end), fp(end), [' ' wheel(i).name ' precession']);
        end
        if ~isnan(wheel(i).f_ax)
            plot(sweep.rpm([1 end]), [1 1]*wheel(i).f_ax, ':', 'Color', cols(i,:));
            text(sweep.rpm(end), wheel(i).f_ax, [' ' wheel(i).name ' axial']);
        end
    end
    for m = 1:numel(sc_modes_Hz)
        plot(sweep.rpm([1 end]), [1 1]*sc_modes_Hz(m), 'r-');
        text(sweep.rpm(1), sc_modes_Hz(m), sprintf(' structure mode %.4g Hz', sc_modes_Hz(m)), 'Color', 'r');
    end
    yl = ylim;
    for i = 1:nW
        plot([wheel(i).rpm_nom wheel(i).rpm_nom], yl, '--', 'Color', cols(i,:));
    end
    xlabel('wheel speed [rpm]'); ylabel('frequency [Hz]');
    title('Campbell diagram: excitation lines vs. modes (crossings = resonance risk)');

    % --- acceleration vs frequency ---
    figure('Name','Rigid-body acceleration vs excitation frequency','NumberTitle','off');
    ttl = {'LATERAL (perpendicular to shaft axis)', 'AXIAL (along shaft axis)'};
    dat = {accel.a_lat_g, accel.a_ax_g};
    for sp = 1:2
        subplot(1,2,sp); hold on; grid on;
        if max(dat{sp}(:)) <= 0
            text(0.5, 0.5, {'no load in this direction', '(imbalance gives radial loads only;', 'set C_ax for a vendor axial force)'}, ...
                'Units', 'normalized', 'HorizontalAlignment', 'center');
            xlabel('excitation frequency [Hz]'); ylabel('peak acceleration [g]'); title(ttl{sp});
            continue
        end
        for j = 1:nH
            f = accel.freq_Hz(:,j);  y = dat{sp}(:,j);
            plot(f(f>0), y(f>0), '-', 'LineWidth', 1.5, 'Color', cols(min(j,nW+1),:));
        end
        set(gca, 'XScale', 'log', 'YScale', 'log');
        yl = ylim;
        for i = 1:nW
            fn = wheel(i).rpm_nom/60;
            plot([fn fn], yl, '--', 'Color', [0.3 0.3 0.3]);
            text(fn, yl(2), sprintf(' %s nominal', wheel(i).name), 'VerticalAlignment', 'top');
        end
        if ~isnan(accel_limit_g)
            plot([min(accel.freq_Hz(accel.freq_Hz>0)) max(accel.freq_Hz(:))], [1 1]*accel_limit_g, 'r-', 'LineWidth', 1.2);
            text(max(accel.freq_Hz(:)), accel_limit_g, ' limit', 'Color', 'r');
        end
        xlabel('excitation frequency [Hz]'); ylabel('peak acceleration [g]');
        title(sprintf('%s\\n%s combination, m = %g kg', ttl{sp}, combine_method, m_struct));
        if nH > 1
            lg = cell(1,nH); for j = 1:nH, lg{j} = sprintf('%gx harmonic', hlist(j)); end
            legend(lg, 'Location', 'northwest');
        end
    end

    % --- time history ---
    figure('Name','Time history at nominal speed (both wheels)','NumberTitle','off');
    for r = 1:6
        subplot(2,3,r); hold on; grid on;
        for i = 1:nW
            plot(t, timeh.wheel(i).load(:,r), '-', 'Color', cols(i,:));
        end
        plot(t, timeh.total(:,r), 'k-', 'LineWidth', 1.4);
        xlabel('time [s]'); ylabel(lbl{r}); title(lbl{r});
    end
end

%% ========================================================================
%% 9. SAVE
%% ========================================================================
summary = struct();
summary.what            = 'Peak (0-to-peak) sinusoidal loads applied by the two momentum wheels on the structure at nominal speed';
summary.point_m         = P_ref(:).';
summary.frame           = 'structure frame [x y z]';
summary.frequency_Hz    = [wheel.rpm_nom] / 60;
summary.F_both_worst_N  = nominal.combined_worst(1:3).';
summary.M_both_worst_Nm = nominal.combined_worst(4:6).';
summary.F_both_rss_N    = nominal.combined_rss(1:3).';
summary.M_both_rss_Nm   = nominal.combined_rss(4:6).';
summary.F_lateral_worst_N = nominal.F_lat.worst;   summary.F_axial_worst_N = nominal.F_ax.worst;
summary.a_lateral_worst_g = nominal.a_lat_g.worst; summary.a_axial_worst_g = nominal.a_ax_g.worst;
summary.F_steady_N      = (steady.F_total + 0).';
summary.M_steady_Nm     = (steady.M_total + 0).';
for i = 1:nW
    summary.wheel(i).name = wheel(i).name;
    summary.wheel(i).F_N  = nominal.wheel(i).amp_combined(1:3).';
    summary.wheel(i).M_Nm = nominal.wheel(i).amp_combined(4:6).';
end
results = struct('P_ref', P_ref, 'wheel', wheel, 'steady', steady, ...
                 'nominal', nominal, 'sweep', sweep, 'accel', accel, ...
                 'time', timeh, 'summary', summary, ...
                 'combine_method', combine_method, 'notes', {notes});
if saveOutputs
    save('MomentumWheelLoads_results.mat', 'results');
    fid = fopen('MomentumWheelLoads_summary.txt', 'w');
    print_summary(fid, summary, wheel, combine_method, m_struct, axial_dir);
    fclose(fid);
    fid = fopen('MomentumWheelLoads_sweep.csv', 'w');
    fprintf(fid, 'rpm');
    for i = 1:nW
        for r = 1:6, fprintf(fid, ',%s %s', wheel(i).name, strtok(lbl{r})); end
    end
    for r = 1:6, fprintf(fid, ',both_%s %s', combine_method, strtok(lbl{r})); end
    fprintf(fid, '\n');
    for k = 1:nS
        fprintf(fid, '%g', sweep.rpm(k));
        for i = 1:nW, fprintf(fid, ',%.6g', sweep.wheel(i).amp(k,:)); end
        fprintf(fid, ',%.6g', sweep.combined(k,:));
        fprintf(fid, '\n');
    end
    fclose(fid);
    fprintf('Saved MomentumWheelLoads_results.mat, MomentumWheelLoads_sweep.csv, MomentumWheelLoads_summary.txt\n');
end

%% ========================================================================
%% 10. SUMMARY  -  printed last so it is the first thing you see on screen
%% ========================================================================
print_summary(1, summary, wheel, combine_method, m_struct, axial_dir);

%% ========================================================================
%% LOCAL FUNCTIONS
%% ========================================================================
function [Fb, Mb, fHz] = wheel_phasors(w, rpm, P_ref)
% Harmonic disturbance phasors of one wheel at a given speed, in the
% structure frame, reduced to P_ref.   Real load = Re( phasor * e^{i w t} ).
%   Fb, Mb : 3 x nHarm complex,  fHz : 1 x nHarm excitation frequencies
    Om  = rpm * 2*pi/60;
    nh  = size(w.harmonics, 1);
    Fb  = zeros(3, nh); Mb = zeros(3, nh); fHz = zeros(1, nh);
    fwd = [1; -1i; 0];               % forward-rotating vector in x_w-y_w plane
    zw  = [0; 0; 1];
    for k = 1:nh
        h  = w.harmonics(k,1);
        Cf = w.harmonics(k,2) * w.Us_SI;
        Cm = w.harmonics(k,3) * w.Ud_SI;
        om = h * Om;  fHz(k) = om/(2*pi);
        % excitation at the rotor CG, wheel frame   [Liu et al. Eq. 1 & 7]
        F_rad = Cf * Om^2 * fwd;
        M_rad = Cm * Om^2 * exp(1i*w.phase_Ud) * fwd;
        F_ax  = (h == 1) * w.C_ax * Om^2 * zw;
        % wheel structural filtering (transmissibility to the mount)
        F_rad = F_rad * tf_translational(om, w.mass, w.f_rad, w.zeta_rad);
        F_ax  = F_ax  * tf_translational(om, w.mass, w.f_ax,  w.zeta_ax);
        M_rad = M_rad * tf_rocking(om, Om, w.Irr, w.Izz, w.f_rock, w.zeta_rock);
        % reduce to the mounting plane (rotor CG sits L_cg above it)
        F_w = F_rad + F_ax;
        M_w = M_rad + cross3(w.L_cg * zw, F_w);
        % rotate to structure frame and move to the reference point
        Fb(:,k) = w.R * F_w;
        Mb(:,k) = w.R * M_w + cross3(w.r_mount - P_ref, Fb(:,k));
    end
end

function H = tf_translational(om, m, fn, zeta)
% Transmissibility of a 1-DOF mass on a spring-damper:  F_mount / F_applied
    if isnan(fn), H = 1; return; end
    k = m * (2*pi*fn)^2;
    c = 2 * zeta * m * (2*pi*fn);
    H = (k + 1i*c*om) / (k - m*om^2 + 1i*c*om);
end

function H = tf_rocking(om, Om, Irr, Izz, fn, zeta)
% Transmissibility of the rotor rocking mode with gyroscopic coupling for a
% forward-whirling excitation at frequency om while spinning at Om.
%   Irr*th'' + c*th' - i*Izz*Om*th' + k*th = tau   with th = thx + i*thy
%   [Liu et al. 2008 Eq. 2-4, written in complex form]
% The forward (nutation) root rises with speed, the backward (precession)
% root falls, exactly as in Liu et al. Fig. 5.
    if isnan(fn), H = 1; return; end
    k = Irr * (2*pi*fn)^2;
    c = 2 * zeta * Irr * (2*pi*fn);
    H = (k + 1i*c*om) / (k - Irr*om^2 + Izz*Om*om + 1i*c*om);
end

function [f_nut, f_prec] = whirl_freqs(w, Om)
% Nutation (forward) and precession (backward) whirl frequencies in Hz of
% the rocking mode at spin speed Om [rad/s].  Undamped.
    k = w.Irr * (2*pi*w.f_rock)^2;
    d = sqrt((Om*w.Izz)^2 + 4*k*w.Irr);
    f_nut  = ( Om*w.Izz + d) / (2*w.Irr) / (2*pi);
    f_prec = (-Om*w.Izz + d) / (2*w.Irr) / (2*pi);
end

function [f_lat, f_ax] = lateral_axial_peak(p, axial_dir)
% Peak axial and peak lateral force of a 3-vector phasor p (load =
% Re(p e^{iwt})).  Lateral peak = max over time of |lateral force vector|
% = largest singular value of [Re(p_lat) Im(p_lat)], which is exact for a
% rotating (whirling) load as well as for a straight-line oscillation.
    ax   = axial_dir(:) / norm(axial_dir);
    pa   = ax.' * p(:);
    f_ax = abs(pa);
    pl   = p(:) - pa * ax;
    f_lat = norm([real(pl), imag(pl)]);
end

function print_summary(fid, S, wheel, method, m_struct, axial_dir)
% The block of numbers to quote.  fid = 1 prints to screen.
    nW = numel(wheel);
    L = '#####################################################################';
    fprintf(fid, '\n%s\n', L);
    fprintf(fid, '#   SUMMARY  -  THE NUMBERS TO QUOTE                                #\n');
    fprintf(fid, '%s\n', L);
    fprintf(fid, ' WHAT      : %s\n', S.what);
    fprintf(fid, ' WHERE     : at point [%g %g %g] m, %s\n', S.point_m, S.frame);
    fprintf(fid, ' SPEED     :');
    for i = 1:nW, fprintf(fid, ' %s %g rpm (%.4g Hz)', wheel(i).name, wheel(i).rpm_nom, wheel(i).rpm_nom/60); end
    fprintf(fid, '\n');
    fprintf(fid, ' SIGN/TYPE : peak amplitude of a sinusoid (use +/- this value)\n');
    fprintf(fid, '\n');
    fprintf(fid, ' >>> BOTH WHEELS TOGETHER, WORST CASE (amplitudes added) <<<   <- quote these\n');
    fprintf(fid, '     Fx = %10.4g N        Mx = %10.4g N*m\n', S.F_both_worst_N(1), S.M_both_worst_Nm(1));
    fprintf(fid, '     Fy = %10.4g N        My = %10.4g N*m\n', S.F_both_worst_N(2), S.M_both_worst_Nm(2));
    fprintf(fid, '     Fz = %10.4g N        Mz = %10.4g N*m\n', S.F_both_worst_N(3), S.M_both_worst_Nm(3));
    fprintf(fid, '\n');
    fprintf(fid, '     both wheels, RSS (random phase, less conservative):\n');
    fprintf(fid, '     Fx = %10.4g N        Mx = %10.4g N*m\n', S.F_both_rss_N(1), S.M_both_rss_Nm(1));
    fprintf(fid, '     Fy = %10.4g N        My = %10.4g N*m\n', S.F_both_rss_N(2), S.M_both_rss_Nm(2));
    fprintf(fid, '     Fz = %10.4g N        Mz = %10.4g N*m\n', S.F_both_rss_N(3), S.M_both_rss_Nm(3));
    fprintf(fid, '\n');
    fprintf(fid, ' LATERAL / AXIAL  (shaft axis = [%g %g %g]), worst case, both wheels:\n', axial_dir);
    fprintf(fid, '     lateral force = %10.4g N   ->  %.4g g on the %g kg assembly\n', S.F_lateral_worst_N, S.a_lateral_worst_g, m_struct);
    fprintf(fid, '     axial   force = %10.4g N   ->  %.4g g on the %g kg assembly\n', S.F_axial_worst_N, S.a_axial_worst_g, m_struct);
    fprintf(fid, '\n');
    fprintf(fid, ' EACH WHEEL ALONE (its harmonics combined with ''%s''):\n', method);
    for i = 1:nW
        fprintf(fid, '     %-6s F = [%10.4g %10.4g %10.4g] N   M = [%10.4g %10.4g %10.4g] N*m\n', ...
            S.wheel(i).name, S.wheel(i).F_N, S.wheel(i).M_Nm);
    end
    fprintf(fid, '\n');
    fprintf(fid, ' STEADY (constant) LOADS, add to the above:\n');
    fprintf(fid, '     F = [%10.4g %10.4g %10.4g] N   M = [%10.4g %10.4g %10.4g] N*m', S.F_steady_N, S.M_steady_Nm);
    if ~any(S.F_steady_N) && ~any(S.M_steady_Nm)
        fprintf(fid, '   (all zero: fixed shaft axis, constant speed, no base acceleration)');
    end
    fprintf(fid, '\n%s\n', L);
end

function w = make_wheel(pre, name, position, spin_axis, cg_height, mass, rpm, ...
    momentum, Izz, Us, Us_unit, Ud, Ud_unit, Irr, cg_offset_mm, tilt_deg, ...
    imbalance_phase, harmonics, axial_coeff, f_rock, z_rock, f_ax, z_ax, f_rad, z_rad)
% Pack one wheel's inputs into the internal structure, checking that every
% required value has been entered.  pre is 'MW1_' or 'MW2_' for messages.
    w = struct();
    w.name      = name;
    w.r_mount   = need(position,  [pre 'position'],  3);
    w.spin_axis = need(spin_axis, [pre 'spin_axis'], 3);
    w.L_cg      = need(cg_height, [pre 'cg_height'], 1);
    w.mass      = need(mass,      [pre 'mass'],      1);
    w.rpm_nom   = need(rpm,       [pre 'rpm'],       1);
    w.H_nom     = empty_to_nan(momentum);
    w.Izz       = empty_to_nan(Izz);
    if isnan(w.H_nom) && isnan(w.Izz)
        error('Enter %smomentum or %sIzz in section 1 (one of the two is needed).', pre, pre);
    end
    w.Us        = empty_to_nan(Us);   w.Us_unit = Us_unit;
    w.Ud        = empty_to_nan(Ud);   w.Ud_unit = Ud_unit;
    w.Irr       = empty_to_nan(Irr);
    w.e_cg_mm   = empty_to_nan(cg_offset_mm);
    w.tilt_deg  = empty_to_nan(tilt_deg);
    if isnan(w.Us) && isnan(w.e_cg_mm)
        error('Enter %sstatic_imbalance (or %scg_offset_mm) in section 1.', pre, pre);
    end
    if isnan(w.Ud) && isnan(w.tilt_deg)
        error('Enter %sdynamic_imbalance (or %stilt_deg together with %sIrr) in section 1.', pre, pre, pre);
    end
    w.phase_Ud  = imbalance_phase;
    w.harmonics = harmonics;
    w.C_ax      = axial_coeff;
    w.f_rock = f_rock;   w.zeta_rock = z_rock;
    w.f_ax   = f_ax;     w.zeta_ax   = z_ax;
    w.f_rad  = f_rad;    w.zeta_rad  = z_rad;
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

function a = combine_amp(A, method)
% Combine the columns of a 6 x N amplitude matrix into one 6 x 1 amplitude.
    switch lower(method)
        case 'worst', a = sum(A, 2);
        case 'rss',   a = sqrt(sum(A.^2, 2));
        otherwise,    error('combine_method must be ''worst'' or ''rss''');
    end
end

function R = wheel_frame(z)
% Right-handed frame [x_w y_w z_w] (columns) with z_w = spin axis.
    z = z(:) / norm(z);
    [~, imin] = min(abs(z));              % axis least aligned with z
    helper = zeros(3,1); helper(imin) = 1;
    x = cross3(helper, z); x = x / norm(x);
    y = cross3(z, x);
    R = [x y z];
end

function c = cross3(a, b)
% 3-vector cross product that also accepts complex phasor vectors.
    c = [a(2)*b(3) - a(3)*b(2);
         a(3)*b(1) - a(1)*b(3);
         a(1)*b(2) - a(2)*b(1)];
end

function v = imbalance_to_SI(val, unit, order)
% Convert static (order 1, -> kg*m) or dynamic (order 2, -> kg*m^2)
% imbalance from common datasheet units to SI.
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
