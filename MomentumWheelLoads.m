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
%  HOW TO USE
%    1. Fill in section 1 "USER INPUTS".  Units are written on every line.
%       Where a line says OPTIONAL you may leave NaN; the script then uses
%       a conservative / rigid fallback and says so in the report.
%    2. Run the script.
%    3. Read the printed report and the figures.  Everything is also kept
%       in the struct  results  and written to
%           MomentumWheelLoads_results.mat
%           MomentumWheelLoads_sweep.csv
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
%  MATLAB R2016b or newer.  (Under GNU Octave move the LOCAL FUNCTIONS
%  block to the top of the file, right after a line containing only "1;",
%  because Octave needs script-local functions defined before use.)
%% =========================================================================
clear; clc; close all;

%% ========================================================================
%% 1. USER INPUTS
%% ========================================================================
%  >>>>>  ALL NUMBERS BELOW ARE EXAMPLE VALUES.  REPLACE WITH YOUR OWN.  <<<<<

% ---- 1a. Reference point --------------------------------------------------
%  Loads are reduced to this point (structure frame, metres).  Typical
%  choices: the structure CG, or the FE node of the wheel bracket.
P_ref = [0; 0; 0];                          % [m]

% ---- 1b. Motion of the structure the wheels are bolted to --------------------
%  If the wheels ride on the arm, put the ARM's angular rate here.  Zero if
%  the mounting structure is not rotating.  Structure frame.
omega_base = [0; 0; 0];                     % [rad/s]   e.g. [0;0;-0.10]
alpha_spin = 0;                             % [rad/s^2] wheel spin-up (+) or
                                            %           spin-down (-) rate.
                                            %           0 at steady speed.
a_base     = [0; 0; 0];                     % [m/s^2]   quasi-static base
                                            %           acceleration. 0 on orbit;
                                            %           e.g. [0;0;9.81*8] for an
                                            %           8 g launch case.
H_other    = [0; 0; 0];                     % [N*m*s]   OPTIONAL momentum of
                                            %           anything else (the arm)
                                            %           to show the net with the
                                            %           wheels. 0 to ignore.

% ---- 1c. WHEEL 1 -----------------------------------------------------------------
w = struct();
w.name       = 'MW-1';
% --- geometry (structure frame) ---
w.r_mount    = [ 0.250; 0.150; 0.050];      % [m]  centre of the mounting plane
w.spin_axis  = [ 0; 0; 1];                  % [-]  angular-momentum direction
w.L_cg       = 0.030;                       % [m]  rotor CG height above the
                                            %      mounting plane, along spin_axis
% --- mass and inertia ---
w.mass       = 1.50;                        % [kg]      rotor (spinning part)
w.H_nom      = 1.00;                        % [N*m*s]   angular momentum at
                                            %           rpm_nom (datasheet).
                                            %           NaN if you give Izz.
w.Izz        = NaN;                         % [kg*m^2]  spin inertia. NaN -> from
                                            %           H_nom / omega_nom
w.Irr        = 0.9e-3;                      % [kg*m^2]  transverse inertia about
                                            %           rotor CG.  OPTIONAL (NaN):
                                            %           only needed for the
                                            %           rocking-mode option.
% --- speed ---
w.rpm_nom    = 6000;                        % [rpm]  nominal (bias) speed
% --- imbalance (datasheet) ---
%   Static  imbalance units allowed : 'g*cm' 'g*mm' 'kg*m' 'oz*in'
%   Dynamic imbalance units allowed : 'g*cm^2' 'g*mm^2' 'kg*m^2' 'oz*in^2'
w.Us         = 0.50;   w.Us_unit = 'g*cm';  % static  imbalance (= mass x CG
                                            % eccentricity). NaN -> use e_cg
w.Ud         = 5.0;    w.Ud_unit = 'g*cm^2';% dynamic imbalance (= principal
                                            % axis tilt). NaN -> use tilt_deg
w.e_cg_mm    = NaN;                         % [mm]  OPTIONAL rotor CG offset from
                                            %       the spin axis, used only if
                                            %       Us is NaN:  Us = mass*e
w.tilt_deg   = NaN;                         % [deg] OPTIONAL tilt of the principal
                                            %       inertia axis, used only if Ud
                                            %       is NaN: Ud = (Izz-Irr)*tilt
w.phase_Ud   = 0;                           % [rad] angular position of the
                                            %       dynamic imbalance relative to
                                            %       the static one. Unknown ->
                                            %       0 (both peak together)
% --- harmonic content ---
%   One row per harmonic:  [ h ,  force ratio ,  moment ratio ]
%   h = multiple of spin speed.  Ratios scale the 1x imbalance coefficients
%   (Us for forces, Ud for moments).  With no test data keep only the
%   fundamental [1 1 1].  If a vendor spectrum shows e.g. a 2x line at 20 %
%   of the fundamental add a row [2 0.2 0.2].
w.harmonics  = [ 1  1  1 ];
w.C_ax       = 0;                           % [kg*m] axial (along spin axis) 1x
                                            %        force coefficient, F = C*W^2.
                                            %        0 if unknown (not derivable
                                            %        from imbalance)
% --- wheel structural modes, OPTIONAL (NaN frequency = rigid pass-through) ---
w.f_rock     = NaN;    w.zeta_rock = 0.02;  % [Hz, -] rocking mode at 0 rpm
w.f_ax       = NaN;    w.zeta_ax   = 0.02;  % [Hz, -] axial mode
w.f_rad      = NaN;    w.zeta_rad  = 0.02;  % [Hz, -] radial translation mode
wheel(1) = w;

% ---- 1d. WHEEL 2 -----------------------------------------------------------------
%  Start from a copy of wheel 1 and overwrite what differs.  Spins the same
%  way as wheel 1 -> same spin_axis.
w            = wheel(1);
w.name       = 'MW-2';
w.r_mount    = [-0.250; 0.150; 0.050];      % [m]
w.spin_axis  = [ 0; 0; 1];                  % same direction as MW-1
w.rpm_nom    = 6000;                        % [rpm]
wheel(2) = w;
clear w

% ---- 1e. Analysis settings --------------------------------------------------------
combine_method = 'worst';   % 'worst' = add amplitudes (structural sizing)
                            % 'rss'   = root-sum-square (jitter, SDO method)
rpm_sweep      = 0:50:8000; % [rpm] speed sweep applied to BOTH wheels
sc_modes_Hz    = [];        % [Hz]  OPTIONAL known structure modes to draw
                            %       on the Campbell diagram, e.g. [35 60 110]
t_end          = 0.05;      % [s]   time-history length at nominal speed
fs             = 20e3;      % [Hz]  time-history sample rate
phase_wheel2   = 0;         % [rad] assumed phase of wheel 2 relative to
                            %       wheel 1 for the time history only
makePlots      = true;
saveOutputs    = true;

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
nS = numel(rpm_sweep);
sweep = struct();
sweep.rpm = rpm_sweep(:);
for i = 1:nW, sweep.wheel(i).amp = zeros(nS, 6); end
sweep.combined = zeros(nS, 6);
for k = 1:nS
    amp_k = [];
    for i = 1:nW
        [Fb, Mb] = wheel_phasors(wheel(i), rpm_sweep(k), P_ref);
        a = [abs(Fb); abs(Mb)];
        sweep.wheel(i).amp(k,:) = combine_amp(a, combine_method).';
        amp_k = [amp_k, a]; %#ok<AGROW>
    end
    sweep.combined(k,:) = combine_amp(amp_k, combine_method).';
end
[~, kmax] = max(max(sweep.combined(:,1:3), [], 2));
sweep.rpm_maxForce = rpm_sweep(kmax);
[~, kmax] = max(max(sweep.combined(:,4:6), [], 2));
sweep.rpm_maxMoment = rpm_sweep(kmax);

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
results = struct('P_ref', P_ref, 'wheel', wheel, 'steady', steady, ...
                 'nominal', nominal, 'sweep', sweep, 'time', timeh, ...
                 'combine_method', combine_method, 'notes', {notes});
if saveOutputs
    save('MomentumWheelLoads_results.mat', 'results');
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
    fprintf('Saved MomentumWheelLoads_results.mat and MomentumWheelLoads_sweep.csv\n');
end

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
