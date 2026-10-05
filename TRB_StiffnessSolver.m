%% =========================================================================
%  TRB_StiffnessSolver.m
%  Quasi-static stiffness of SINGLE-row or DOUBLE-row tapered roller
%  bearings (TRB / DRTRB) under combined load, with optional preload,
%  clearance, thermal fit change and roller centrifugal force.
%
%  HOW TO USE
%    1. Fill in the "USER INPUTS" section below (units are stated on every
%       line).  Everything else is derived automatically.
%    2. Run the script.
%    3. Read the printed report and the figures.  The main result is the
%       6x6 stiffness matrix  stiffnessMatrix6x6  (N/mm, N/rad, N*mm/rad).
%    4. Set  runValidationCases = true  once to check the model against the
%       published numbers of Zhang 2023 (Table 3) and Kumar 2005 (REBM,
%       Table 3.1).  Both should reproduce to within a few percent.
%    (MATLAB R2016b or newer.  Under GNU Octave move the LOCAL FUNCTIONS
%     block above the USER INPUTS section, because Octave needs script-local
%     functions defined before they are called.)
%
%  WHAT THE MODEL DOES (in one paragraph)
%    The two rings are rigid.  One ring is displaced relative to the other
%    by q = [dx dy dz thetaX thetaY].  For every roller the elastic approach
%    along its contact normal is computed from q (plus preload / clearance /
%    thermal offsets).  Each roller's contact load follows the line-contact
%    law Q = Kn * delta^(10/9).  All roller loads are summed into forces and
%    moments on the ring.  Newton-Raphson adjusts q until those internal
%    loads balance the applied loads.  The stiffness matrix is the exact
%    analytical Jacobian  K = sum( k_j * g_j * g_j' )  at the solution.
%
%  CONVENTIONS
%    - Units: N, mm, rad, MPa, kg, degC.   Angles are entered in degrees.
%    - z is the bearing axis.  Origin = bearing centre (mid-way between the
%      two rows for a double-row bearing, at the row for a single row).
%    - Rotation about z is free, so row/column 6 of the 6x6 matrix is zero.
%    - WHICH RING IS FIXED:  set  loadAppliedTo  to the ring that is pushed
%      by the machine (the ring that moves). The other ring is the fixed
%      reaction.  Internally everything is computed as "inner ring relative
%      to outer ring"; if you choose 'outer' the script flips the signs for
%      you and reports outer-ring displacement.  The stiffness matrix is a
%      relative quantity and is the same either way.
%    - SINGLE ROW: with one row at z = 0 all contact normals pass through the
%      pressure-cone apex on the axis, so the model cannot carry a moment
%      independent of the radial load (My = -R tan(ao) Fx).  Only the three
%      translations are solved; applied Mx, My are ignored and the reaction
%      moments are reported.  Rows 4-5 of K are then linear combinations of
%      rows 1-2 (K has rank 3).  Real single-row tilt stiffness comes from
%      the contact spread along the roller, which this model does not have.
%    - Row index k = 1 is the row at negative z, k = 2 the row at +z.
%      For a back-to-back (O) pair the contact normal (inner -> outer) of
%      row 1 points toward +z and that of row 2 toward -z, so a +Fz on the
%      inner ring loads row 1.  This is Zhang 2019 Eq. 21 with i = 1 the
%      "left" row, and Zhang 2023 Eq. 7 with epsilon = +1 for row 1.
%    - MOMENT SIGN: Mx, My follow the right-hand rule about +x, +y with the
%      row numbering above.  The bending moment "M" of Zhang 2023 is -My
%      in this frame (their positive M unloads row 1 on the +x side).
%    - CONTACT COEFFICIENT CONVENTION (this changes Kn by ~2x, see below):
%      Palmgren's 3.84e-5*Q^0.9/l^0.8 and Luo's modification of it are
%      calibrated as the TOTAL approach of a roller squeezed between two
%      raceways (Harris; Lim & Singh's Kn = 7.86e4*l^(8/9); REBM).  Zhang
%      2019 / 2023 apply the same coefficient once per contact and then add
%      the two contacts in series, which doubles the compliance.  The
%      default 'whole-roller' follows Harris and reproduces REBM (Kumar
%      2005) within 3 %.  'per-contact' is the literal reading of Zhang.
%
%  REFERENCES (the formulation combines these)
%    Lim & Singh 1990, J.Sound Vib. 139(2)         - stiffness = Jacobian
%    Zhang, Shi, Liu, Chen 2019, Math.Probl.Eng.    - double-row kinematics
%                                                    (Eqs. 20-23), damped
%                                                    Newton-Raphson
%    Zhang, Lv, Han, Li 2023, Sensors 23, 4967      - different inner/outer
%                                                    contact angles (Eqs. 3-4,
%                                                    10-16), preload (22-23),
%                                                    centrifugal force (5, 25)
%    Kumar 2005, MSc thesis, Western Michigan Univ. - single-row TRB with
%                                                    REBM (Lim & Singh),
%                                                    ABAQUS and test data
%    Harris & Kotzalas 2006, Rolling Bearing Analysis - line-contact laws,
%                                                    cage speed, Hertz stress
%
%  ASSUMPTIONS / LIMITS (read before trusting a number)
%    - Rigid rings, rigid shaft and housing.  Real assemblies are softer
%      (fits, housing, shaft all act in series with the bearing).  Kumar
%      2005 measured roughly half the REBM axial stiffness for this reason.
%    - Contact angles do not change with load (small-displacement theory).
%    - Each roller is one rigid line contact: no crown, no roller tilt, no
%      edge loading.  Tilt stiffness is therefore somewhat over-predicted.
%    - Centrifugal force uses the "minimum outer load" shortcut of Zhang
%      2023 (with the sin(ao+af) that actually follows from their Eq. 3),
%      not a full per-roller equilibrium.  Fine for moderate speeds.
%    - No lubricant film stiffness, no friction, no gyroscopic effects.
%    - The stiffness here is the Jacobian of the global equilibrium (Zhang
%      2019 Eq. 23, Lim & Singh).  It is NOT the per-row series/parallel sum
%      of local contact stiffness used in Zhang 2023 Eqs. 27-34, so it will
%      not match their Figures 9-24 and is not meant to.
% =========================================================================

clear; clc; close all;

%% ========================================================================
%  USER INPUTS  -----------------------------------------------------------
%  ========================================================================

% ---------- Bearing type and arrangement ----------------------------------
bearingType        = 'double';          % 'single' or 'double'
arrangement        = 'back-to-back';    % double only: 'back-to-back' (O) or 'face-to-face' (X)
rowSpacing_mm      = 40;                % double only: axial distance between the two roller-set centres [mm]

% ---------- Roller set (same for both rows) -------------------------------
numRollersPerRow          = 15;         % Z, rollers in ONE row [-]
rollerMeanDiameter_mm     = 8;          % Dw, mean roller diameter [mm]
rollerEffectiveLength_mm  = 12;         % Le, roller-raceway CONTACT length (not overall length) [mm]
rollerMass_kg             = 5e-3;       % one roller [kg] (only used for centrifugal force)
cagePhase_deg             = 0;          % azimuth of roller #1 measured from +x [deg]

% ---------- Contact angles -------------------------------------------------
contactAngleOuter_deg     = 15;         % alpha_o, cup (outer raceway) contact angle [deg]
contactAngleInner_deg     = 12;         % alpha_i, cone (inner raceway) contact angle [deg]
flangeAngle_deg           = 75;         % alpha_f, roller big-end / rib contact angle [deg]

% ---------- Diameters ------------------------------------------------------
pitchDiameter_mm          = 70;         % dm, roller-centre circle diameter [mm]
innerRacewayDiameter_mm   = 62;         % Di, cone raceway diameter at mid-roller [mm]
outerRacewayDiameter_mm   = 78;         % Do, cup raceway diameter at mid-roller [mm]

% ---------- Material (body 1 = rings, body 2 = rollers) --------------------
youngsModulusRings_MPa    = 2.06e5;     % E1 [MPa]
poissonRatioRings         = 0.30;       % nu1 [-]
youngsModulusRollers_MPa  = 2.06e5;     % E2 [MPa]
poissonRatioRollers       = 0.30;       % nu2 [-]
contactLaw                = 'luo';      % 'palmgren' (classic, ignores diameters) or 'luo' (includes Dw and raceway curvature)
contactCoefficientConvention = 'whole-roller';  % 'whole-roller' (Harris / Lim & Singh / REBM, recommended)
                                                % 'per-contact'  (literal Zhang 2019/2023, ~2x more compliant)

% ---------- Fit: preload / clearance ---------------------------------------
% double row:  preloadMode = 'force'         -> preloadValue = axial preload force F0 [N] (per row, i.e. the
%                                                force each row carries with zero external load, zero thermal
%                                                offset and zero clearance)
%              preloadMode = 'interference'  -> preloadValue = TOTAL axial interference e [mm] (+ = preload)
%              preloadMode = 'endplay'       -> preloadValue = TOTAL axial endplay [mm] (+ = clearance)
% single row:  preloadMode is ignored (a single row cannot be self-preloaded); use radialClearance_mm.
preloadMode               = 'force';
preloadValue              = 500;        % see above
radialClearance_mm        = 0;          % diametral/2 radial clearance, + = clearance, - = interference [mm]

% ---------- Thermal fit change (all zero = ignored) -------------------------
% Simple steady-state model: free thermal growth of inner ring, outer ring, rollers,
% and the shaft-vs-housing axial growth over the row spacing.  Inner ring sits on the
% shaft, outer ring sits in the housing (or rotor).
cteRings_perC             = 11.5e-6;    % steel bearing rings & rollers [1/degC]
cteShaft_perC             = 11.5e-6;    % shaft carrying the inner rings [1/degC]
cteHousing_perC           = 11.5e-6;    % housing / rotor carrying the outer rings [1/degC]
tempRiseInnerRing_degC    = 0;          % above assembly temperature [degC]
tempRiseOuterRing_degC    = 0;
tempRiseRollers_degC      = 0;
tempRiseShaft_degC        = 0;
tempRiseHousing_degC      = 0;

% ---------- Applied loads (bearing frame) ---------------------------------
% loadAppliedTo = 'outer': the OUTER ring moves (e.g. inner ring clamped on a
%                          stationary shaft, outer ring carries a rotor/housing).
%                          Enter the load the rotor/housing pushes onto the outer ring.
% loadAppliedTo = 'inner': the INNER ring moves (classic rotating shaft in a
%                          fixed housing).  Enter the load the shaft pushes onto the inner ring.
% The fixed ring always carries the equal-and-opposite reaction; you never enter that one.
loadAppliedTo             = 'outer';
externalForceX_N          = 3000;       % Fx [N]
externalForceY_N          = 0;          % Fy [N]
externalForceZ_N          = 500;        % Fz [N]  (axial, along the shaft)
externalMomentX_Nmm       = 0;          % Mx [N*mm]
externalMomentY_Nmm       = 20000;      % My [N*mm]

% ---------- Speed / centrifugal force --------------------------------------
includeCentrifugal        = true;       % true/false
speedInput                = 'ring';     % 'ring': give the rotating ring speed (cage speed is derived)
                                        % 'cage': give the cage / roller-set speed directly (as Zhang 2023 does)
rotatingRing              = 'outer';    % speedInput 'ring': which ring rotates ('inner' or 'outer')
rotatingRingSpeed_rpm     = 3000;       % speedInput 'ring': speed of the rotating ring [rpm]
cageSpeed_rpm             = 1200;       % speedInput 'cage': cage speed [rpm]

% ---------- Solver settings ------------------------------------------------
numLoadSteps              = 10;         % load is ramped 0 -> 100% in this many steps (robustness)
maxNewtonIterations       = 50;         % per load step
residualTolerance         = 1e-9;       % relative force residual
stepTolerance             = 1e-10;      % relative displacement step
verboseSolver             = false;      % print every Newton iteration

% ---------- Post-processing options ----------------------------------------
doFiniteDifferenceCheck   = true;       % compare analytic K with finite differences (model self-check)
doCageAngleSweep          = true;       % sweep roller azimuth over one roller pitch -> time-varying stiffness
numCageSweepPoints        = 37;
doPlots                   = true;

% ---------- Parameter sweeps (nominal values above are used for everything
%            that is not being swept) --------------------------------------
doPreloadSweep            = false;                      % double row only (computed, not plotted)
preloadSweepForce_N       = linspace(0, 3000, 25);      % axial preload force per row [N]
doAxialLoadSweep          = true;
axialLoadSweepForce_N     = linspace(-4000, 4000, 33);  % Fz [N], sign per loadAppliedTo convention
doRadialLoadSweep         = true;
radialLoadSweepForce_N    = linspace(0, 10000, 26);     % Fx [N]
doMomentSweep             = false;                      % computed, not plotted
momentSweep_Nmm           = linspace(0, 2e5, 21);       % My [N*mm]

% ---------- Validation against the literature ------------------------------
runValidationCases        = false;      % true: reproduce Zhang 2023 Table 3 and Kumar 2005 Table 3.1, then continue

%% ========================================================================
%  END OF USER INPUTS.  Nothing below needs editing for normal use.
%  ========================================================================

%% ------------------------------------------------------------------------
%  1. Pack inputs, build the bearing (geometry, contact law, offsets, speed)
%  ------------------------------------------------------------------------
p = struct();
p.bearingType = bearingType;                    p.arrangement = arrangement;
p.rowSpacing_mm = rowSpacing_mm;
p.numRollersPerRow = numRollersPerRow;          p.rollerMeanDiameter_mm = rollerMeanDiameter_mm;
p.rollerEffectiveLength_mm = rollerEffectiveLength_mm;
p.rollerMass_kg = rollerMass_kg;                p.cagePhase_deg = cagePhase_deg;
p.contactAngleOuter_deg = contactAngleOuter_deg; p.contactAngleInner_deg = contactAngleInner_deg;
p.flangeAngle_deg = flangeAngle_deg;
p.pitchDiameter_mm = pitchDiameter_mm;          p.innerRacewayDiameter_mm = innerRacewayDiameter_mm;
p.outerRacewayDiameter_mm = outerRacewayDiameter_mm;
p.youngsModulusRings_MPa = youngsModulusRings_MPa;     p.poissonRatioRings = poissonRatioRings;
p.youngsModulusRollers_MPa = youngsModulusRollers_MPa; p.poissonRatioRollers = poissonRatioRollers;
p.contactLaw = contactLaw;                      p.contactCoefficientConvention = contactCoefficientConvention;
p.preloadMode = preloadMode;                    p.preloadValue = preloadValue;
p.radialClearance_mm = radialClearance_mm;
p.cteRings_perC = cteRings_perC;                p.cteShaft_perC = cteShaft_perC;
p.cteHousing_perC = cteHousing_perC;
p.tempRiseInnerRing_degC = tempRiseInnerRing_degC; p.tempRiseOuterRing_degC = tempRiseOuterRing_degC;
p.tempRiseRollers_degC = tempRiseRollers_degC;  p.tempRiseShaft_degC = tempRiseShaft_degC;
p.tempRiseHousing_degC = tempRiseHousing_degC;
p.loadAppliedTo = loadAppliedTo;
p.includeCentrifugal = includeCentrifugal;      p.speedInput = speedInput;
p.rotatingRing = rotatingRing;                  p.rotatingRingSpeed_rpm = rotatingRingSpeed_rpm;
p.cageSpeed_rpm = cageSpeed_rpm;

solverSettings = struct('numLoadSteps', numLoadSteps, 'maxNewtonIterations', maxNewtonIterations, ...
    'residualTolerance', residualTolerance, 'stepTolerance', stepTolerance, 'verbose', verboseSolver);

[bearing, derived] = assembleBearing(p);

% Flat names for the rest of the script
isDoubleRow          = derived.isDoubleRow;
numRows              = derived.numRows;
rowAxialPosition_mm  = derived.rowAxialPosition_mm;
rowNormalAxialSign   = derived.rowNormalAxialSign;
alphaOuter           = derived.alphaOuter;
alphaInner           = derived.alphaInner;
alphaFlange          = derived.alphaFlange;
loadDeflectionKn     = derived.loadDeflectionKn;
innerLoadRatio       = derived.innerLoadRatio;
flangeLoadRatio      = derived.flangeLoadRatio;
axialInterferencePerRow_mm      = derived.axialInterferencePerRow_mm;
normalOffsetExcludingPreload_mm = derived.normalOffsetExcludingPreload_mm;
normalOffset_mm      = derived.normalOffset_mm;
cageSpeed_rads       = derived.cageSpeed_rads;
centrifugalForce_N   = derived.centrifugalForce_N;
minimumOuterLoad_N   = derived.minimumOuterLoad_N;
loadSignFactor       = derived.loadSignFactor;
movingRingName       = derived.movingRingName;
fixedRingName        = derived.fixedRingName;
rollerAzimuth_rad    = bearing.rollerAzimuth_rad;
pitchRadius_mm       = bearing.pitchRadius_mm;

%% ------------------------------------------------------------------------
%  2. Optional: reproduce published results
%  ------------------------------------------------------------------------
if runValidationCases
    validateAgainstLiterature(solverSettings);
end

%% ------------------------------------------------------------------------
%  3. Loads
%  ------------------------------------------------------------------------
% Loads entered by the user, in the frame of the ring they push on.
userLoad = [externalForceX_N; externalForceY_N; externalForceZ_N; externalMomentX_Nmm; externalMomentY_Nmm];

% The solver works with "load on the inner ring" and "inner ring displacement
% relative to the outer ring".  If the user's load acts on the OUTER ring, the
% inner ring feels the equal-and-opposite reaction and the reported
% displacement (outer relative to inner) is the negative of the solver's q.
% Stiffness is unchanged (K relates relative motion to transmitted load).
externalLoad = loadSignFactor*userLoad;      % load on the inner ring, used by the solver
if ~isDoubleRow && any(userLoad(4:5) ~= 0)
    warning(['Single-row bearing: the applied moments Mx, My are ignored. With one row at z = 0 the roller ' ...
             'loads can only produce My = -R*tan(ao)*Fx and Mx = +R*tan(ao)*Fy (resultant through the ' ...
             'pressure-cone apex). The reaction moments are reported in the results.']);
end

%% ------------------------------------------------------------------------
%  4. Solve equilibrium:  Fint(q) = Fext   (damped Newton-Raphson + load steps)
%  ------------------------------------------------------------------------
[displacementVector, solverInfo] = solveBearingEquilibrium(bearing, externalLoad, solverSettings);
if ~solverInfo.converged
    warning('The nominal load case did not converge. Results below are not an equilibrium.');
    if ~isDoubleRow
        fprintf(['Hint: a single row must be held together by the axial load. The induced thrust of the radial load is at least\n' ...
                 '      |F_radial|*tan(ao) = %.0f N (more for a partial load zone), so Fz must exceed that in the tightening direction.\n'], ...
                 hypot(userLoad(1), userLoad(2))*tan(alphaOuter));
    end
end

%% ------------------------------------------------------------------------
%  5. Evaluate stiffness and roller loads at the solution
%  ------------------------------------------------------------------------
[internalLoad, stiffnessMatrix5x5, rollerOuterLoad_N, rollerApproach_mm] = evaluateBearing(bearing, displacementVector);

% Displacement of the MOVING ring relative to the FIXED ring, in the user's frame
reportedDisplacement = loadSignFactor*displacementVector;

% Expand to 6x6 (row/col 6 = rotation about the bearing axis, free)
stiffnessMatrix6x6 = zeros(6,6);
stiffnessMatrix6x6(1:5,1:5) = stiffnessMatrix5x5;

% Inner-race and rib loads per roller from the roller balance (Zhang 2023 Eqs. 3-4, with Fc)
rollerInnerLoad_N  = max(innerLoadRatio*rollerOuterLoad_N - centrifugalForce_N*sin(alphaFlange)/sin(alphaInner+alphaFlange), 0);
rollerFlangeLoad_N = max(flangeLoadRatio*rollerOuterLoad_N + centrifugalForce_N*sin(alphaInner)/sin(alphaInner+alphaFlange), 0);

% Hertz line-contact peak pressure (cylinder on cylinder, no crown).
% Johnson convention:  1/E* = (1-nu1^2)/E1 + (1-nu2^2)/E2,
%                      b    = sqrt(4*Q*Req/(pi*Le*E*)),   p_max = 2Q/(pi*b*Le)
effectiveModulus = 1/((1-poissonRatioRings^2)/youngsModulusRings_MPa + (1-poissonRatioRollers^2)/youngsModulusRollers_MPa);  % E* [MPa]
% Transverse curvature radius of a CONICAL raceway at the contact is (D/2)/cos(alpha),
% not D/2 (the section normal to the roller axis is an ellipse, not the pitch circle).
racewayCurvatureRadiusInner_mm = (innerRacewayDiameter_mm/2)/cos(alphaInner);
racewayCurvatureRadiusOuter_mm = (outerRacewayDiameter_mm/2)/cos(alphaOuter);
equivalentRadiusInner = 1/(2/rollerMeanDiameter_mm + 1/racewayCurvatureRadiusInner_mm);   % convex-convex
equivalentRadiusOuter = 1/(2/rollerMeanDiameter_mm - 1/racewayCurvatureRadiusOuter_mm);   % convex-concave
peakPressureInner_MPa = hertzLinePressure(rollerInnerLoad_N, equivalentRadiusInner, rollerEffectiveLength_mm, effectiveModulus);
peakPressureOuter_MPa = hertzLinePressure(rollerOuterLoad_N, equivalentRadiusOuter, rollerEffectiveLength_mm, effectiveModulus);

%% ------------------------------------------------------------------------
%  6. Self-checks
%  ------------------------------------------------------------------------
symmetryError = max(max(abs(stiffnessMatrix5x5 - stiffnessMatrix5x5'))) / max(max(abs(stiffnessMatrix5x5)));

finiteDifferenceError = NaN;
if doFiniteDifferenceCheck
    finiteDifferenceK = zeros(5,5);
    for columnIndex = 1:5
        perturbation = zeros(5,1);
        % step: 1e-6 mm for translations, 1e-6/R rad for rotations
        if columnIndex <= 3, stepSize = 1e-6; else, stepSize = 1e-6/pitchRadius_mm; end
        perturbation(columnIndex) = stepSize;
        loadPlus  = evaluateBearing(bearing, displacementVector + perturbation);
        loadMinus = evaluateBearing(bearing, displacementVector - perturbation);
        finiteDifferenceK(:, columnIndex) = (loadPlus - loadMinus)/(2*stepSize);
    end
    finiteDifferenceError = norm(finiteDifferenceK - stiffnessMatrix5x5, 'fro')/norm(stiffnessMatrix5x5, 'fro');
end

%% ------------------------------------------------------------------------
%  7. Parameter sweeps (each point is a full nonlinear re-solve)
%  ------------------------------------------------------------------------
% All sweeps store: stiffness diagonal (5), reported displacement (5), number
% of loaded rollers per row, maximum roller load, and per-row axial load.
% Points where no equilibrium exists (e.g. a single row pulled apart) are NaN.

% ---- 7a. cage angle over one roller pitch (time-varying stiffness) --------
if doCageAngleSweep
    fprintf('Cage-angle sweep: %d solves ...\n', numCageSweepPoints);
    cageSweepAngles_rad = linspace(0, 2*pi/numRollersPerRow, numCageSweepPoints);
    cageSweepDiagonal = nan(numCageSweepPoints, 5);
    bearingSweep = bearing;
    for sweepIndex = 1:numCageSweepPoints
        bearingSweep.rollerAzimuth_rad = rollerAzimuth_rad + cageSweepAngles_rad(sweepIndex);
        [~, sweepK, ~, sweepConverged] = solveAndEvaluate(bearingSweep, externalLoad, solverSettings);
        if sweepConverged, cageSweepDiagonal(sweepIndex, :) = diag(sweepK)'; end
    end
end

% ---- 7b. preload sweep (double row only) ---------------------------------
doPreloadSweep = doPreloadSweep && isDoubleRow;
if doPreloadSweep
    numPreloadPoints = numel(preloadSweepForce_N);
    fprintf('Preload sweep: %d solves ...\n', numPreloadPoints);
    preloadSweepDiagonal      = nan(numPreloadPoints, 5);
    preloadSweepDisplacement  = nan(numPreloadPoints, 5);
    preloadSweepLoadedRollers = nan(numPreloadPoints, numRows);
    preloadSweepMaxLoad       = nan(numPreloadPoints, numRows);
    preloadSweepInterference_um = zeros(numPreloadPoints, 1);
    bearingSweep = bearing;
    for sweepIndex = 1:numPreloadPoints
        sweepInterference_mm = preloadForceToInterference(preloadSweepForce_N(sweepIndex), numRollersPerRow, loadDeflectionKn, alphaOuter);
        preloadSweepInterference_um(sweepIndex) = 1e3*sweepInterference_mm;
        bearingSweep.normalOffset_mm = normalOffsetExcludingPreload_mm + sweepInterference_mm*sin(alphaOuter);
        [sweepQ, sweepK, sweepRollerLoad, sweepConverged] = solveAndEvaluate(bearingSweep, externalLoad, solverSettings);
        if ~sweepConverged, continue; end
        preloadSweepDiagonal(sweepIndex, :)      = diag(sweepK)';
        preloadSweepDisplacement(sweepIndex, :)  = (loadSignFactor*sweepQ)';
        preloadSweepLoadedRollers(sweepIndex, :) = sum(sweepRollerLoad > minimumOuterLoad_N + 1e-9, 1);
        preloadSweepMaxLoad(sweepIndex, :)       = max(sweepRollerLoad, [], 1);
    end
end

% ---- 7c. axial load sweep ---------------------------------------------------
if doAxialLoadSweep
    fprintf('Axial load sweep: %d solves ...\n', numel(axialLoadSweepForce_N));
    [axialSweepDiagonal, axialSweepDisplacement, axialSweepLoadedRollers, axialSweepMaxLoad, axialSweepRowAxialForce] = ...
        runLoadSweep(bearing, userLoad, 3, axialLoadSweepForce_N, loadSignFactor, alphaOuter, rowNormalAxialSign, solverSettings);
end

% ---- 7d. radial load sweep --------------------------------------------------
if doRadialLoadSweep
    fprintf('Radial load sweep: %d solves ...\n', numel(radialLoadSweepForce_N));
    [radialSweepDiagonal, radialSweepDisplacement, radialSweepLoadedRollers, radialSweepMaxLoad, radialSweepRowAxialForce] = ...
        runLoadSweep(bearing, userLoad, 1, radialLoadSweepForce_N, loadSignFactor, alphaOuter, rowNormalAxialSign, solverSettings);
end

% ---- 7e. moment sweep (double row only) ------------------------------------
if doMomentSweep && ~isDoubleRow
    fprintf('Moment sweep skipped: a single row cannot carry an independent moment in this model.\n');
end
doMomentSweep = doMomentSweep && isDoubleRow;
if doMomentSweep
    fprintf('Moment sweep: %d solves ...\n', numel(momentSweep_Nmm));
    [momentSweepDiagonal, momentSweepDisplacement, momentSweepLoadedRollers, momentSweepMaxLoad, momentSweepRowAxialForce] = ...
        runLoadSweep(bearing, userLoad, 5, momentSweep_Nmm, loadSignFactor, alphaOuter, rowNormalAxialSign, solverSettings);
end

%% ------------------------------------------------------------------------
%  8. Report
%  ------------------------------------------------------------------------
fprintf('\n==================== TAPERED ROLLER BEARING STIFFNESS ====================\n');
fprintf('Type: %s', bearingType);
if isDoubleRow, fprintf(' (%s, row spacing %.2f mm)', arrangement, rowSpacing_mm); end
fprintf('\nRollers/row: %d   Dw = %.2f mm   Le = %.2f mm   dm = %.2f mm\n', numRollersPerRow, rollerMeanDiameter_mm, rollerEffectiveLength_mm, pitchDiameter_mm);
fprintf('Contact angles: outer %.2f deg, inner %.2f deg, rib %.2f deg\n', contactAngleOuter_deg, contactAngleInner_deg, flangeAngle_deg);
fprintf('Contact law: %s (%s)   Kn = %.4e N/mm^(10/9)   Qi/Qo = %.4f   Qrib/Qo = %.4f\n', ...
    contactLaw, contactCoefficientConvention, loadDeflectionKn, innerLoadRatio, flangeLoadRatio);
if isDoubleRow
    fprintf('Preload mode: %s = %g  ->  axial interference per row %.4f um\n', preloadMode, preloadValue, 1e3*axialInterferencePerRow_mm(1));
end
fprintf('Normal offset per row (preload+clearance+thermal) [um]: %s\n', mat2str(1e3*normalOffset_mm, 4));
fprintf('Cage speed %.1f rad/s (%.1f rpm), centrifugal force per roller %.2f N, min outer load %.2f N\n', ...
    cageSpeed_rads, cageSpeed_rads*60/(2*pi), centrifugalForce_N, minimumOuterLoad_N);
fprintf('\nLoad applied to the %s ring (the %s ring is fixed and reacts it).\n', movingRingName, fixedRingName);
fprintf('Applied load  [Fx Fy Fz Mx My] = [%g %g %g %g %g]  (N, N*mm)\n', userLoad);
fprintf('Check: transmitted load recovered by the solver = [%.6g %.6g %.6g %.6g %.6g]\n', loadSignFactor*internalLoad);
if ~isDoubleRow
    fprintf('Single row: tilt is not an independent DOF (resultant passes through the apex at z = %+.2f mm).\n', -pitchRadius_mm*tan(alphaOuter));
    fprintf('            Reaction moments about the bearing centre: Mx = %.6g N*mm, My = %.6g N*mm\n', loadSignFactor*internalLoad(4), loadSignFactor*internalLoad(5));
end
fprintf('Solver: converged = %d, total Newton iterations = %d, final residual = %.2e\n', ...
    solverInfo.converged, solverInfo.totalIterations, solverInfo.finalResidual);

fprintf('\nDisplacement of the %s ring relative to the %s ring:\n', movingRingName, fixedRingName);
fprintf('  dx = %.4f um   dy = %.4f um   dz = %.4f um   thetaX = %.4e rad   thetaY = %.4e rad\n', ...
    1e3*reportedDisplacement(1), 1e3*reportedDisplacement(2), 1e3*reportedDisplacement(3), reportedDisplacement(4), reportedDisplacement(5));

fprintf('\nStiffness matrix K (rows/cols: x y z thetaX thetaY thetaZ)\n');
fprintf('  translation-translation blocks in N/mm, translation-rotation in N/rad, rotation-rotation in N*mm/rad\n');
disp(stiffnessMatrix6x6);
fprintf('Diagonal:  Kxx = %.4e N/mm   Kyy = %.4e N/mm   Kzz = %.4e N/mm\n', stiffnessMatrix6x6(1,1), stiffnessMatrix6x6(2,2), stiffnessMatrix6x6(3,3));
fprintf('           KtxTx = %.4e N*mm/rad   KtyTy = %.4e N*mm/rad\n', stiffnessMatrix6x6(4,4), stiffnessMatrix6x6(5,5));
fprintf('Self-checks: symmetry error = %.1e', symmetryError);
if doFiniteDifferenceCheck, fprintf('   finite-difference vs analytic Jacobian = %.1e', finiteDifferenceError); end
fprintf('\n');

fprintf('\nRoller loads per row:\n');
for rowIndex = 1:numRows
    loadedRollers = sum(rollerOuterLoad_N(:, rowIndex) > minimumOuterLoad_N + 1e-9);
    fprintf('  Row %d (z = %+.2f mm): %d of %d rollers loaded, max Qo = %.1f N, max Qi = %.1f N, max Qrib = %.1f N, pmax(inner/outer) = %.0f / %.0f MPa\n', ...
        rowIndex, rowAxialPosition_mm(rowIndex), loadedRollers, numRollersPerRow, ...
        max(rollerOuterLoad_N(:, rowIndex)), max(rollerInnerLoad_N(:, rowIndex)), max(rollerFlangeLoad_N(:, rowIndex)), ...
        max(peakPressureInner_MPa(:, rowIndex)), max(peakPressureOuter_MPa(:, rowIndex)));
end
if doCageAngleSweep
    fprintf('\nCage-angle sweep over one roller pitch (stiffness variation, peak-to-peak / mean):\n');
    fprintf('  Kxx %.2f %%   Kyy %.2f %%   Kzz %.2f %%   KtxTx %.2f %%   KtyTy %.2f %%\n', ...
        100*(max(cageSweepDiagonal) - min(cageSweepDiagonal))./mean(cageSweepDiagonal, 'omitnan'));
end
fprintf('==========================================================================\n');

%% ------------------------------------------------------------------------
%  9. Plots (one plot per figure)
%     Radial set (from the radial load sweep):
%       radial load vs radial displacement, radial stiffness vs radial displacement,
%       radial displacement vs radial load, radial stiffness vs radial load
%     Axial set (from the axial load sweep):
%       axial displacement vs axial load, axial stiffness vs axial load
%  ------------------------------------------------------------------------
if doPlots
    if doRadialLoadSweep
        radialDisplacement_um = 1e3*radialSweepDisplacement(:,1);     % dx of the moving ring
        radialStiffness_kNmm  = radialSweepDiagonal(:,1)/1e3;          % Kxx

        figure('Name', 'Radial load vs radial displacement');
        plot(radialDisplacement_um, radialLoadSweepForce_N, '-o', 'LineWidth', 1.4);
        xlabel('Radial displacement d_x [\mum]'); ylabel('Radial load F_x [N]'); grid on;
        title('Radial load-deflection curve');

        figure('Name', 'Radial stiffness vs radial displacement');
        plot(radialDisplacement_um, radialStiffness_kNmm, '-o', 'LineWidth', 1.4);
        xlabel('Radial displacement d_x [\mum]'); ylabel('Radial stiffness K_{xx} [kN/mm]'); grid on;
        title('Radial stiffness vs radial displacement');

        figure('Name', 'Radial displacement vs radial load');
        plot(radialLoadSweepForce_N, radialDisplacement_um, '-o', 'LineWidth', 1.4);
        xlabel('Radial load F_x [N]'); ylabel('Radial displacement d_x [\mum]'); grid on;
        title(sprintf('Radial deflection of the %s ring vs radial load', movingRingName));

        figure('Name', 'Radial stiffness vs radial load');
        plot(radialLoadSweepForce_N, radialStiffness_kNmm, '-o', 'LineWidth', 1.4);
        xlabel('Radial load F_x [N]'); ylabel('Radial stiffness K_{xx} [kN/mm]'); grid on;
        title('Radial stiffness vs radial load');
    end

    if doAxialLoadSweep
        axialDisplacement_um = 1e3*axialSweepDisplacement(:,3);       % dz of the moving ring
        axialStiffness_kNmm  = axialSweepDiagonal(:,3)/1e3;            % Kzz

        figure('Name', 'Axial displacement vs axial load');
        plot(axialLoadSweepForce_N, axialDisplacement_um, '-o', 'LineWidth', 1.4);
        xlabel('Axial load F_z [N]'); ylabel('Axial displacement d_z [\mum]'); grid on;
        title(sprintf('Axial deflection of the %s ring vs axial load', movingRingName));

        figure('Name', 'Axial stiffness vs axial load');
        plot(axialLoadSweepForce_N, axialStiffness_kNmm, '-o', 'LineWidth', 1.4);
        xlabel('Axial load F_z [N]'); ylabel('Axial stiffness K_{zz} [kN/mm]'); grid on;
        title('Axial stiffness vs axial load');
    end
end

%% ========================================================================
%  LOCAL FUNCTIONS
%  ========================================================================

function [bearing, derived] = assembleBearing(p)
% ASSEMBLEBEARING  Turn the user inputs into the geometry container used by
% evaluateBearing (Kn, contact normals, offsets, centrifugal floor) plus a
% struct of derived quantities for reporting.

bearingType   = lower(strtrim(p.bearingType));
arrangement   = lower(strtrim(p.arrangement));
contactLaw    = lower(strtrim(p.contactLaw));
convention    = lower(strtrim(p.contactCoefficientConvention));
preloadMode   = lower(strtrim(p.preloadMode));
rotatingRing  = lower(strtrim(p.rotatingRing));
speedInput    = lower(strtrim(p.speedInput));
loadAppliedTo = lower(strtrim(p.loadAppliedTo));

% ---- rows: axial position z_k and sign s_k of the normal's axial component
% For a single row: one row at z = 0, normal (inner -> outer) pointing +z.
% Back-to-back (O): pressure lines diverge outward, so the normal of the row
% at +z points toward -z  (Zhang 2019 Eq. 21, i = 1 left row has +z).
isDoubleRow = strcmp(bearingType, 'double');
if ~isDoubleRow && ~strcmp(bearingType, 'single')
    error('bearingType must be ''single'' or ''double''.');
end
if isDoubleRow
    rowAxialPosition_mm = [-p.rowSpacing_mm/2, +p.rowSpacing_mm/2];
    switch arrangement
        case 'back-to-back', rowNormalAxialSign = [+1, -1];
        case 'face-to-face', rowNormalAxialSign = [-1, +1];
        otherwise, error('arrangement must be ''back-to-back'' or ''face-to-face''.');
    end
else
    rowAxialPosition_mm = 0;
    rowNormalAxialSign  = +1;
end
numRows = numel(rowAxialPosition_mm);

% ---- angles
alphaOuter  = deg2rad(p.contactAngleOuter_deg);
alphaInner  = deg2rad(p.contactAngleInner_deg);
alphaFlange = deg2rad(p.flangeAngle_deg);
if alphaOuter < alphaInner
    warning('Outer contact angle is normally larger than the inner one for a tapered roller. Check inputs.');
end
pitchRadius_mm    = p.pitchDiameter_mm/2;
rollerAzimuth_rad = deg2rad(p.cagePhase_deg) + 2*pi*(0:p.numRollersPerRow-1)'/p.numRollersPerRow;

% ---- contact law  ->  Kn for one roller (both contacts in series)
% Line contact:  delta = G * Q^0.9   (delta in mm, Q in N).
%   'palmgren' : G = 3.84e-5 / Le^0.8  (steel), scaled by the actual material
%                compliance so non-steel bodies are handled.
%   'luo'      : G = 4.80 * [ (1-nu1^2)/(pi E1) + (1-nu2^2)/(pi E2) ]^0.9
%                      * (1 +/- Dw/Draceway)^0.1 / (Le^0.74 * Dw^0.1)
%                '+' inner (convex-convex), '-' outer (convex-concave).
%                (Zhang 2023 Eqs. 13-14; equals 4.83e-5*(..)/(Dw^0.1 Le^0.74)
%                 of Zhang 2019 Eq. 5 for steel, and equals Palmgren at
%                 Luo's calibration point Le = 10 mm, Dw = 40 mm.)
% Convention: both coefficients are calibrated as the approach of a roller
% between TWO raceways.  'whole-roller' therefore gives each contact half of
% the coefficient so that the series sum returns the calibrated value
% (Harris; Lim & Singh Kn = 7.86e4 Le^(8/9); REBM).  'per-contact' applies
% the full coefficient to each contact, as Zhang 2019/2023 do literally.
materialCompliance = (1-p.poissonRatioRings^2)/(pi*p.youngsModulusRings_MPa) ...
                   + (1-p.poissonRatioRollers^2)/(pi*p.youngsModulusRollers_MPa);   % [1/MPa]
steelCompliance    = 2*(1-0.3^2)/(pi*2.06e5);                                       % reference for Palmgren
curvatureRatioInner = p.rollerMeanDiameter_mm/p.innerRacewayDiameter_mm;            % Dw/Di
curvatureRatioOuter = p.rollerMeanDiameter_mm/p.outerRacewayDiameter_mm;            % Dw/Do
switch contactLaw
    case 'palmgren'
        coefficientInner = 3.84e-5 * (materialCompliance/steelCompliance)^0.9 / p.rollerEffectiveLength_mm^0.8;
        coefficientOuter = coefficientInner;
    case 'luo'
        coefficientInner = 4.80 * materialCompliance^0.9 * (1 + curvatureRatioInner)^0.1 ...
                         / (p.rollerEffectiveLength_mm^0.74 * p.rollerMeanDiameter_mm^0.1);
        coefficientOuter = 4.80 * materialCompliance^0.9 * (1 - curvatureRatioOuter)^0.1 ...
                         / (p.rollerEffectiveLength_mm^0.74 * p.rollerMeanDiameter_mm^0.1);
    otherwise
        error('contactLaw must be ''palmgren'' or ''luo''.');
end
switch convention
    case 'whole-roller', contactShare = 0.5;
    case 'per-contact',  contactShare = 1.0;
    otherwise, error('contactCoefficientConvention must be ''whole-roller'' or ''per-contact''.');
end
complianceInner = contactShare*coefficientInner;
complianceOuter = contactShare*coefficientOuter;

% Roller equilibrium (no speed): the three contact lines (outer, inner, rib)
% must close, so  Qi = innerLoadRatio*Qo,  Qf = flangeLoadRatio*Qo  (Zhang 2023 Eqs. 3-4).
innerLoadRatio  = sin(alphaOuter + alphaFlange)/sin(alphaInner + alphaFlange);
flangeLoadRatio = sin(alphaOuter - alphaInner)/sin(alphaInner + alphaFlange);

% Inner and outer contacts in series, inner deflection projected onto the
% outer contact normal with cos(alpha_o - alpha_i)  (Zhang 2023 Eqs. 11-12, 16):
%   delta_n = Go*Qo^0.9 + Gi*(ci*Qo)^0.9*cos(ao-ai)  =>  Qo = Kn * delta_n^(10/9)
totalCompliance   = complianceOuter + complianceInner*innerLoadRatio^0.9*cos(alphaOuter - alphaInner);
loadDeflectionKn  = totalCompliance^(-10/9);      % [N/mm^(10/9)]
loadDeflectionExp = 10/9;

% ---- preload / clearance  ->  normal-direction offset per row
axialInterferencePerRow_mm = zeros(1, numRows);
if isDoubleRow
    switch preloadMode
        case 'force'          % Zhang 2023 Eqs. 22-23
            if p.preloadValue < 0, error('preloadMode ''force'': the preload force must be >= 0.'); end
            axialInterferencePerRow_mm(:) = preloadForceToInterference(p.preloadValue, p.numRollersPerRow, loadDeflectionKn, alphaOuter);
        case 'interference'
            axialInterferencePerRow_mm(:) = p.preloadValue/2;
        case 'endplay'
            axialInterferencePerRow_mm(:) = -p.preloadValue/2;
        otherwise
            error('preloadMode must be ''force'', ''interference'' or ''endplay''.');
    end
elseif p.preloadValue ~= 0
    warning('Single-row bearing: preloadMode/preloadValue ignored (a lone row is preloaded by external axial load). Set preloadValue = 0 to silence this.');
end

% ---- thermal: free radial growth of raceways and rollers, axial growth of
% the shaft relative to the housing over the row spacing.
thermalRadialGrowthInner_mm = p.cteRings_perC*p.tempRiseInnerRing_degC*p.innerRacewayDiameter_mm/2;
thermalRadialGrowthOuter_mm = p.cteRings_perC*p.tempRiseOuterRing_degC*p.outerRacewayDiameter_mm/2;
thermalRollerGrowth_mm      = p.cteRings_perC*p.tempRiseRollers_degC*p.rollerMeanDiameter_mm;
thermalAxialInterferenceChange_mm = 0;
if isDoubleRow
    thermalAxialShaftMinusHousing_mm = (p.cteShaft_perC*p.tempRiseShaft_degC - p.cteHousing_perC*p.tempRiseHousing_degC)*p.rowSpacing_mm;
    % Axial growth of the shaft relative to the housing pushes the cones apart.
    % Back-to-back: cones are the adjusted parts  -> preload DEcreases.
    % Face-to-face: cups are the adjusted parts   -> preload INcreases.
    if strcmp(arrangement, 'back-to-back')
        thermalAxialInterferenceChange_mm = -thermalAxialShaftMinusHousing_mm;
    else
        thermalAxialInterferenceChange_mm = +thermalAxialShaftMinusHousing_mm;
    end
end
% Total normal offset per row:
%   axial interference   -> * sin(alpha_o)
%   radial interference  -> * cos(alpha_o)   (radial clearance enters with '-')
%   roller growth        -> straight along the normal (both contacts)
normalOffsetExcludingPreload_mm = ones(1, numRows) * ( (thermalAxialInterferenceChange_mm/2)*sin(alphaOuter) ...
                + (thermalRadialGrowthInner_mm - thermalRadialGrowthOuter_mm - p.radialClearance_mm)*cos(alphaOuter) ...
                + thermalRollerGrowth_mm );
normalOffset_mm = normalOffsetExcludingPreload_mm + axialInterferencePerRow_mm*sin(alphaOuter);

% ---- speed, centrifugal force and minimum outer-race load
% Cage speed from pure-rolling kinematics with the mean contact angle (Harris),
% or entered directly.  Then Fc = 0.5*m*dm*wc^2 (dm in m)  (Zhang 2023 Eq. 5).
meanContactAngle = 0.5*(alphaOuter + alphaInner);
switch speedInput
    case 'ring'
        rotatingRingSpeed_rads = p.rotatingRingSpeed_rpm*2*pi/60;
        switch rotatingRing
            case 'inner', cageSpeed_rads = 0.5*rotatingRingSpeed_rads*(1 - p.rollerMeanDiameter_mm*cos(meanContactAngle)/p.pitchDiameter_mm);
            case 'outer', cageSpeed_rads = 0.5*rotatingRingSpeed_rads*(1 + p.rollerMeanDiameter_mm*cos(meanContactAngle)/p.pitchDiameter_mm);
            otherwise,    error('rotatingRing must be ''inner'' or ''outer''.');
        end
    case 'cage'
        cageSpeed_rads = p.cageSpeed_rpm*2*pi/60;
    otherwise
        error('speedInput must be ''ring'' or ''cage''.');
end
if p.includeCentrifugal
    centrifugalForce_N = 0.5*p.rollerMass_kg*(p.pitchDiameter_mm/1000)*cageSpeed_rads^2;
else
    centrifugalForce_N = 0;
end
% With Fc the roller is thrown outward: even a roller that has lost inner
% contact still presses on the cup.  Setting Qi = 0 in Zhang 2023 Eq. 3 gives
% Qo_min = Fc*sin(af)/sin(ao+af)  (their Eq. 25 prints sin(ai+af); a typo).
minimumOuterLoad_N = centrifugalForce_N*sin(alphaFlange)/sin(alphaOuter + alphaFlange);

% ---- which ring the user's load acts on
switch loadAppliedTo
    case 'inner', loadSignFactor = +1; movingRingName = 'inner'; fixedRingName = 'outer';
    case 'outer', loadSignFactor = -1; movingRingName = 'outer'; fixedRingName = 'inner';
    otherwise,    error('loadAppliedTo must be ''inner'' or ''outer''.');
end

% ---- pack
bearing = struct();
bearing.numRows              = numRows;
bearing.numRollersPerRow     = p.numRollersPerRow;
bearing.rollerAzimuth_rad    = rollerAzimuth_rad;
bearing.rowAxialPosition_mm  = rowAxialPosition_mm;
bearing.rowNormalAxialSign   = rowNormalAxialSign;
bearing.pitchRadius_mm       = pitchRadius_mm;
bearing.alphaOuter           = alphaOuter;
bearing.loadDeflectionKn     = loadDeflectionKn;
bearing.loadDeflectionExp    = loadDeflectionExp;
bearing.normalOffset_mm      = normalOffset_mm;
bearing.minimumOuterLoad_N   = minimumOuterLoad_N;
% Degrees of freedom the solver balances.  For a single row at z = 0 every
% contact normal passes through the pressure-cone apex on the axis (the
% "effective centre" of Kumar 2005 Fig. 3.2), so the roller loads can only
% produce  My = -R*tan(ao)*Fx  and  Mx = +R*tan(ao)*Fy.  Tilt is not an
% independent degree of freedom: only the translations are solved and the
% moments come out as reactions.
if isDoubleRow, bearing.activeDof = 1:5; else, bearing.activeDof = 1:3; end

derived = struct();
derived.isDoubleRow          = isDoubleRow;
derived.numRows              = numRows;
derived.rowAxialPosition_mm  = rowAxialPosition_mm;
derived.rowNormalAxialSign   = rowNormalAxialSign;
derived.alphaOuter           = alphaOuter;
derived.alphaInner           = alphaInner;
derived.alphaFlange          = alphaFlange;
derived.complianceInner      = complianceInner;
derived.complianceOuter      = complianceOuter;
derived.loadDeflectionKn     = loadDeflectionKn;
derived.innerLoadRatio       = innerLoadRatio;
derived.flangeLoadRatio      = flangeLoadRatio;
derived.axialInterferencePerRow_mm      = axialInterferencePerRow_mm;
derived.normalOffsetExcludingPreload_mm = normalOffsetExcludingPreload_mm;
derived.normalOffset_mm      = normalOffset_mm;
derived.cageSpeed_rads       = cageSpeed_rads;
derived.centrifugalForce_N   = centrifugalForce_N;
derived.minimumOuterLoad_N   = minimumOuterLoad_N;
derived.loadSignFactor       = loadSignFactor;
derived.movingRingName       = movingRingName;
derived.fixedRingName        = fixedRingName;
end

% -------------------------------------------------------------------------
function interference_mm = preloadForceToInterference(preloadForce_N, numRollers, loadDeflectionKn, alphaOuter)
% PRELOADFORCETOINTERFERENCE  Axial interference per row that makes one row
% carry the axial force F0 with all Z rollers equally loaded (Zhang 2023 Eqs. 22-23):
%   F0 = Z*Kn*(e*sin(ao))^(10/9)*sin(ao)   ->   e = (F0/(Z*Kn*sin(ao)))^0.9 / sin(ao)
interference_mm = (preloadForce_N/(numRollers*loadDeflectionKn*sin(alphaOuter)))^0.9 / sin(alphaOuter);
end

% -------------------------------------------------------------------------
function [internalLoad, stiffnessMatrix, rollerOuterLoad_N, rollerApproach_mm] = evaluateBearing(bearing, displacementVector)
% EVALUATEBEARING  Internal load vector, analytic stiffness matrix and roller
% loads for a given ring displacement q = [dx dy dz thetaX thetaY].
%
%   For roller j in row k with position p = [R cos(psi), R sin(psi), z_k] and
%   contact normal n = [cos(ao) cos(psi), cos(ao) sin(psi), s_k sin(ao)]:
%       delta_n = n . (d + theta x p) + offset_k  =  g' * q + offset_k
%       g       = [ n ; (p x n)_x ; (p x n)_y ]        (5x1)
%       Qo      = Kn * delta_n^(10/9)       (0 if delta_n <= 0),  floored at Qo_min
%       Fint    = sum Qo * g                (force AND moment on the inner ring)
%       K       = sum (dQo/ddelta_n) * g * g'
%   Expanding g reproduces Zhang 2019 Eqs. 20-21 exactly, including the
%   moment arm (R sin(ao) + z_k cos(ao)).  The same g that maps displacement
%   -> approach also maps load -> force, which is why K is symmetric.

numRows             = bearing.numRows;
numRollersPerRow    = bearing.numRollersPerRow;
rollerAzimuth_rad   = bearing.rollerAzimuth_rad;
rowAxialPosition_mm = bearing.rowAxialPosition_mm;
rowNormalAxialSign  = bearing.rowNormalAxialSign;
pitchRadius_mm      = bearing.pitchRadius_mm;
alphaOuter          = bearing.alphaOuter;
loadDeflectionKn    = bearing.loadDeflectionKn;
loadDeflectionExp   = bearing.loadDeflectionExp;
normalOffset_mm     = bearing.normalOffset_mm;
minimumOuterLoad_N  = bearing.minimumOuterLoad_N;

internalLoad      = zeros(5,1);
stiffnessMatrix   = zeros(5,5);
rollerOuterLoad_N = zeros(numRollersPerRow, numRows);
rollerApproach_mm = zeros(numRollersPerRow, numRows);

for rowIndex = 1:numRows
    axialSign   = rowNormalAxialSign(rowIndex);
    rowAxial_mm = rowAxialPosition_mm(rowIndex);
    for rollerIndex = 1:numRollersPerRow
        psi = rollerAzimuth_rad(rollerIndex);
        cosPsi = cos(psi); sinPsi = sin(psi);

        % Contact normal (inner -> outer) and roller position
        normalVector   = [cos(alphaOuter)*cosPsi; cos(alphaOuter)*sinPsi; axialSign*sin(alphaOuter)];
        positionVector = [pitchRadius_mm*cosPsi; pitchRadius_mm*sinPsi; rowAxial_mm];
        leverVector    = cross(positionVector, normalVector);          % p x n
        geometryVector = [normalVector; leverVector(1); leverVector(2)]; % g (5x1)

        % Elastic approach along the normal
        approach_mm = geometryVector'*displacementVector + normalOffset_mm(rowIndex);
        rollerApproach_mm(rollerIndex, rowIndex) = approach_mm;

        % Contact load and its derivative (local stiffness)
        if approach_mm > 0
            outerLoad_N    = loadDeflectionKn*approach_mm^loadDeflectionExp;
            localStiffness = loadDeflectionExp*loadDeflectionKn*approach_mm^(loadDeflectionExp - 1);
        else
            outerLoad_N    = 0;
            localStiffness = 0;
        end
        % Centrifugal floor: a roller held only by its own inertia still loads the
        % cup but adds no ring-to-ring stiffness (its load no longer changes with q).
        if outerLoad_N < minimumOuterLoad_N
            outerLoad_N    = minimumOuterLoad_N;
            localStiffness = 0;
        end
        rollerOuterLoad_N(rollerIndex, rowIndex) = outerLoad_N;

        % Accumulate force/moment and stiffness
        internalLoad    = internalLoad + outerLoad_N*geometryVector;
        stiffnessMatrix = stiffnessMatrix + localStiffness*(geometryVector*geometryVector');
    end
end
end

% -------------------------------------------------------------------------
function [displacementVector, solverInfo] = solveBearingEquilibrium(bearing, externalLoad, s)
% SOLVEBEARINGEQUILIBRIUM  Damped Newton-Raphson with load stepping.
%
%   Residual  R(q) = Fint(q) - Fext.   Newton step:  K(q) dq = -R(q).
%   - The load is ramped in numLoadSteps increments; each converged q seeds
%     the next step (keeps the iteration inside the right load zone).
%   - Displacements and residuals are scaled (rotations by pitch radius) so
%     the Jacobian is well conditioned in mixed units.
%   - Each step must decrease ||R|| ("downhill", Zhang 2019 Eqs. 31-32).
%     Instead of halving the step, a Levenberg-Marquardt damping is raised
%     until the residual drops; this also copes with a rank-deficient K when
%     only one or two rollers are loaded.  If no damping gives a decrease the
%     iterate is NOT accepted and the solve is abandoned with converged = false.
%   - If no roller is elastically loaded (zero preload, clearance, or all
%     rollers on the centrifugal floor) K is singular; a growing nudge along
%     the load direction gets it started.  If K stays singular, every roller
%     is unloaded under this load and no equilibrium exists (e.g. a single
%     row pulled apart): the solve stops with converged = false.

pitchRadius_mm = bearing.pitchRadius_mm;
scalingMatrix  = diag([1 1 1 1/pitchRadius_mm 1/pitchRadius_mm]);   % q = S * qScaled
active         = bearing.activeDof;                                  % DOFs that are balanced
inactive       = setdiff(1:5, active);
if ~isfield(s, 'warn'), s.warn = true; end

displacementVector = zeros(5,1);
externalScaled = scalingMatrix*externalLoad;
referenceLoad  = max(norm(externalScaled(active)), 1);               % for relative residual
totalIterations = 0;
converged = true;
abortSolve = false;
marquardt = 1e-8;        % Levenberg-Marquardt damping, adapted during the iteration

for loadStepIndex = 1:s.numLoadSteps
    loadFraction = loadStepIndex/s.numLoadSteps;
    targetLoad   = loadFraction*externalLoad;

    stepConverged = false;
    nudgeCount = 0;
    for newtonIndex = 1:s.maxNewtonIterations
        totalIterations = totalIterations + 1;
        [internalLoad, stiffnessMatrix] = evaluateBearing(bearing, displacementVector);
        residual       = internalLoad - targetLoad;
        residualScaled = scalingMatrix*residual;
        residualScaled(inactive) = 0;                               % inactive DOFs are not balanced
        residualNorm   = norm(residualScaled)/referenceLoad;

        if residualNorm < s.residualTolerance
            stepConverged = true; break;
        end

        % Singular K: no roller is elastically loaded (zero preload, clearance, or
        % every roller sitting on the centrifugal floor).  Nudge along the load
        % direction with a growing step (0.1 um, 1 um, ... 100 um in scaled space)
        % until a roller takes load.  If that never happens, no equilibrium exists.
        if trace(stiffnessMatrix(active, active)) == 0
            loadDirectionScaled = scalingMatrix*targetLoad;                 % [F ; M/R]
            loadDirectionScaled(inactive) = 0;
            nudgeCount = nudgeCount + 1;
            if nudgeCount > 4 || norm(loadDirectionScaled) == 0
                if s.warn
                    warning(['Every roller is unloaded at load step %d of %d: no equilibrium exists for this load ' ...
                             '(e.g. a single row pulled apart, or a double row with endplay under pure moment).'], ...
                             loadStepIndex, s.numLoadSteps);
                end
                abortSolve = true; break;
            end
            nudgeScaled = 1e-4*10^(nudgeCount-1)*loadDirectionScaled/norm(loadDirectionScaled);
            displacementVector = displacementVector + scalingMatrix*nudgeScaled;
            continue;
        end

        % Scaled, damped Newton (Levenberg-Marquardt) step:
        %     (S K S + mu*diagScale*I) dqScaled = -S R
        % mu ~ 0 is a pure Newton step.  If the step does not reduce the
        % residual, mu is raised tenfold and the step recomputed: it becomes
        % shorter and bends toward the load direction, which is what is needed
        % when K is rank-deficient (one or two rollers loaded) or when rollers
        % enter/leave the load zone.  On success mu is relaxed again.
        jacobianScaled = scalingMatrix*stiffnessMatrix*scalingMatrix;
        jacobianActive = jacobianScaled(active, active);
        diagScale = max(abs(diag(jacobianActive)));
        stepAccepted = false;
        for attemptIndex = 1:40
            newtonStepScaled = zeros(5,1);
            newtonStepScaled(active) = -(jacobianActive + marquardt*diagScale*eye(numel(active)))\residualScaled(active);
            trialDisplacement = displacementVector + scalingMatrix*newtonStepScaled;
            trialInternalLoad = evaluateBearing(bearing, trialDisplacement);
            trialScaled = scalingMatrix*(trialInternalLoad - targetLoad);
            trialScaled(inactive) = 0;
            trialNorm = norm(trialScaled)/referenceLoad;
            if trialNorm < residualNorm
                stepAccepted = true;
                marquardt = max(marquardt/10, 1e-12);
                break;
            end
            marquardt = marquardt*10;
            if marquardt > 1e12, break; end
        end
        if ~stepAccepted
            if s.warn
                warning(['No step reduces the residual at load step %d of %d (relative residual %.2e): ' ...
                         'no equilibrium found for this load. Last iterate kept, solve abandoned.'], ...
                         loadStepIndex, s.numLoadSteps, residualNorm);
            end
            abortSolve = true; break;
        end

        stepNormRelative = norm(newtonStepScaled)/max(norm(scalingMatrix\displacementVector), 1e-6);
        displacementVector = trialDisplacement;

        if s.verbose
            fprintf('  load step %2d  iter %2d  |R| = %.3e  |dq| = %.3e  mu = %.1e\n', ...
                loadStepIndex, newtonIndex, trialNorm, stepNormRelative, marquardt);
        end
        % Convergence is decided by the RESIDUAL only.  A vanishing step with a
        % non-zero residual means the iteration stagnated, which is reported,
        % not counted as a solution.
        if trialNorm < s.residualTolerance
            stepConverged = true; break;
        elseif stepNormRelative < s.stepTolerance
            if s.warn
                warning('Newton-Raphson stagnated at load step %d of %d (relative residual %.2e > tolerance %.1e).', ...
                    loadStepIndex, s.numLoadSteps, trialNorm, s.residualTolerance);
            end
            break;
        end
    end
    if ~stepConverged
        converged = false;
        if ~abortSolve && newtonIndex == s.maxNewtonIterations && s.warn
            warning('Newton-Raphson hit the iteration limit at load step %d of %d.', loadStepIndex, s.numLoadSteps);
        end
    end
    if abortSolve, break; end
end

internalLoad   = evaluateBearing(bearing, displacementVector);
residualScaled = scalingMatrix*(internalLoad - externalLoad);
finalResidual  = norm(residualScaled(active))/referenceLoad;

solverInfo = struct('converged', converged, 'totalIterations', totalIterations, 'finalResidual', finalResidual);
end

% -------------------------------------------------------------------------
function [displacementVector, stiffnessMatrix, rollerOuterLoad_N, converged] = solveAndEvaluate(bearing, externalLoad, s)
% SOLVEANDEVALUATE  Convenience wrapper: solve equilibrium, then return q, K,
% the roller loads at the solution and the convergence flag.  Used by all the sweeps.
sQuiet = s; sQuiet.verbose = false; sQuiet.warn = false;   % sweeps report failures as NaN, not as warnings
[displacementVector, solverInfo] = solveBearingEquilibrium(bearing, externalLoad, sQuiet);
[~, stiffnessMatrix, rollerOuterLoad_N] = evaluateBearing(bearing, displacementVector);
converged = solverInfo.converged;
end

% -------------------------------------------------------------------------
function [sweepDiagonal, sweepDisplacement, sweepLoadedRollers, sweepMaxLoad, sweepRowAxialForce] = ...
    runLoadSweep(bearing, userLoad, loadComponentIndex, sweepValues, loadSignFactor, alphaOuter, rowNormalAxialSign, s)
% RUNLOADSWEEP  Vary ONE component of the user's load vector (1..5 = Fx Fy Fz Mx My),
% keep the others at their nominal values, re-solve at every point.
%   sweepRowAxialForce(i,k) = axial force carried by row k (sum of Qo*sin(ao)*s_k),
%   handy for seeing how an axial load shifts between the two rows.
%   Points with no equilibrium are left as NaN.
numPoints = numel(sweepValues);
numRows   = bearing.numRows;
sweepDiagonal      = nan(numPoints, 5);
sweepDisplacement  = nan(numPoints, 5);
sweepLoadedRollers = nan(numPoints, numRows);
sweepMaxLoad       = nan(numPoints, numRows);
sweepRowAxialForce = nan(numPoints, numRows);
numFailed = 0;
for pointIndex = 1:numPoints
    sweepUserLoad = userLoad;
    sweepUserLoad(loadComponentIndex) = sweepValues(pointIndex);
    [sweepQ, sweepK, sweepRollerLoad, sweepConverged] = solveAndEvaluate(bearing, loadSignFactor*sweepUserLoad, s);
    if ~sweepConverged, numFailed = numFailed + 1; continue; end
    sweepDiagonal(pointIndex, :)      = diag(sweepK)';
    sweepDisplacement(pointIndex, :)  = (loadSignFactor*sweepQ)';
    sweepLoadedRollers(pointIndex, :) = sum(sweepRollerLoad > bearing.minimumOuterLoad_N + 1e-9, 1);
    sweepMaxLoad(pointIndex, :)       = max(sweepRollerLoad, [], 1);
    sweepRowAxialForce(pointIndex, :) = loadSignFactor*sum(sweepRollerLoad, 1).*sin(alphaOuter).*rowNormalAxialSign;
end
if numFailed > 0
    fprintf('  (%d of %d sweep points had no equilibrium and are NaN)\n', numFailed, numPoints);
end
end

% -------------------------------------------------------------------------
function peakPressure_MPa = hertzLinePressure(contactLoad_N, equivalentRadius_mm, contactLength_mm, effectiveModulus_MPa)
% HERTZLINEPRESSURE  Peak pressure of a Hertzian line contact (Johnson form).
%   b    = sqrt( 4 Q Req / (pi Le E*) )      half contact width, 1/E* = sum (1-nu^2)/E
%   pmax = 2 Q / (pi b Le)
halfWidth_mm = sqrt(4*contactLoad_N*equivalentRadius_mm./(pi*contactLength_mm*effectiveModulus_MPa));
peakPressure_MPa = zeros(size(contactLoad_N));
loadedMask = contactLoad_N > 0;
peakPressure_MPa(loadedMask) = 2*contactLoad_N(loadedMask)./(pi*halfWidth_mm(loadedMask)*contactLength_mm);
end

% -------------------------------------------------------------------------
function p = defaultInputs()
% DEFAULTINPUTS  A complete input struct (used as the base for the validation cases).
p = struct();
p.bearingType = 'double';  p.arrangement = 'back-to-back';  p.rowSpacing_mm = 40;
p.numRollersPerRow = 15;   p.rollerMeanDiameter_mm = 8;     p.rollerEffectiveLength_mm = 12;
p.rollerMass_kg = 5e-3;    p.cagePhase_deg = 0;
p.contactAngleOuter_deg = 15; p.contactAngleInner_deg = 12; p.flangeAngle_deg = 75;
p.pitchDiameter_mm = 70;   p.innerRacewayDiameter_mm = 62;  p.outerRacewayDiameter_mm = 78;
p.youngsModulusRings_MPa = 2.06e5;   p.poissonRatioRings = 0.30;
p.youngsModulusRollers_MPa = 2.06e5; p.poissonRatioRollers = 0.30;
p.contactLaw = 'luo';      p.contactCoefficientConvention = 'whole-roller';
p.preloadMode = 'force';   p.preloadValue = 0;              p.radialClearance_mm = 0;
p.cteRings_perC = 11.5e-6; p.cteShaft_perC = 11.5e-6;       p.cteHousing_perC = 11.5e-6;
p.tempRiseInnerRing_degC = 0; p.tempRiseOuterRing_degC = 0; p.tempRiseRollers_degC = 0;
p.tempRiseShaft_degC = 0;  p.tempRiseHousing_degC = 0;
p.loadAppliedTo = 'inner';
p.includeCentrifugal = false; p.speedInput = 'cage';  p.rotatingRing = 'inner';
p.rotatingRingSpeed_rpm = 0;  p.cageSpeed_rpm = 0;
end

% -------------------------------------------------------------------------
function validateAgainstLiterature(s)
% VALIDATEAGAINSTLITERATURE  Reproduce two published data sets.
%
%  (A) Zhang, Lv, Han, Li 2023, Sensors 23, 4967, Table 3 ("Model in this
%      paper" column): HH926700 double-row bearing, maximum contact loads for
%      three composite load cases at a cage speed of 1200 rpm with 100 N
%      preload per row and M = 20 N*m.  Their positive M is -My here.  Contact
%      loads do not depend on the coefficient convention, so the literal
%      'per-contact' reading is used to stay close to the paper.
%  (B) Kumar 2005 (MSc thesis, WMU), Table 3.1: REBM (Lim & Singh) stiffness
%      of a single-row TRB (Z = 20, pitch radius 40.5 mm, roller length
%      18.251 mm, cup angle 31.128 deg, no clearance) under pure axial load.
%      REBM uses Palmgren once for the whole roller, so 'whole-roller'.

fprintf('\n################ VALIDATION AGAINST PUBLISHED RESULTS ################\n');

% ---------- (A) Zhang 2023, Table 3 ----------
p = defaultInputs();
p.bearingType = 'double'; p.arrangement = 'back-to-back'; p.rowSpacing_mm = 75.96;
p.numRollersPerRow = 14;  p.rollerMeanDiameter_mm = 0.5*(30.27 + 36.74);  p.rollerEffectiveLength_mm = 57.02;
p.rollerMass_kg = 0.126;
p.contactAngleOuter_deg = 22.54; p.contactAngleInner_deg = 16.24; p.flangeAngle_deg = 70.20;
p.pitchDiameter_mm = 198.93; p.innerRacewayDiameter_mm = 167.35; p.outerRacewayDiameter_mm = 230.51;
p.youngsModulusRings_MPa = 2.1e5; p.youngsModulusRollers_MPa = 2.1e5;
p.contactLaw = 'luo'; p.contactCoefficientConvention = 'per-contact';
p.preloadMode = 'force'; p.preloadValue = 100;
p.loadAppliedTo = 'inner';
p.includeCentrifugal = true; p.speedInput = 'cage'; p.cageSpeed_rpm = 1200;
[bearingA, derivedA] = assembleBearing(p);

%               Fr    Fa     Qo_max  Qi_max  Qf_max   (paper, N)
casesA = [    1500  5000    1304.8  1119.3  198.9
              5000  1500    1084.6   898.9  174.7
              8000  3000    1731.3  1546.1  245.8 ];
fprintf('\n(A) Zhang 2023 Table 3, HH926700, wc = 1200 rpm, F0 = 100 N/row, M = 20 N*m\n');
fprintf('    Fc = %.1f N per roller, Qo_min = %.1f N (their Fig. 7b floor ~190 N)\n', derivedA.centrifugalForce_N, derivedA.minimumOuterLoad_N);
fprintf('    %6s %6s | %8s %8s %6s | %8s %8s %6s | %s\n', 'Fr', 'Fa', 'Qo paper', 'Qo code', 'err%', 'Qi paper', 'Qi code', 'err%', 'Qrib paper / code');
for caseIndex = 1:size(casesA, 1)
    load = [casesA(caseIndex,1); 0; casesA(caseIndex,2); 0; -20000];
    [~, ~, Qo, ok] = solveAndEvaluate(bearingA, load, s);
    Qi = max(derivedA.innerLoadRatio*Qo  - derivedA.centrifugalForce_N*sin(derivedA.alphaFlange)/sin(derivedA.alphaInner + derivedA.alphaFlange), 0);
    Qf = max(derivedA.flangeLoadRatio*Qo + derivedA.centrifugalForce_N*sin(derivedA.alphaInner)/sin(derivedA.alphaInner + derivedA.alphaFlange), 0);
    QoMax = max(Qo(:)); QiMax = max(Qi(:)); QfMax = max(Qf(:));
    fprintf('    %6g %6g | %8.1f %8.1f %5.1f%% | %8.1f %8.1f %5.1f%% | %6.1f / %6.1f %s\n', ...
        casesA(caseIndex,1), casesA(caseIndex,2), casesA(caseIndex,3), QoMax, 100*(QoMax/casesA(caseIndex,3) - 1), ...
        casesA(caseIndex,4), QiMax, 100*(QiMax/casesA(caseIndex,4) - 1), casesA(caseIndex,5), QfMax, ternary(ok, '', '(not converged)'));
end
fprintf('    Expected: cases 1 and 3 match to 0.1 N, case 2 within about 4 %%.\n');

% ---------- (B) Kumar 2005, Table 3.1 (REBM) ----------
p = defaultInputs();
p.bearingType = 'single';
p.numRollersPerRow = 20; p.rollerEffectiveLength_mm = 18.251; p.pitchDiameter_mm = 2*40.5;
p.rollerMeanDiameter_mm = 10; p.innerRacewayDiameter_mm = 71; p.outerRacewayDiameter_mm = 91;   % not used by Palmgren
p.contactAngleOuter_deg = 31.128; p.contactAngleInner_deg = 31.128; p.flangeAngle_deg = 75;      % REBM: one contact angle
p.contactLaw = 'palmgren'; p.contactCoefficientConvention = 'whole-roller';
p.preloadValue = 0; p.radialClearance_mm = 0; p.includeCentrifugal = false;
p.loadAppliedTo = 'inner';
[bearingB, ~] = assembleBearing(p);

%            Fa      Kxx     Kzz    (REBM, MN/mm)
casesB = [ 4000    3.84    2.80
          20000    4.67    3.29
          40000    4.83    3.53
          60000    5.03    3.67
          85000    5.21    3.80
          98000    5.29    3.86 ];
fprintf('\n(B) Kumar 2005 Table 3.1, REBM single-row TRB, pure axial load (Palmgren, whole-roller)\n');
fprintf('    %7s | %9s %9s %6s | %9s %9s %6s\n', 'Fa [N]', 'Kxx REBM', 'Kxx code', 'err%', 'Kzz REBM', 'Kzz code', 'err%');
for caseIndex = 1:size(casesB, 1)
    load = [0; 0; casesB(caseIndex,1); 0; 0];
    [~, K, ~, ok] = solveAndEvaluate(bearingB, load, s);
    fprintf('    %7g | %9.2f %9.2f %5.1f%% | %9.2f %9.2f %5.1f%% %s\n', casesB(caseIndex,1), ...
        casesB(caseIndex,2), K(1,1)/1e6, 100*(K(1,1)/1e6/casesB(caseIndex,2) - 1), ...
        casesB(caseIndex,3), K(3,3)/1e6, 100*(K(3,3)/1e6/casesB(caseIndex,3) - 1), ternary(ok, '', '(not converged)'));
end
fprintf('    Expected: within about 3 %% (REBM rounds Kn to 7.86e4*l^(8/9); exact Palmgren inversion gives 8.06e4).\n');
fprintf('    With ''per-contact'' the code would be a factor 2 lower.\n');
fprintf('#######################################################################\n\n');
end

% -------------------------------------------------------------------------
function out = ternary(condition, valueIfTrue, valueIfFalse)
if condition, out = valueIfTrue; else, out = valueIfFalse; end
end
