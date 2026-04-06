% =========================================================
% PROJECT : Dynamic Weight-Adaptive Threat Scoring Framework
% FILE    : radar_simulation.m  (UPDATED — v2)
% RUNS IN : MATLAB R2020a or later
% OUTPUT  : ../data/input_data.txt
% =========================================================
% NEW IN v2:
%   1. Extended Kalman Filter (EKF) for maneuvering targets
%   2. Formation detection — groups nearby targets
%   3. Evasive maneuver simulation (targets that zigzag)
%   4. Stealth target (low RCS, appears/disappears)
%   5. Better console report with formation info
%   6. Multiple plots: distance, speed, RCS, formation map
% =========================================================
% HOW TO RUN:
%   1. Open MATLAB
%   2. Set working directory to AirDefenseRadar/matlab/
%   3. Press Run
%   4. input_data.txt appears in AirDefenseRadar/data/
% =========================================================

clc; clear; close all;

%% ── CONFIGURATION ────────────────────────────────────────────
NUM_TARGETS = 10;
DT          = 1.0;    % Time step (seconds)
NUM_STEPS   = 8;      % EKF iterations per target (more = more accurate)
rng(42);              % Fixed seed for reproducibility

OUTPUT_FILE = '../data/input_data.txt';

% Create output folder if needed
if ~exist('../data', 'dir')
    mkdir('../data');
    fprintf('Created folder: ../data/\n');
end

%% ── TRUE TARGET PARAMETERS ───────────────────────────────────
%
%  Target types by speed:
%    Speed < 300 km/h  → DRONE
%    Speed 300-699     → AIRCRAFT
%    Speed >= 700      → MISSILE
%
true_speed    = 80  + (950  - 80)  .* rand(NUM_TARGETS, 1);
true_dist     = 30  + (320  - 30)  .* rand(NUM_TARGETS, 1);
true_rcs      = 0.1 + (15.0 - 0.1) .* rand(NUM_TARGETS, 1);
true_approach = sign(rand(NUM_TARGETS, 1) - 0.4);   % 60% approaching
true_approach(true_approach == 0) = 1;

% ── STEALTH TARGET: Target 3 has very low RCS ─────────────────
% Simulates an aircraft designed to avoid radar detection
true_rcs(3) = 0.05 + rand() * 0.1;  % near-invisible on radar
fprintf('>> Stealth target assigned: TGT-3 (RCS = %.2f m²)\n', true_rcs(3));

% ── EVASIVE MANEUVER: Target 7 zigzags ────────────────────────
% We add a sinusoidal perturbation to its distance measurements
evasive_target = 7;
fprintf('>> Evasive target assigned: TGT-%d (zigzag maneuver)\n', evasive_target);

%% ── NOISE MODEL ──────────────────────────────────────────────
speed_noise_std = 15.0;
dist_noise_std  = 5.0;
rcs_noise_std   = 0.5;

%% ══════════════════════════════════════════════════════════════
%  EXTENDED KALMAN FILTER (EKF)
%  ─────────────────────────────────────────────────────────────
%  Difference from regular Kalman:
%    Regular KF → assumes target moves in straight line
%    EKF        → handles turning / accelerating / evasive targets
%
%  State vector: x = [distance; velocity; acceleration]
%  (3 states instead of 2 — adds acceleration estimation)
%
%  Non-linear transition (EKF linearises it each step):
%    dist(k+1)  = dist(k)  + vel(k)*DT + 0.5*acc(k)*DT²
%    vel(k+1)   = vel(k)   + acc(k)*DT
%    acc(k+1)   = acc(k)   (acceleration assumed constant)
%
%  Jacobian F_jac replaces fixed F matrix from regular KF
% ══════════════════════════════════════════════════════════════

% EKF noise matrices (3x3 for 3 states)
Q_ekf = diag([0.5, 1.0, 2.0]);     % Process noise (dist, vel, acc)
R_ekf = dist_noise_std^2;           % Measurement noise (scalar)
H_ekf = [1, 0, 0];                  % Observe distance only

% Initialise state and covariance for each target
x_ekf = zeros(3, NUM_TARGETS);      % [dist; vel; acc]
P_ekf = repmat(eye(3) * 100, 1, 1, NUM_TARGETS);

% Initial state: set distance from noisy measurement
noisy_dist_init = true_dist + dist_noise_std .* randn(NUM_TARGETS, 1);
for t = 1:NUM_TARGETS
    x_ekf(:, t) = [noisy_dist_init(t); 0; 0];
end

% Run EKF iterations
filtered_dist = zeros(NUM_TARGETS, 1);
dist_history  = zeros(NUM_TARGETS, NUM_STEPS);  % for plotting

for step = 1:NUM_STEPS

    % Fresh noisy measurement (add evasive zigzag for target 7)
    z_meas = true_dist + dist_noise_std .* randn(NUM_TARGETS, 1);
    z_meas(evasive_target) = z_meas(evasive_target) + ...
                             8 * sin(step * 0.8);  % zigzag offset

    for t = 1:NUM_TARGETS

        % Current state
        d   = x_ekf(1, t);
        v   = x_ekf(2, t);
        a   = x_ekf(3, t);

        % ── EKF PREDICT ──────────────────────────────────────
        % Non-linear state transition
        x_pred = [d + v*DT + 0.5*a*DT^2;
                  v + a*DT;
                  a];

        % Jacobian of transition (linearisation at current state)
        F_jac = [1, DT, 0.5*DT^2;
                 0,  1, DT;
                 0,  0,  1];

        P_pred = F_jac * P_ekf(:,:,t) * F_jac' + Q_ekf;

        % ── EKF UPDATE ───────────────────────────────────────
        y_innov      = z_meas(t) - H_ekf * x_pred;
        S_innov      = H_ekf * P_pred * H_ekf' + R_ekf;
        K_gain       = P_pred * H_ekf' / S_innov;
        x_ekf(:, t)  = x_pred + K_gain * y_innov;
        P_ekf(:,:,t) = (eye(3) - K_gain * H_ekf) * P_pred;

        % Store history for plotting
        dist_history(t, step) = x_ekf(1, t);
    end
end

% Extract final filtered distances
for t = 1:NUM_TARGETS
    filtered_dist(t) = x_ekf(1, t);
end

%% ── SPEED AND RCS NOISE ──────────────────────────────────────
noisy_speed = true_speed + speed_noise_std .* randn(NUM_TARGETS, 1);
noisy_rcs   = true_rcs   + rcs_noise_std   .* randn(NUM_TARGETS, 1);

% Clamp all values to valid physical ranges
filtered_dist = max(10,  min(320,  filtered_dist));
noisy_speed   = max(50,  min(980,  noisy_speed));
noisy_rcs     = max(0.05, min(15.0, noisy_rcs));

%% ── FORMATION DETECTION ──────────────────────────────────────
%  Two targets are in a "formation" if they are within
%  FORMATION_THRESHOLD km of each other in distance
%  AND flying in the same direction.
%
%  Practical meaning: a formation attack is more dangerous
%  because multiple threats arrive simultaneously.
%
FORMATION_THRESHOLD = 40;  % km
formation_group = zeros(NUM_TARGETS, 1);  % 0 = no group
group_id = 1;

fprintf('\n>> Formation Detection:\n');
for i = 1:NUM_TARGETS
    for j = i+1:NUM_TARGETS
        dist_diff = abs(filtered_dist(i) - filtered_dist(j));
        same_dir  = (true_approach(i) == true_approach(j));

        if dist_diff < FORMATION_THRESHOLD && same_dir
            % Assign same group ID to both
            if formation_group(i) == 0 && formation_group(j) == 0
                formation_group(i) = group_id;
                formation_group(j) = group_id;
                group_id = group_id + 1;
            elseif formation_group(i) ~= 0
                formation_group(j) = formation_group(i);
            else
                formation_group(i) = formation_group(j);
            end
            fprintf('   TGT-%d and TGT-%d are in FORMATION (dist diff = %.1f km)\n', ...
                    i, j, dist_diff);
        end
    end
end

num_formations = max(formation_group);
if num_formations == 0
    fprintf('   No formations detected.\n');
else
    fprintf('   %d formation group(s) detected.\n', num_formations);
end

%% ── WRITE input_data.txt ─────────────────────────────────────
% Format: speed(int) distance(int) rcs(1dp) approach(+1/-1)
fid = fopen(OUTPUT_FILE, 'w');
if fid == -1
    error('Cannot write to %s', OUTPUT_FILE);
end

for t = 1:NUM_TARGETS
    fprintf(fid, '%d %d %.1f %d\n', ...
        round(noisy_speed(t)),    ...
        round(filtered_dist(t)), ...
        noisy_rcs(t),            ...
        true_approach(t));
end
fclose(fid);
fprintf('\n✔  input_data.txt written (%d targets)\n', NUM_TARGETS);

%% ── CONSOLE REPORT ───────────────────────────────────────────
fprintf('\n%s\n', repmat('═', 1, 72));
fprintf('  ADIR-7  ·  TARGET ACQUISITION REPORT\n');
fprintf('%s\n', repmat('═', 1, 72));
fprintf('%-4s %-8s %-10s %-12s %-10s %-8s %-10s\n', ...
    'ID', 'Type', 'Speed', 'Distance', 'RCS', 'Dir', 'Formation');
fprintf('%s\n', repmat('-', 1, 72));

type_names = {'DRONE', 'AIRCRAFT', 'MISSILE'};
for t = 1:NUM_TARGETS
    spd = round(noisy_speed(t));
    if spd < 300,       tname = 'DRONE';
    elseif spd < 700,   tname = 'AIRCRAFT';
    else,               tname = 'MISSILE';
    end

    dir_str = 'Approach';
    if true_approach(t) == -1, dir_str = 'Recede'; end

    grp_str = '-';
    if formation_group(t) > 0
        grp_str = sprintf('GRP-%d', formation_group(t));
    end

    special = '';
    if t == 3,              special = '[STEALTH]'; end
    if t == evasive_target, special = '[EVASIVE]'; end

    fprintf('%-4d %-8s %-10d %-12d %-10.1f %-8s %-10s %s\n', ...
        t, tname, spd, round(filtered_dist(t)), ...
        noisy_rcs(t), dir_str, grp_str, special);
end
fprintf('%s\n', repmat('═', 1, 72));

%% ── PLOTS ────────────────────────────────────────────────────

% ── Plot 1: EKF Convergence (Distance Estimation) ────────────
figure('Name', 'EKF Convergence', 'Color', 'k', 'Position', [50 50 700 400]);
hold on;
colors = lines(NUM_TARGETS);
for t = 1:NUM_TARGETS
    plot(1:NUM_STEPS, dist_history(t,:), '-o', 'Color', colors(t,:), ...
         'LineWidth', 1.5, 'MarkerSize', 4);
end
yline(true_dist, '--w', 'Alpha', 0.3);
set(gca, 'Color','k','XColor','w','YColor','w');
xlabel('EKF Iteration', 'Color','w');
ylabel('Distance Estimate (km)', 'Color','w');
title('EKF Convergence — All Targets', 'Color','w', 'FontSize',12);
legend(arrayfun(@(i) sprintf('TGT-%d',i), 1:NUM_TARGETS, 'UniformOutput',false), ...
       'TextColor','w','Color','none','Location','eastoutside','FontSize',7);
grid on;

% ── Plot 2: True vs Kalman Distance Bar Chart ─────────────────
figure('Name', 'True vs EKF Distance', 'Color', 'k', 'Position', [760 50 700 400]);
hold on;
b1 = bar((1:NUM_TARGETS)-0.2, true_dist,      0.35, 'FaceColor', [0.0 0.5 1.0]);
b2 = bar((1:NUM_TARGETS)+0.2, filtered_dist,  0.35, 'FaceColor', [0.0 1.0 0.4]);
set(gca,'Color','k','XColor','w','YColor','w','GridColor',[0 1 0 0.2]);
grid on;
xlabel('Target ID','Color','w');
ylabel('Distance (km)','Color','w');
title('EKF: True vs Estimated Distance','Color','w','FontSize',12);
legend([b1 b2],{'True Distance','EKF Estimate'},'TextColor','w','Color','none');
xticks(1:NUM_TARGETS);

% ── Plot 3: Speed Distribution ────────────────────────────────
figure('Name', 'Speed Distribution', 'Color', 'k', 'Position', [50 500 700 350]);
clrs = zeros(NUM_TARGETS, 3);
for t = 1:NUM_TARGETS
    spd = round(noisy_speed(t));
    if spd < 300,      clrs(t,:) = [0.0 1.0 0.25];   % green  = drone
    elseif spd < 700,  clrs(t,:) = [1.0 0.7 0.0];    % amber  = aircraft
    else,              clrs(t,:) = [1.0 0.13 0.13];  % red    = missile
    end
end
b = bar(1:NUM_TARGETS, noisy_speed, 'FaceColor', 'flat');
b.CData = clrs;
set(gca,'Color','k','XColor','w','YColor','w');
xlabel('Target ID','Color','w');
ylabel('Speed (km/h)','Color','w');
title('Target Speed Distribution  (Green=Drone  Amber=Aircraft  Red=Missile)', ...
      'Color','w','FontSize',10);
yline(300,'--y','LineWidth',1.2); yline(700,'--r','LineWidth',1.2);
grid on; xticks(1:NUM_TARGETS);

% ── Plot 4: Formation Map (polar) ─────────────────────────────
figure('Name', 'Formation Map', 'Color', 'k', 'Position', [760 500 500 500]);
ax = polaraxes;
ax.Color            = 'k';
ax.GridColor        = [0 0.6 0.2];
ax.GridAlpha        = 0.4;
ax.TickLabelColor   = [0 1 0.25];
hold(ax, 'on');

marker_styles = {'o','s','^','d','v','p','h','x','+','*'};
for t = 1:NUM_TARGETS
    ang = (t-1) / NUM_TARGETS * 2 * pi;
    r   = filtered_dist(t);
    col = [0 1 0.25];
    if noisy_speed(t) >= 700, col = [1 0.13 0.13];
    elseif noisy_speed(t) >= 300, col = [1 0.7 0];
    end
    polarplot(ax, ang, r, marker_styles{mod(t-1,10)+1}, ...
              'Color', col, 'MarkerSize', 10, 'LineWidth', 2);

    if formation_group(t) > 0
        polarplot(ax, ang, r, 'o', 'Color', 'w', ...
                  'MarkerSize', 18, 'LineWidth', 1);
    end
    text(ax, ang, r + 15, sprintf('T%d', t), 'Color', col, 'FontSize', 8);
end
title(ax, 'Polar Formation Map  (white ring = formation)', ...
      'Color','w','FontSize',10);
