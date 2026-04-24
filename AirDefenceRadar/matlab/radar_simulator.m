clc; clear; close all;

NUM_TARGETS = 10;
DT          = 1.0;
NUM_STEPS   = 8;
rng(42);

OUTPUT_FILE = '../data/input_data.txt';

if ~exist('../data', 'dir')
    mkdir('../data');
    fprintf('Created folder: ../data/\n');
end

true_speed    = 80  + (950  - 80)  .* rand(NUM_TARGETS, 1);
true_dist     = 30  + (320  - 30)  .* rand(NUM_TARGETS, 1);
true_rcs      = 0.1 + (15.0 - 0.1) .* rand(NUM_TARGETS, 1);
true_approach = sign(rand(NUM_TARGETS, 1) - 0.4);
true_approach(true_approach == 0) = 1;

true_rcs(3) = 0.05 + rand() * 0.1;
fprintf('>> Stealth target assigned: TGT-3 (RCS = %.2f m²)\n', true_rcs(3));

evasive_target = 7;
fprintf('>> Evasive target assigned: TGT-%d (zigzag maneuver)\n', evasive_target);

speed_noise_std = 15.0;
dist_noise_std  = 5.0;
rcs_noise_std   = 0.5;

Q_ekf = diag([0.5, 1.0, 2.0]);
R_ekf = dist_noise_std^2;
H_ekf = [1, 0, 0];

x_ekf = zeros(3, NUM_TARGETS);
P_ekf = repmat(eye(3) * 100, 1, 1, NUM_TARGETS);

noisy_dist_init = true_dist + dist_noise_std .* randn(NUM_TARGETS, 1);
for t = 1:NUM_TARGETS
    x_ekf(:, t) = [noisy_dist_init(t); 0; 0];
end

filtered_dist = zeros(NUM_TARGETS, 1);
dist_history  = zeros(NUM_TARGETS, NUM_STEPS);

for step = 1:NUM_STEPS

    z_meas = true_dist + dist_noise_std .* randn(NUM_TARGETS, 1);
    z_meas(evasive_target) = z_meas(evasive_target) + 8 * sin(step * 0.8);

    for t = 1:NUM_TARGETS

        d   = x_ekf(1, t);
        v   = x_ekf(2, t);
        a   = x_ekf(3, t);

        x_pred = [d + v*DT + 0.5*a*DT^2;
                  v + a*DT;
                  a];

        F_jac = [1, DT, 0.5*DT^2;
                 0,  1, DT;
                 0,  0,  1];

        P_pred = F_jac * P_ekf(:,:,t) * F_jac' + Q_ekf;

        y_innov      = z_meas(t) - H_ekf * x_pred;
        S_innov      = H_ekf * P_pred * H_ekf' + R_ekf;
        K_gain       = P_pred * H_ekf' / S_innov;
        x_ekf(:, t)  = x_pred + K_gain * y_innov;
        P_ekf(:,:,t) = (eye(3) - K_gain * H_ekf) * P_pred;

        dist_history(t, step) = x_ekf(1, t);
    end
end

for t = 1:NUM_TARGETS
    filtered_dist(t) = x_ekf(1, t);
end

noisy_speed = true_speed + speed_noise_std .* randn(NUM_TARGETS, 1);
noisy_rcs   = true_rcs   + rcs_noise_std   .* randn(NUM_TARGETS, 1);

filtered_dist = max(10,  min(320,  filtered_dist));
noisy_speed   = max(50,  min(980,  noisy_speed));
noisy_rcs     = max(0.05, min(15.0, noisy_rcs));

FORMATION_THRESHOLD = 40;
formation_group = zeros(NUM_TARGETS, 1);
group_id = 1;

fprintf('\n>> Formation Detection:\n');
for i = 1:NUM_TARGETS
    for j = i+1:NUM_TARGETS
        dist_diff = abs(filtered_dist(i) - filtered_dist(j));
        same_dir  = (true_approach(i) == true_approach(j));

        if dist_diff < FORMATION_THRESHOLD && same_dir
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

figure('Name', 'Speed Distribution', 'Color', 'k', 'Position', [50 500 700 350]);
clrs = zeros(NUM_TARGETS, 3);
for t = 1:NUM_TARGETS
    spd = round(noisy_speed(t));
    if spd < 300,      clrs(t,:) = [0.0 1.0 0.25];
    elseif spd < 700,  clrs(t,:) = [1.0 0.7 0.0];
    else,              clrs(t,:) = [1.0 0.13 0.13];
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
