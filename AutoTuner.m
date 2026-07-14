% AutoTuner.m
clc; clear; close all; clear global;

modes = {'Default', 'Parking', 'Caution', 'Avoidance'};
results = struct();

% Ranges: [Heading, Dist, Vel, PredictTime]
% We search wide ranges to let the math decide.
range_min = [0.05, 0.1, 0.1, 1.0];
range_max = [1.0,  1.0, 1.0, 3.5];

trials = 300; % Number of tries per mode

disp('========================================');
disp('  AUTOMATIC DWA TUNER (4 MODES)  ');
disp('========================================');

for m = 1:length(modes)
    currentMode = modes{m};
    fprintf('\nProcessing Mode: %s ...\n', currentMode);
    
    best_score = inf;
    best_p = [];
    
    for i = 1:trials
        % Random parameters
        p = range_min + (range_max - range_min) .* rand(1,4);
        
        % For Avoidance/Parking, force specific prediction times?
        % Optional: You can constrain search space per mode here if you want.
        % For now, we let the optimizer find it purely randomly.
        
        score = SimulateRun(p, currentMode);
        
        if score < best_score
            best_score = score;
            best_p = p;
        end
        
        if mod(i, 100) == 0
            fprintf('  Iteration %d/%d (Best Score: %.1f)\n', i, trials, best_score);
        end
    end
    
    % Save result
    results.(currentMode) = best_p;
    fprintf('  -> Done. Best: [%.2f, %.2f, %.2f, %.1f]\n', ...
        best_p(1), best_p(2), best_p(3), best_p(4));
end

disp(' ');
disp('========================================');
disp('       FINAL PARAMETER SETTINGS         ');
disp('========================================');
disp('Copy these values into your run.mlapp code:');
disp(' ');

fprintf('%% 1. DEFAULT MODE\n');
p = results.Default;
fprintf('current_eval1 = [%.2f, %.2f, %.2f, %.1f];\n', p(1), p(2), p(3), p(4));

fprintf('\n%% 2. PARKING MODE (dist_to_goal < 1.5)\n');
p = results.Parking;
fprintf('current_eval1 = [%.2f, %.2f, %.2f, %.1f];\n', p(1), p(2), p(3), p(4));

fprintf('\n%% 3. CAUTION MODE (min_dist_obs < 2.0)\n');
p = results.Caution;
fprintf('current_eval1 = [%.2f, %.2f, %.2f, %.1f];\n', p(1), p(2), p(3), p(4));

fprintf('\n%% 4. AVOIDANCE MODE (min_dist_obs < 0.8)\n');
p = results.Avoidance;
fprintf('current_eval1 = [%.2f, %.2f, %.2f, %.1f];\n', p(1), p(2), p(3), p(4));
disp('========================================');