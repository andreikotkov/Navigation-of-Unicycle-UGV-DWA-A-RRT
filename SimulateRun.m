function score = SimulateRun(testParams, modeType)
    % SimulateRun: Runs DWA in specific scenarios
    % testParams = [Heading, Dist, Vel, PredictTime]
    
  
    global dt; 
    dt = 0.1;
    
    
    % Kinematic limits: [v_max, w_max, a_max, alpha_max, dv, dw]
    % using (val * pi / 180) 
    Kinematic = [2.0, (40 * pi / 180), 0.4, (80 * pi / 180), 0.03, (3 * pi / 180)];
    obstacleR = 0.5;
    
    % --- SCENARIO SETUP ---
    switch modeType
        case 'Default'
            x = [0 0 0 0 0]'; 
            goal = [10 10]; 
            obstacle = []; 
            success_dist = 0.5;
        case 'Parking'
            x = [8 8 0 0 0]'; 
            goal = [10 10]; 
            obstacle = [];
            success_dist = 0.1;
        case 'Caution'
            x = [0 0 0 0 0]'; 
            goal = [10 0];
            obstacle = [5 0; 5 1; 5 -1]; 
            success_dist = 0.5;
        case 'Avoidance'
            x = [4 0 0 0 0]'; 
            goal = [10 0];
            obstacle = [5 2; 5 1.5; 5 1; 5 -1; 5 -1.5; 5 -2]; 
            success_dist = 0.5;
        otherwise
            error('Unknown modeType: %s', modeType);
    end

    reached = false;
    collision = false;
    steps = 0;
    max_steps = 1000;
    
    % --- SIMULATION LOOP ---
    try
        for i = 1:max_steps
            steps = i;
            
            % Check Goal
            if norm(x(1:2)' - goal) < success_dist
                reached = true; break; 
            end
            
            % Run DWA
            [u, ~] = DynamicWindowApproach(x, Kinematic, goal, testParams, obstacle, obstacleR);
            
            % Motion Update
            x = f(x, u);
            
            % Check Collision
            if ~isempty(obstacle)
                dists = sqrt(sum((obstacle - x(1:2)').^2, 2));
                if any(dists < obstacleR)
                    collision = true; break; 
                end
            end
            
            % Check Map Bounds
            if x(1)<-2 || x(1)>12 || x(2)<-2 || x(2)>12
                collision = true; break; 
            end
        end
    catch ME
        % Error Handling: If this crashes, print WHY
        fprintf('Error inside SimulateRun loop!\n');
        fprintf('Identifier: %s\n', ME.identifier);
        fprintf('Message: %s\n', ME.message);
        rethrow(ME);
    end
    
    % --- SCORING ---
    if collision
        score = 5000; 
    elseif ~reached
        score = 2000 + norm(x(1:2)' - goal)*10; 
    else
        score = steps;
        if strcmp(modeType, 'Parking')
            score = score + (x(4)*100); 
        elseif strcmp(modeType, 'Caution')
            min_obs_dist = min(sqrt(sum((obstacle - x(1:2)').^2, 2)));
            if min_obs_dist < 1.0
                score = score + 500; 
            end
        end
    end
end