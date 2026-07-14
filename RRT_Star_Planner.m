function path = RRT_Star_Planner(start_pos, goal_pos, obstacle_list, map_limits)
    % RRT_STAR_PLANNER: Calculates path using RRT* algorithm
    % map_limits: [xmin, xmax, ymin, ymax]
    
    % --- PARAMETERS ---
    step_size = 0.5;        % Tree growth step size (meters)
    max_iter = 3000;        % Maximum iterations
    search_radius = 1.5;    % Neighbor search radius (for Rewiring)
    goal_threshold = 0.5;   % Distance threshold to accept goal
    obs_safety_margin = 0.4; % Safety margin from obstacles
    
    % Tree Initialization: [x, y, cost, parent_idx]
    tree.node = [start_pos, 0, 0]; 
    
    path = [];
    found = false;
    
    for i = 1:max_iter
        % 1. Random Point Selection (Goal Bias: 10% chance to pick goal)
        if rand < 0.1
            rnd_point = goal_pos;
        else
            rnd_point = [rand*(map_limits(2)-map_limits(1))+map_limits(1), ...
                         rand*(map_limits(4)-map_limits(3))+map_limits(3)];
        end
        
        % 2. Find Nearest Node
        dists = sqrt(sum((tree.node(:,1:2) - rnd_point).^2, 2));
        [~, nearest_idx] = min(dists);
        nearest_node = tree.node(nearest_idx, :);
        
        % 3. Steer (Move towards random point)
        theta = atan2(rnd_point(2) - nearest_node(2), rnd_point(1) - nearest_node(1));
        new_point = nearest_node(1:2) + step_size * [cos(theta), sin(theta)];
        
        % 4. Check Boundaries and Obstacles
        if new_point(1) < map_limits(1) || new_point(1) > map_limits(2) || ...
           new_point(2) < map_limits(3) || new_point(2) > map_limits(4)
           continue;
        end
        
        % Simple obstacle check (Distance based)
        obs_dists = sqrt(sum((obstacle_list - new_point).^2, 2));
        if any(obs_dists < obs_safety_margin); continue; end
        
        % 5. Choose Best Parent
        % Find neighbors within search radius
        dists_to_new = sqrt(sum((tree.node(:,1:2) - new_point).^2, 2));
        neighbors = find(dists_to_new <= search_radius);
        
        min_cost = nearest_node(3) + step_size;
        parent_idx = nearest_idx;
        
        for k = 1:length(neighbors)
            idx = neighbors(k);
            cost = tree.node(idx, 3) + norm(tree.node(idx,1:2) - new_point);
            if cost < min_cost
                % Check collision along the line
                if ~check_collision(tree.node(idx,1:2), new_point, obstacle_list)
                    min_cost = cost;
                    parent_idx = idx;
                end
            end
        end
        
        % 6. Add Node to Tree
        new_node_data = [new_point, min_cost, parent_idx];
        tree.node = [tree.node; new_node_data];
        new_node_idx = size(tree.node, 1);
        
        % 7. Rewire (Optimize Tree)
        for k = 1:length(neighbors)
            idx = neighbors(k);
            if idx == parent_idx; continue; end
            
            dist_n = norm(new_point - tree.node(idx,1:2));
            new_cost_neighbor = min_cost + dist_n;
            
            if new_cost_neighbor < tree.node(idx, 3)
                 if ~check_collision(new_point, tree.node(idx,1:2), obstacle_list)
                     tree.node(idx, 3) = new_cost_neighbor;
                     tree.node(idx, 4) = new_node_idx;
                 end
            end
        end
        
        % Check if Goal is Reached
        if norm(new_point - goal_pos) < goal_threshold
            % Backtrack path from goal to start
            path = [goal_pos];
            curr = new_node_idx;
            while curr > 0
                path = [tree.node(curr, 1:2); path];
                curr = tree.node(curr, 4);
            end
            path = [start_pos; path];
            return;
        end
    end
    
    % Fallback: Return straight line if path not found
    if isempty(path)
        disp('RRT* Path not found! Returning straight line.');
        path = [start_pos; goal_pos];
    end
end

function collision = check_collision(p1, p2, obs_list)
    collision = false;
    steps = 5; % Check 5 discrete points along the line
    for i=1:steps
        pt = p1 + (p2-p1)*(i/steps);
        d = sqrt(sum((obs_list - pt).^2, 2));
        if any(d < 0.4); collision = true; break; end
    end
end