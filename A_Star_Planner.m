function path = A_Star_Planner(start_pos, goal_pos, obstacle_list, map_limits, resolution)
    % A_STAR_PLANNER: Calculates a collision-free path using A* algorithm.
    %
    % Inputs:
    %   start_pos: [x, y]
    %   goal_pos:  [x, y]
    %   obstacle_list: [x, y] matrix of obstacles
    %   map_limits: [xmin, xmax, ymin, ymax]
    %   resolution: Grid cell size in meters
    
    % 1. Setup Grid Map
    xmin = map_limits(1); xmax = map_limits(2);
    ymin = map_limits(3); ymax = map_limits(4);
    
    width = ceil((xmax - xmin) / resolution);
    height = ceil((ymax - ymin) / resolution);
    
    % Initialize map (0 = Free, 1 = Occupied)
    map = zeros(width, height);
    
    % 2. Process Obstacles (Add Safety Margin)
    margin = 0.5; % Safety margin in meters
    margin_idx = ceil(margin / resolution);
    
    for i = 1:size(obstacle_list, 1)
        ox = obstacle_list(i,1); 
        oy = obstacle_list(i,2);
        
        % Convert world coordinates to grid indices
        idx_x = ceil((ox - xmin) / resolution);
        idx_y = ceil((oy - ymin) / resolution);
        
        % Inflate obstacle area for safety
        for mx = -margin_idx:margin_idx
            for my = -margin_idx:margin_idx
                ix = idx_x + mx; 
                iy = idx_y + my;
                if ix > 0 && ix <= width && iy > 0 && iy <= height
                    map(ix, iy) = 1; 
                end
            end
        end
    end

    % 3. Define Start and Goal Nodes
    start_node = [ceil((start_pos(1) - xmin)/resolution), ceil((start_pos(2) - ymin)/resolution)];
    goal_node  = [ceil((goal_pos(1) - xmin)/resolution), ceil((goal_pos(2) - ymin)/resolution)];
    
    % Ensure nodes are within map bounds
    start_node = max([1 1], min(start_node, [width height]));
    goal_node  = max([1 1], min(goal_node, [width height]));

    % 4. Initialize A* Lists
    openList = [];
    closedList = false(width, height);
    
    % Node structure: [x, y, g, h, f, parent_x, parent_y]
    h_start = norm(start_node - goal_node);
    startData = [start_node(1), start_node(2), 0, h_start, h_start, 0, 0];
    openList = [openList; startData];
    
    % Parent tracking
    cameFrom = zeros(width, height, 2); 
    gScore = inf(width, height);
    gScore(start_node(1), start_node(2)) = 0;
    
    % 8-connected neighbors
    neighbors = [1 0; -1 0; 0 1; 0 -1; 1 1; 1 -1; -1 1; -1 -1];
    
    found = false; 
    goal_reached_node = [];
    
    % 5. Main Search Loop
    while ~isempty(openList)
        % Get node with lowest F score
        [~, minIdx] = min(openList(:, 5));
        current = openList(minIdx, :);
        openList(minIdx, :) = []; 
        
        cx = current(1); cy = current(2);
        
        if closedList(cx, cy); continue; end
        closedList(cx, cy) = true;
        
        % Check Goal Reached
        if norm([cx cy] - goal_node) < 1.5
            found = true; 
            goal_reached_node = [cx, cy]; 
            break;
        end
        
        % Explore Neighbors
        for k = 1:8
            nx = cx + neighbors(k,1);
            ny = cy + neighbors(k,2);
            
            if nx > 0 && nx <= width && ny > 0 && ny <= height
                % Check if free and not visited
                if map(nx, ny) == 0 && ~closedList(nx, ny)
                    tentative_g = gScore(cx, cy) + norm(neighbors(k,:));
                    
                    if tentative_g < gScore(nx, ny)
                        cameFrom(nx, ny, :) = [cx, cy];
                        gScore(nx, ny) = tentative_g;
                        
                        h = norm([nx ny] - goal_node);
                        f = tentative_g + h;
                        openList = [openList; nx, ny, tentative_g, h, f, cx, cy];
                    end
                end
            end
        end
    end
    
    % 6. Reconstruct Path (Backtracking)
    path = [];
    if found
        curr = goal_reached_node;
        grid_path = curr;
        while any(curr ~= start_node)
            par = squeeze(cameFrom(curr(1), curr(2), :))';
            if all(par == 0), break; end
            grid_path = [par; grid_path];
            curr = par;
        end
        % Convert grid indices back to world meters
        path = (grid_path * resolution) + [xmin, ymin];
        path = [start_pos; path; goal_pos];
    else
        disp('A* Warning: Path not found. Returning straight line.');
        path = [start_pos; goal_pos];
    end
end