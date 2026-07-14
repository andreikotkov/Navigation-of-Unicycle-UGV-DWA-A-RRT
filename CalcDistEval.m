function dist = CalcDistEval(x, ob, R)
% CalcDistEval: Calculates the distance to the nearest obstacle

% Initialize with a large number 
dist = 100.0; 

if isempty(ob)
    return; % If no obstacles, return 100 
end
% ----------------------------------------------

for io = 1:size(ob, 1)
    % Calculate distance to the edge of the obstacle
    
    disttmp = norm(ob(io,:) - x(1:2)') - R;
    
    % Keep the minimum distance found
    if dist > disttmp
        dist = disttmp;
    end
end

% Clamp to 0 if collision occurred
if dist < 0
    dist = 0;
end
end