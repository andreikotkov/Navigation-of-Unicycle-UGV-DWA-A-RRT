function heading=CalcHeadingEval(x,goal)
% A function that calculates the evaluation function of heading

% 1. Calculate the angle to the goal (in Radians)
targetTheta = atan2(goal(2) - x(2), goal(1) - x(1));

% 2. Get current robot orientation
currentTheta = x(3);

% 3. Calculate error and wrap it to [-pi, pi]
error = targetTheta - currentTheta;
error = atan2(sin(error), cos(error)); % This fixes the 180 degree crossing

% 4. Score: Higher is better (Max pi, Min 0)
heading = pi - abs(error);
end