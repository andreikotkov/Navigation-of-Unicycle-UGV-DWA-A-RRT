function [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam)
%A function that calculates the evaluation value for each path
evalDB=[];
trajDB=[];

% Extract linear acceleration limit for braking calculation
max_accel = model(3); 

for vt=Vr(1):model(5):Vr(2)
    for ot=Vr(3):model(6):Vr(4)
        %Trajectory estimation
        [xt,traj]=GenerateTrajectory(x,vt,ot,evalParam(4),model);
        
        %Calculation of each evaluation function
        heading=CalcHeadingEval(xt,goal);
        dist=CalcDistEval(xt,ob,R);
        vel=abs(vt);
        
      
        % Calculate safe stopping distance: d = v^2 / (2*a)
        stopDist = (vt^2) / (2 * max_accel);
        
        % If the robot cannot stop before hitting the obstacle, penalize heavily
        if dist < stopDist
            % Penalize this path so it is extremely unlikely to be chosen
            % We don't discard it entirely to avoid empty evalDB errors
            continue; 
        end
        
        
        % Apply a small penalty to turning LEFT (ot > 0) to encourage passing
        % on the right (Traffic Rule)
        side_penalty = 0;
        if ot > 0
            side_penalty = 0.05; 
        end
        
        % Apply penalty to heading score
        heading = heading - side_penalty;

        evalDB=[evalDB;[vt ot heading dist vel]];
        trajDB=[trajDB;traj];     
    end
end
end