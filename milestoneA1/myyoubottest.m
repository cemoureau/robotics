function myyoubottest()

    % Myyoubottest illustrates the V-REP Matlab bindings.
    % (C) Copyright Quentin Budo, Céline Moureau, Arnaud Paquet.
   
    %% Initiate the connection to the simulator.    
    disp('Program started');
    vrep = remApi('remoteApi');
    vrep.simxFinish(-1);
    id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);

    if id < 0
        disp('Failed connecting to remote API server. Exiting.');
        vrep.delete();
        return;
    end
    fprintf('Connection %d to remote API server open.\n', id);

    % Make sure we close the connection whenever the script is interrupted.
    cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

    % This will only work in "continuous remote API server service". 
    vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

    % Retrieve all handles, and stream arm and wheel joints, the robot's pose, the Hokuyo, and the arm tip pose. 
    h = youbot_init(vrep, id);
    h = youbot_hokuyo_init(vrep, h);

    % Let a few cycles pass to make sure there's a value waiting for us next time we try to get a joint angle or 
    % the robot pose with the simx_opmode_buffer option.
    pause(.2);

    %% Youbot constants.
    % The time step the simulator is using (your code should run close to it). 
    timestep = .05;

    % Definition of the starting pose of the arm (the angle to impose at each joint to be in the rest position).
    startingJoints = [0, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];
    
    %% Initial values.
    disp('Starting robot');
    
    % Parameters for controlling the youBot's wheels: at each iteration, those values will be set for the wheels. 
    % They are adapted at each iteration by the code. 
    forwBackVel = 0; % Move straight ahead. 
    rightVel = 0; % Go sideways. 
    rotateRightVel = 0; % Rotate. 
    prevOrientation = 0; % Previous angle to goal (easy way to have a condition on the robot's angular speed). 
    prevPosition = 0; % Previous distance to goal (easy way to have a condition on the robot's speed). 

    % Set the arm to its starting configuration. 
    res = vrep.simxPauseCommunication(id, true); % Send order to the simulator through vrep object. 
    vrchk(vrep, res); % Check the return value from the previous V-REP call (res) and exit in case of error.
    
    % Set the target position of a joint.
    for i = 1:5
        res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), startingJoints(i), vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
    end    
    res = vrep.simxPauseCommunication(id, false); 
    vrchk(vrep, res);
    
    % Initialise save matrix for navigation. 
    initTraj = true;
    minMap = [0 0];
    maxMap = [0 0];
    minMapInter = [0 0];
    maxMapInter = [0 0];
    NewGoal = [0 0];
    resol = 4; % 25 cm.
    
    % MaxDist is the distance after which we reach the maximum speed.
    maxSpeed = 12.5;
    maxDist = 25;
    % Initial square map of lenght 12m.
    totalMap = ones(12*resol+1);
    
    % Grid to store the points from the area that the hokuyo captor sees.
    distBeam = 5*resol+2;
    [Xmap, Ymap] = meshgrid(-distBeam:1:distBeam, -distBeam:1:distBeam);
    Xmap = reshape(Xmap, 1, []);
    Ymap = reshape(Ymap, 1, []);

    % Initialise the state machine. 
    fsm = 'start';
    step = 'navigation';
    
    % Make sure everything is settled before we start. 
    pause(2);
    
    %% Start simulation. 
    while true
        tic % See end of loop to see why it's useful. 
        
        if vrep.simxGetConnectionId(id) == -1
            error('Lost connection to remote API.');
        end
    
        % Get the position and the orientation of the robot. 
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);

        %% NAVIGATION
        if strcmp(step, 'navigation')            
            if strcmp(fsm, 'start')
                % Read data from the depth sensor, more often called the Hokuyo 
                % This function returns the set of points the Hokuyo saw in pts. contacts indicates, for each point, if it
                % corresponds to an obstacle (the ray the Hokuyo sent was interrupted by an obstacle, and was not allowed to
                % go to infinity without being stopped(stopped at a distance of 5m).
                trf = transl(youbotPos) * trotx(youbotEuler(1)) * troty(youbotEuler(2)) * trotz(youbotEuler(3));
                worldHokuyo1 = homtrans(trf, [h.hokuyo1Pos(1); h.hokuyo1Pos(2); h.hokuyo1Pos(3)]);
                worldHokuyo2 = homtrans(trf, [h.hokuyo2Pos(1); h.hokuyo2Pos(2); h.hokuyo2Pos(3)]);
                % Use the sensor to detect the visible points, within the world frame. 
                [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer, trf);
                
                % Parameters to update the map
                PositionUpdateMap = [youbotPos(1) youbotPos(2)]; 
                AngleUpdateMap = youbotEuler(3); 
                
                % Initialize map extremities.
                minMapNew = floor(resol*[min([worldHokuyo1(1), pts(1, :), worldHokuyo2(1)])...
                    min([worldHokuyo1(2), pts(2, :), worldHokuyo2(2)])]);
                maxMapNew = ceil(resol*[max([worldHokuyo1(1), pts(1, :), worldHokuyo2(1)])...
                    max([worldHokuyo1(2), pts(2, :), worldHokuyo2(2)])]);
                
                minMap = [round((maxMapNew(1) + minMapNew(1))/2) - 6*resol,...
                    round((maxMapNew(2) + minMapNew(2))/2) - 6*resol];
                maxMap = [round((maxMapNew(1) + minMapNew(1))/2) + 6*resol,...
                    round((maxMapNew(2) + minMapNew(2))/2) + 6*resol];
                 
                inMap = inpolygon(Xmap, Ymap,...
                    resol*[worldHokuyo1(1), pts(1, :), worldHokuyo2(1)]- ...
                    round(resol*youbotPos(1)),...
                    resol*[worldHokuyo1(2), pts(2, :), worldHokuyo2(2)]- ...
                    round(resol*youbotPos(2)));

                NewVisited = [transpose(Xmap(inMap) - minMap(1) + ...
                    round(resol*youbotPos(1)) + 1)...
                    transpose(Ymap(inMap) - minMap(2) + ...
                    round(resol*youbotPos(2)) + 1);...
                    (round(resol*worldHokuyo1(1))- minMap(1)+1)...
                    (round(resol*worldHokuyo1(2))- minMap(2)+1);...
                    (round(resol*youbotPos(1))- minMap(1)+1)...
                    (round(resol*youbotPos(2))- minMap(2)+1)];

                % Add visited area.
                for i = 1 : length(NewVisited)
                    totalMap(NewVisited(i,1), NewVisited(i,2)) = 0;
                end
                
                % Vector containing the position of the obstacles.
                NewObstacle = bsxfun(@minus, unique(round(resol*[transpose(pts(1, contacts)) ...
                    transpose(pts(2, contacts))]),'rows'), minMap-1); 
                
                % Add walls.
                for i = 1 : length(NewObstacle)
                    if totalMap(NewObstacle(i,1), NewObstacle(i,2)) ~= 2  
                        totalMap(NewObstacle(i,1), NewObstacle(i,2)) = 2;
                        if totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)-1) ~= 2
                            totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)-1) = 3;
                        end
                        if totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)) ~= 2
                            totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)) = 3;
                        end
                        if totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)+1) ~= 2
                            totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)+1) = 3;
                        end
                        if totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)-1) ~= 2
                            totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)-1) = 3;
                        end
                        if totalMap(NewObstacle(i,1), NewObstacle(i,2)-1) ~= 2
                            totalMap(NewObstacle(i,1), NewObstacle(i,2)-1) = 3;
                        end
                        if totalMap(NewObstacle(i,1), NewObstacle(i,2)+1) ~= 2
                            totalMap(NewObstacle(i,1), NewObstacle(i,2)+1) = 3;
                        end
                        if totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)-1) ~= 2
                            totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)-1) = 3;
                        end
                        if totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)) ~= 2
                            totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)) = 3;
                        end
                        if totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)+1) ~= 2
                            totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)+1) = 3;
                        end
                    end                   
                end
                
                % Plot of the total map.
                figure;
                imagesc(totalMap)
                axis equal
                    
                % begin the displacement by a rotation of180 degree
                trajectory = [(resol*youbotPos(1)-minMap(1)+1) (resol*youbotPos(2)-minMap(2)+1);...
                    (round(resol*(youbotPos(1)))-minMap(1)+1) (round(resol*(youbotPos(2)))-minMap(2)+1)];
                posTrajectory = 1;
                NewAngl = youbotEuler(3)+pi+0.05;
   
                fsm = 'rotate';
            
            % If initialisation is finished, update.
            else
               
                % Update afer a certain distance or a certain rotation
                if (sqrt((youbotPos(1)-PositionUpdateMap(1))^2 + (youbotPos(2)-PositionUpdateMap(2))^2)>= 5/(7*resol))...
                    ||(abs(angdiff(AngleUpdateMap - youbotEuler(3))) > (pi/5))

                    PositionUpdateMap = [youbotPos(1) youbotPos(2)];
                    AngleUpdateMap = youbotEuler(3);
                    
                    trf = transl(youbotPos) * trotx(youbotEuler(1)) * troty(youbotEuler(2)) * trotz(youbotEuler(3));
                    worldHokuyo1 = homtrans(trf, [h.hokuyo1Pos(1); h.hokuyo1Pos(2); h.hokuyo1Pos(3)]);
                    worldHokuyo2 = homtrans(trf, [h.hokuyo2Pos(1); h.hokuyo2Pos(2); h.hokuyo2Pos(3)]);
                    % Use the sensor to detect the visible points, within the world frame. 
                    [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer, trf);
                    %update map extremity
                    minMapNew = round(resol*[min([worldHokuyo1(1), pts(1, :), worldHokuyo2(1)])...
                        min([worldHokuyo1(2), pts(2, :), worldHokuyo2(2)])]) - 1;
                    maxMapNew = round(resol*[max([worldHokuyo1(1), pts(1, :), worldHokuyo2(1)])...
                        max([worldHokuyo1(2), pts(2, :), worldHokuyo1(2)])]) + 1;
                    % if map need to be modified
                    if (minMapNew(1) < minMap(1))||(minMapNew(2) < minMap(2))||...
                    (maxMapNew(1) > maxMap(1))||(maxMapNew(2) > maxMap(2))
                        % modify min x
                        if (minMapNew(1) < minMap(1))
                            minMapInter(1) = minMapNew(1) - 4*resol;
                        else
                            minMapInter(1) = minMap(1);
                        end
                        % modify min y
                        if (minMapNew(2) < minMap(2))
                            minMapInter(2) = minMapNew(2) - 4*resol;
                        else
                            minMapInter(2) = minMap(2);
                        end
                        % modify max x
                        if (maxMapNew(1) > maxMap(1))
                            maxMapInter(1) = maxMapNew(1) + 4*resol;
                        else
                            maxMapInter(1) = maxMap(1);
                        end
                        % modify max y
                        if (maxMapNew(2) > maxMap(2))
                            maxMapInter(2) = maxMapNew(2) + 4*resol;
                        else
                            maxMapInter(2) = maxMap(2);
                        end
                        %update totalMap
                        totalMap = horzcat(ones(size(totalMap,1), minMap(2) - minMapInter(2)),...
                            totalMap, ones(size(totalMap,1), maxMapInter(2) - maxMap(2)));
                        totalMap = vertcat(ones(minMap(1) - minMapInter(1), size(totalMap,2)),...
                            totalMap, ones(maxMapInter(1) - maxMap(1), size(totalMap,2)));
                        % update trajectory
                        trajectory = bsxfun(@plus,trajectory, minMap - minMapInter);
                        % update minMap and maxMap
                        minMap = minMapInter;
                        maxMap = maxMapInter;
                    end

                    % update walls
                    NewObstacle = bsxfun(@minus, unique(round(resol*[transpose(pts(1, contacts)) ...
                        transpose(pts(2, contacts))]),'rows'), minMap-1);

                    for i = 1 : length(NewObstacle)
                        if totalMap(NewObstacle(i,1), NewObstacle(i,2)) ~= 2  
                            totalMap(NewObstacle(i,1), NewObstacle(i,2)) = 2;
                            if totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)-1) ~= 2
                                totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)-1) = 3;
                            end
                            if totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)) ~= 2
                                totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)) = 3;
                            end
                            if totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)+1) ~= 2
                                totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)+1) = 3;
                            end
                            if totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)-1) ~= 2
                                totalMap(NewObstacle(i,1)-1, NewObstacle(i,2)-1) = 3;
                            end
                            if totalMap(NewObstacle(i,1), NewObstacle(i,2)-1) ~= 2
                                totalMap(NewObstacle(i,1), NewObstacle(i,2)-1) = 3;
                            end
                            if totalMap(NewObstacle(i,1), NewObstacle(i,2)+1) ~= 2
                                totalMap(NewObstacle(i,1), NewObstacle(i,2)+1) = 3;
                            end
                            if totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)-1) ~= 2
                                totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)-1) = 3;
                            end
                            if totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)) ~= 2
                                totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)) = 3;
                            end
                            if totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)+1) ~= 2
                                totalMap(NewObstacle(i,1)+1, NewObstacle(i,2)+1) = 3;
                            end
                        end                   
                    end

                    %update visited area
                    inMap = inpolygon(Xmap, Ymap,...
                       resol*[worldHokuyo1(1), pts(1, :), worldHokuyo2(1)]- ...
                       round(resol*youbotPos(1)),...
                       resol*[worldHokuyo1(2), pts(2, :), worldHokuyo2(2)]- ...
                       round(resol*youbotPos(2)));

                    NewVisited = [transpose(Xmap(inMap) - minMap(1) + ...
                        round(resol*youbotPos(1)) + 1)...
                        transpose(Ymap(inMap) - minMap(2) + ...
                        round(resol*youbotPos(2)) + 1);...
                        (round(resol*worldHokuyo1(1))- minMap(1)+1)...
                        (round(resol*worldHokuyo1(2))- minMap(2)+1);...
                        (round(resol*youbotPos(1))- minMap(1)+1)...
                        (round(resol*youbotPos(2))- minMap(2)+1)];

                    %add visited area
                    for i = 1 : length(NewVisited)
                        if totalMap(NewVisited(i,1), NewVisited(i,2)) == 1
                            totalMap(NewVisited(i,1), NewVisited(i,2)) = 0;
                        end
                    end
                        
                % Move
                elseif strcmp(fsm, 'rotate')            
                    % The rotation velocity depends on the difference between the current angle and the target. 
                    rotateRightVel = angdiff(NewAngl, youbotEuler(3));

                    % When the rotation is done (with a sufficiently high precision), move on to the next state. 
                    if (abs(angdiff(NewAngl, youbotEuler(3))) < .5 / 180 * pi) && ...
                            (abs(angdiff(prevOrientation, youbotEuler(3))) < .05 / 180 * pi)
                        rotateRightVel = 0;
                        fsm = 'drive';
                        prevPosition = [youbotPos(1) youbotPos(2)];
                        interPos = [resol*youbotPos(1) resol*youbotPos(2)];
                        initialDist = sqrt((trajectory(posTrajectory,1) - (resol*youbotPos(1)-minMap(1)+1))^2 +...
                            (trajectory(posTrajectory,2) - (resol*youbotPos(2)-minMap(2)+1))^2);
                    end                 
                    prevOrientation = youbotEuler(3);

                elseif strcmp(fsm, 'drive')

                    % DistPrev : distance between the position of the robot and the previous point.
                    % InitialDist : distance between the previous point and the next point.
                    DistPrev = sqrt((interPos(1) - resol*youbotPos(1))^2 +...
                        (interPos(2) - resol*youbotPos(2))^2);
                   
                    % To satisfy conditions imposed by mechanics about the speed, we choose to accelerate 
                    % until a certain length called maxDist and after this length we keep a constant speed. 
                    % The decceleration use the same principle.
                    
                    % If the distance is too short, we can't go to the maximum speed.
                    if DistPrev <= initialDist   
                        if initialDist < 2*maxDist
                            middleDist = initialDist/2;
                            
                            % Acceleration.
                            forwBackVel = -maxSpeed/maxDist * DistPrev;

                            % Decceleration.
                            if DistPrev > middleDist
                                forwBackVel = -maxSpeed/maxDist * (maxDist-DistPrev);
                            end
                            
                        else
                            if DistPrev < maxDist
                                forwBackVel = -maxSpeed/maxDist * DistPrev;
                            elseif DistPrev < initialDist - maxDist
                                forwBackVel = -maxSpeed;
                            else
                                forwBackVel = -maxSpeed/maxDist * (maxDist-DistPrev);
                            end
                        end
                        if (DistPrev - initialDist) < 2*maxDist
                            middleDist = (DistPrev - initialDist)/2;
                            % Acceleration
                            forwBackVel = maxSpeed/maxDist * (maxDist - (DistPrev - initialDist));

                            % Decceleration
                            if (DistPrev - initialDist) < middleDist
                                forwBackVel = maxSpeed/maxDist * (DistPrev - initialDist);
                            end 
                        else                        
                            if (DistPrev - initialDist) < maxDist
                                forwBackVel = maxSpeed/maxDist * (DistPrev - initialDist);
                            elseif (DistPrev - initialDist) < (DistPrev - initialDist) - maxDist
                                forwBackVel = maxSpeed;
                            else
                                forwBackVel = maxSpeed/maxDist * (maxDist - (DistPrev - initialDist));
                            end
                        end
                    end

                    % If the goal is reached stop the robot.
                    if (abs(initialDist - DistPrev) < 0.01) && (resol*sqrt((prevPosition(1)-youbotPos(1))^2 + ...
                            (prevPosition(2)-youbotPos(2))^2) < 0.005)
                        forwBackVel = 0;
                        
                        if posTrajectory == size(trajectory,1)
                            fsm ='newdestination';     
                        else
                            posTrajectory = posTrajectory +1;
                            NewAngl = atan(((resol*youbotPos(1)-minMap(1)+1)-trajectory(posTrajectory,1))/...
                                (trajectory(posTrajectory,2)-(resol*youbotPos(2)-minMap(2)+1)));
                            if (trajectory(posTrajectory,2)-(resol*youbotPos(2)-minMap(2)+1)) > 0
                                NewAngl = NewAngl + pi;
                            end
                            prevOrientation = youbotEuler(3);
                            fsm = 'rotate';                       
                        end
                    end
                    prevPosition = [youbotPos(1) youbotPos(2)];

                elseif strcmp(fsm, 'newdestination')
                    
                    figure;
                    imagesc(totalMap)
                    axis equal
                     
                    %define new goal
                    totalBoundariesMap = sparse([],[],[],double(maxMap(1)-minMap(1)+1), double(maxMap(2)-minMap(2)+1),0);
                    for i = 2 : size(totalMap, 1)-1             
                        for j = 2 : size(totalMap, 2)-1               
                            % if visited point
                            if totalMap(i,j) == 0
                                %if in contact with not yet visited point
                                if((totalMap(i-1,j) == 1)||(totalMap(i+1,j) == 1)|| ...
                                        (totalMap(i,j-1) == 1)||(totalMap(i,j+1) == 1))
                                    totalBoundariesMap = totalBoundariesMap + ...
                                        sparse(double(i),double(j), 1, double(maxMap(1)-minMap(1)+1),...
                                        double(maxMap(2)-minMap(2)+1));
                                end
                            end
                        end
                    end
                    
                    % extract rows and colums from totalBoundariesMap
                    [remRow, remCol] = find(totalBoundariesMap == 1);
                    
                     % If the map is completely explored, navigation is finished.
                    if isempty(remRow)
                        step = 'finished';
                    else
                        NewStart = [resol*youbotPos(1)-minMap(1)+1, resol*youbotPos(2)-minMap(2)+1];
                        
                        % find destination with max distance for the first
                        % trajectory and min for the other
                        if initTraj
                            indexGoal = find(bsxfun(@hypot, remRow-NewStart(1), remCol-NewStart(2))==...
                                max(bsxfun(@hypot, remRow-NewStart(1), remCol-NewStart(2))),1,'first');
                            initTraj = false;
                        else
                            indexGoal = find(bsxfun(@hypot, remRow-NewStart(1), remCol-NewStart(2))==...
                                min(bsxfun(@hypot, remRow-NewStart(1), remCol-NewStart(2))),1,'first');
                        end
                     
                        NewGoal = [remCol(indexGoal) remRow(indexGoal)];
                        NewStart = [round(resol*(youbotPos(1)))-minMap(1)+1, round(resol*(youbotPos(2)))-minMap(2)+1];
                        
                        % We chose Dstar because it is faster than the others.
                        figure;
                        ds = Dstar(totalMap);    % create navigation object
                        ds.plan(NewGoal)       % create plan for specified goal
                        trajectory = ds.path(flip(NewStart));     % animate path from this start location
                        ds.path(flip(NewStart)) 

                        % to have x than y 
                        trajectory = flip(trajectory,2); 
                        if size(trajectory,1)>1
                            
                            trajectory2 = zeros(size(trajectory,1), 1);

                            if ((trajectory(1,1)- NewStart(1)) ~= (trajectory(2,1)-trajectory(1,1)))||...
                                    ((trajectory(1,2)- NewStart(2)) ~= (trajectory(2,2)-trajectory(1,2)))
                                    trajectory2(1) = 1;
                            end
                            for i = 2: length(trajectory)-1
                                if ((trajectory(i,1)-trajectory(i-1,1)) ~= (trajectory(i+1,1)-trajectory(i,1)))||...
                                    ((trajectory(i,2)-trajectory(i-1,2)) ~= (trajectory(i+1,2)-trajectory(i,2)))
                                    trajectory2(i) = 1;
                                end
                            end
                            trajectory2(end-1) = 1;
                            trajectory2(end) = 0;
                            trajectory = trajectory(trajectory2 == 1, :);
                        end
                        
                        posTrajectory = 1;
                        NewAngl = atan(((resol*youbotPos(1)-minMap(1)+1)-trajectory(1,1))/...
                            (trajectory(1,2)-(resol*youbotPos(2)-minMap(2)+1)));
                        if (trajectory(1,2)-(resol*youbotPos(2)-minMap(2)+1)) > 0
                            NewAngl = NewAngl + pi;
                        end  
                        fsm = 'rotate';
                    end        

                 % End of navigation.
                 elseif  strcmp(fsm, 'finished')  
                    pause(1);
                    fsm = 'start';
                    step = 'finished';
                    fprintf('Switching to step: %s\n', step);
                 else
                    fsm = 'finished';
                    error('Unknown state %s.', fsm);
                 end
            end
        % Exit the function.
        elseif strcmp(step, 'finished')
            pause(3);
            break;
        else
            error('Unknown state %s.', fsm);
        end
        
        % Update wheel velocities using the global values (whatever the state is). 
        h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel);
        drawnow;
        % Make sure that we do not go faster than the physics simulation (each iteration must take roughly 50 ms). 
        elapsed = toc;
        timeleft = timestep - elapsed;
        if timeleft > 0
            pause(min(timeleft, .01));
        end
    end    
end % Main function.