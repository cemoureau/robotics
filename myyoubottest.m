    function myyoubottest()

    % youbot Illustrates the V-REP Matlab bindings.

    % (C) Copyright Renaud Detry 2013, Thibaut Cuvelier 2017, Mathieu Baijot 2017.
    % Distributed under the GNU General Public License.
    % (See http://www.gnu.org/copyleft/gpl.html)
   
    %% Initiate the connection to the simulator. 
    
    disp('Program started');
    % Use the following line if you had to recompile remoteApi
    %vrep = remApi('remoteApi', 'extApi.h');
    vrep = remApi('remoteApi');
    vrep.simxFinish(-1);
    id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
    
    % If you get an error like: 
    %  Remote API function call returned with error code: 64. Explanation: simxStart was not yet called.
    % Make sure your code is within a function! You cannot call V-REP from a script. 

    if id < 0
        disp('Failed connecting to remote API server. Exiting.');
        vrep.delete();
        return;
    end
    fprintf('Connection %d to remote API server open.\n', id);

    % Make sure we close the connection whenever the script is interrupted.
    cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

    % This will only work in "continuous remote API server service". 
    % See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
    vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

    % Retrieve all handles, and stream arm and wheel joints, the robot's pose, the Hokuyo, and the arm tip pose. 
    h = youbot_init(vrep, id);
    h = youbot_hokuyo_init(vrep, h);

    % Let a few cycles pass to make sure there's a value waiting for us next time we try to get a joint angle or 
    % the robot pose with the simx_opmode_buffer option.
    pause(.2);

    %% Youbot constants
    % The time step the simulator is using (your code should run close to it). 
    timestep = .05;

    % Minimum and maximum angles for all joints. Only useful to implement custom IK. 
    armJointRanges = [-2.9496064186096, 2.9496064186096;
                      -1.5707963705063, 1.308996796608;
                      -2.2863812446594, 2.2863812446594;
                      -1.7802357673645, 1.7802357673645;
                      -1.5707963705063, 1.5707963705063 ];

    % Definition of the starting pose of the arm (the angle to impose at each joint to be in the rest position).
    startingJoints = [0, 30.91 * pi / 180, 52.42 * pi / 180, 72.68 * pi / 180, 0];
    
    
    %% initial values 
    disp('Starting robot');
    
    % Define the preset pickup pose for this demo. 
    pickupJoints = [90 * pi / 180, 19.6 * pi / 180, 113 * pi / 180, - 41 * pi / 180, 0];

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
    
    % Set the target position of a joint
    for i = 1:5
        res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), startingJoints(i), vrep.simx_opmode_oneshot);
        vrchk(vrep, res, true);
    end
    
    res = vrep.simxPauseCommunication(id, false); 
    vrchk(vrep, res);

    % Initialise the plot. 
    plotData = true;
    if plotData
    end
    % Make sure everything is settled before we start. 
    pause(2);

    % Retrieve the position of the gripper. 
    [res, homeGripperPosition] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    
    % Initialise save matrix for navigation
    SaveMapObstacles = zeros(0, 2);
    SaveMapBoundaries = zeros(0, 2);
    val = 0;
    minMap = [0 0];
    maxMap = [0 0];
    resol = 25;
    % Initialise the state machine. 
    fsm = 'rotate';
    step = 'navigation';
    
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
            % Read data from the depth sensor, more often called the Hokuyo 
            % This function returns the set of points the Hokuyo saw in pts. contacts indicates, for each point, if it
            % corresponds to an obstacle (the ray the Hokuyo sent was interrupted by an obstacle, and was not allowed to
            % go to infinity without being stopped(stopped at a distance of 5m).
            trf = transl(youbotPos) * trotx(youbotEuler(1)) * troty(youbotEuler(2)) * trotz(youbotEuler(3));
            %worldHokuyo1 = homtrans(trf, [h.hokuyo1Pos(1); h.hokuyo1Pos(2); h.hokuyo1Pos(3)]);
            %worldHokuyo2 = homtrans(trf, [h.hokuyo2Pos(1); h.hokuyo2Pos(2); h.hokuyo2Pos(3)]);

            % Use the sensor to detect the visible points, within the world frame. 
            [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer, trf);
            %tolerance of 5 cm
            SaveMapObstacles = unique(union(SaveMapObstacles,...
                round(resol*[transpose(pts(1, contacts)) transpose(pts(2, contacts))]),'rows'),'rows');
            
%             SaveNewObstacles = unique(setxor(intersect(SaveMapObstacles,...
%                 round(resol*[transpose(pts(1, contacts)) transpose(pts(2, contacts))]),'rows'),...
%                 round(resol*[transpose(pts(1, contacts)) transpose(pts(2, contacts))]), ...
%                 'rows'),'rows');
            
            a = size(SaveMapObstacles)
            SaveMapBoundaries = unique(union(SaveMapBoundaries,...
                round(resol*[transpose(pts(1, :)) transpose(pts(2, :))]),'rows'),'rows');
%             SaveMapBoundaries = unique(union(SaveMapBoundaries,...
%                 round(50*[transpose([h.hokuyo1Pos(1), pts(1, :), h.hokuyo2Pos(1)]) ...
%                 transpose([h.hokuyo1Pos(2), pts(2, :), h.hokuyo2Pos(2)])])/50,'rows'),'rows');
            b = size(SaveMapBoundaries)

            if plotData
                
                minMapNew = min(SaveMapObstacles, [], 1)-1;
                maxMapNew = max(SaveMapObstacles, [], 1)+1;
                
                if (minMapNew(1) < minMap(1))||(minMapNew(2) < minMap(2))||...
                        (maxMapNew(1) < maxMap(1))||(maxMapNew(2) < maxMap(2))
                    minMap = minMapNew;
                    maxMap = maxMapNew;
                    %map = sparse(double(maxMap(1)-minMap(1)), double(maxMap(2)-minMap(2)), 1);
                    map = sparse(double(SaveMapObstacles(:,1)-minMap(1)), double(SaveMapObstacles(:,2)-minMap(2)), 1);
%                     map = zeros(maxMap(1) - minMap(1), maxMap(2) - minMap(2));
%                     map(round(resol*SaveMapObstacles(:,1)), round(resol*SaveMapObstacles(:,2)))= 1;
                   
                else
                    %map(round(resol*SaveNewObstacles(:,1)), round(resol*SaveNewObstacles(:,2)))= 1;
                    map = sparse(double(SaveMapObstacles(:,1)-minMap(1)), double(SaveMapObstacles(:,2)-minMap(2)), 1);
%                   
                end
                imagesc(map)
                axis equal
                drawnow;
                
%                 % Create a 2D mesh of points, stored in the vectors X and Y. This will be used to display the area the robot can
%                 % see, by selecting the points within this mesh that are within the visibility range. 
%                 [X, Y] = meshgrid(-10:.02:10, -10:.02:10); % Values selected for the area the robot will explore for this demo. 
%                 X = reshape(X, 1, []); % Make a vector of the matrix X. 
%                 Y = reshape(Y, 1, []);
%                 [in, on] = inpolygon(X, Y,SaveMapBoundaries(:,1),SaveMapBoundaries(:,2));
%                 
%                 mapNotObstacles = setxor(SaveMapBoundaries, SaveMapObstacles,'rows'); 
%                 subplot(1,2,1)
%                 hold on
%                 plot(SaveMapObstacles(:,1),SaveMapObstacles(:,2),'*');
%                 axis([-10, 10, -10, 10]);
%                 axis equal;
%                 drawnow;
%                 hold off
%                 subplot(1,2,2)
%                 hold on
%                 %plot(SaveMapBoundaries(:,1),SaveMapBoundaries(:,2),'*r');
%                 %plot(mapNotObstacles(:,1),mapNotObstacles(:,2),'*r');
%                 plot(X(in), Y(in), '*');
%                 hold off
%                 %axis([-10, 10, -10, 10]);
%                 %axis equal;
%                 drawnow;
                    % Select the points in the mesh [X, Y] that are visible, as returned by the Hokuyo (it returns the area that
                    % is visible, but the visualisation draws a series of points that are within this visible area). 
%                      [in, on] = inpolygon(X, Y,...
%                                     [h.hokuyo1Pos(1), pts(1, :), h.hokuyo2Pos(1)],...
%                                     [h.hokuyo1Pos(2), pts(2, :), h.hokuyo2Pos(2)]);


             end
                angl = -pi/2;

            %% Apply the state machine. 
            if strcmp(fsm, 'rotate')
                %% First, rotate the robot to go to one table.             
                % The rotation velocity depends on the difference between the current angle and the target. 
                rotateRightVel = angdiff(angl, youbotEuler(3));
            
                % When the rotation is done (with a sufficiently high precision), move on to the next state. 
                if (abs(angdiff(angl, youbotEuler(3))) < .1 / 180 * pi) && ...
                        (abs(angdiff(prevOrientation, youbotEuler(3))) < .01 / 180 * pi)
                    rotateRightVel = 0;
                    fsm = 'drive'; 
                end
                val = val + 1;
%                 if val == 100
%                     fsm = 'finished';
%                 end
                prevOrientation = youbotEuler(3);
             elseif strcmp(fsm, 'drive')
                 
                %% Then, make it move straight ahead until it reaches the table (x = 3.167 m). 
                % The further the robot, the faster it drives. (Only check for the first dimension.)
                % For the project, you should not use a predefined value, but rather compute it from your map. 
                forwBackVel = - (youbotPos(1) + 3.167);

                % If the robot is sufficiently close and its speed is sufficiently low, stop it and move its arm to 
                % a specific location before moving on to the next state.
                if (youbotPos(1) + 3.167 < .001) && (abs(youbotPos(1) - prevPosition) < .001)
                    forwBackVel = 0;

                    % Change the orientation of the camera to focus on the table (preparation for next state). 
                    vrep.simxSetObjectOrientation(id, h.rgbdCasing, h.ref, [0, 0, pi/4], vrep.simx_opmode_oneshot);

                    % Move the arm to the preset pose pickupJoints (only useful for this demo; you should compute it based
                    % on the object to grasp). 
                    for i = 1:5
                        res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), pickupJoints(i),...
                                                              vrep.simx_opmode_oneshot);
                        vrchk(vrep, res, true);
                    end

                    fsm = 'finished';
             
                end
                prevPosition = youbotPos(1);

            elseif strcmp(fsm, 'finished')
                
%                 [X, Y] = meshgrid(-10:.02:10, -10:.02:10); % Values selected for the area the robot will explore for this demo. 
%                 X = reshape(X, 1, []); % Make a vector of the matrix X. 
%                 Y = reshape(Y, 1, []);
%                 [in, on] = inpolygon(X, Y,SaveMapBoundaries(:,1),SaveMapBoundaries(:,2));
%                 hold on
%                 %plot(SaveMapBoundaries(:,1),SaveMapBoundaries(:,2),'*r');
%                 %plot(mapNotObstacles(:,1),mapNotObstacles(:,2),'*r');
%                 plot(X(in), Y(in), '*');
%                 hold off
%                 drawnow;
                
                
                
                
                % end of the navigation
                pause(1);
                fsm = 'start';
                step = 'finished';
                fprintf('Switching to step: %s\n', step);
            else
                fsm = 'finished';
                error('Unknown state %s.', fsm);
            end
        
        
            
            
            

            
            
        elseif strcmp(step, 'manipulation')
            % manipulation code
            
            step = 'vision';
            fprintf('Switching to step: %s\n', step);
        elseif strcmp(step, 'vision')
            % vision code
            
            step = 'manipulationANDvision';
            fprintf('Switching to step: %s\n', step);
        elseif strcmp(step, 'manipulationANDvision')
            % manipulation and vision code
            
            step = 'calibration';
            fprintf('Switching to step: %s\n', step);
        elseif strcmp(step, 'calibration')
            % manipulation code
            
            step = 'finished';
            fprintf('Switching to step: %s\n', step);
        elseif strcmp(step, 'finished')
            % exit the function. 
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
    
end % main function