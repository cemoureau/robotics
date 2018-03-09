function parcoursPieceSuite2(arg)
    if nargin == 0
        toPlot = 1;
    else
        toPlot = 0;
    end

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
    %   Remote API function call returned with error code: 64. Explanation: simxStart was not yet called.
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
    % The tip corresponds to the point between the two tongs of the gripper (for more details, see later or in the 
    % file focused/youbot_arm.m). 
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
    
    
    
    
    
    
    
    
    %% Preset values for the demo. 
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

    
    
    % Initialise the state machine. 
    fsm = 'forward';
    
    

    %% Start the demo. 
        passed = 0;
        passed2 = 0;
   % while true
        tic % See end of loop to see why it's useful. 
        
        if vrep.simxGetConnectionId(id) == -1
            error('Lost connection to remote API.');
        end
    
        % Get the position and the orientation of the robot. 
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);

        %% Plot something if required. 
            
        trf = transl(youbotPos) * trotx(youbotEuler(1)) * troty(youbotEuler(2)) * trotz(youbotEuler(3));
        worldHokuyo1 = homtrans(trf, [h.hokuyo1Pos(1); h.hokuyo1Pos(2); h.hokuyo1Pos(3)]);
        worldHokuyo2 = homtrans(trf, [h.hokuyo2Pos(1); h.hokuyo2Pos(2); h.hokuyo2Pos(3)]);
        [pts, contacts] = youbot_hokuyo(vrep, h, vrep.simx_opmode_buffer,trf);

        toSaveprev = [transpose(pts(1, contacts)) transpose(pts(2, contacts))];
        if passed == 0
           toSave = toSaveprev;
           passed = 1;
        else
            toSave = union(toSave,toSaveprev,'rows');
        end

        toSaveprev2 = [transpose([h.hokuyo1Pos(1), pts(1, :), h.hokuyo2Pos(1)]) transpose([h.hokuyo1Pos(2), pts(2, :), h.hokuyo2Pos(2)])];
        if passed2 == 0
           toSave2 = toSaveprev2;
           passed2 = 1;
        else
            toSave2 = union(toSave2,toSaveprev2,'rows');
        end
        
        angl = -pi/2;
   
        %% Apply the state machine. 
        if strcmp(fsm, 'forward')
            
            %The goal is to reach to point of toSave2 which is the most far
            %away from the robot and which is not a wall
            boundaries = setxor(toSave2,toSave,'rows'); 
            %subplot(1,2,2)
%             hold on
%             plot(boundaries(:,1),boundaries(:,2),'ro');
%             hold off
            
            m1 = max(round(toSave(:,1).*10));
            m2 = max(round(toSave2(:,1).*10));
            m3 = max(round(toSave(:,2).*10));
            m4= max(round(toSave2(:,2).*10));
           
            i = round(toSave(:,1).*10);
            j = round(toSave(:,2).*10);
            %v = ones(length(i),1);
            
            minI = min(i);
            minJ = min(j);
            minBi = min(round(toSave2(:,1).*10));
            minBj = min(round(toSave2(:,2).*10));
            
            addI = - min(minI,minBi) +1;
            addJ = - min(minJ,minBj) +1;
            %cm: ajouter condition est ce que c'est négatif pour la suite
            
            nonegi = double(i + addI);
            nonegj = double(j + addJ);
            
            xmax = max(m1,m2)+ addI;
            ymax = max(m3,m4)+ addJ;
            
            map = sparse(nonegi,nonegj,1,double(xmax),double(ymax));
            map(map>1)=1;
            
            figure;
            goal = [round(boundaries(188,2)*10)+addJ,round(boundaries(188,1)*10) + addI];
            start = [round(youbotPos(2)*10)+addJ, round(youbotPos(1)*10) + addI];
            %cm: !!!!!! il faut inverser x et y !!!!!!
            ds = Dstar(map);    % create navigation object
            ds.plan(goal)       % create plan for specified goal
            ds.path(start)      % animate path from this start location
            
            
            
            
            
            
            figure;
            imagesc(map); 
            hold on
            plot(round(youbotPos(2)*10)+addJ, round(youbotPos(1)*10) + addI, 'g*');
            plot(round(boundaries(188,2)*10)+addJ,round(boundaries(188,1)*10) + addI,'r*');
            
            
            
            
            % New
            
            % Take indices of obstacles
            [xi,xj,~] = find(map);

            % Extract point from the current plot. x and y are cells!
            h = findobj(gca,'Type','line');
            x = get(h,'Xdata');
            y = get(h,'Ydata');
            
            % If the path is too close to an obstacle we take another path
            % and recompute next the new path to the last goal.
            % We assume the length of the robot about 6 and the width : 3
            % r = racine carré de (?x2 + ?y2)
%             for i=2:length(x)
%                 for j=1:length(xj)
%                     if (sqrt((x{length(x)+1-i} - xj(j))^2 + (y{length(y)+1-i} - xi(j))^2) < 3)
%                         if (x{length(x)+1-i} - xj(j) < 0)
%                             x{length(x)+1-i} = x{length(x)+1-i} - (3 - abs(x{length(x)+1-i} - xj(j)));
%                         end
%                         if (x{length(x)+1-i} - xj(j) > 0)
%                             x{length(x)+1-i} = x{length(x)+1-i} + (3 - abs(x{length(x)+1-i} - xj(j)));
%                         end
%                         if (x{length(x)+1-i} - xj(j) < 0)
%                             x{length(x)+1-i} = x{length(x)+1-i} - (3 - abs(x{length(x)+1-i} - xj(j)));
%                         end
%                         if (x{length(x)+1-i} - xj(j) > 0)
%                             x{length(x)+1-i} = x{length(x)+1-i} + (3 - abs(x{length(x)+1-i} - xj(j)));
%                         end
%                         
%                         if (y{length(y)+1-i} - xi(j) < 3)
%                             y{length(y)+1-i} = y{length(y)+1-i} + 3;                       
%                         elseif (y{length(y)+1-i} - xi(j) < 3)
%                             y{length(y)+1-i} = y{length(y)+1-i} - 3;
%                         end
%                     end
%                 end                
%             end
            
            for i=2:length(x)
                for j=1:length(xj)
                    if (sqrt((x{length(x)+1-i} - xj(j))^2 + (y{length(y)+1-i} - xi(j))^2) < 3)
                        theta = atan(y{length(y)+1-i} - xi(j)/x{length(x)+1-i} - xj(j));
                        deltax = (3 - sqrt((x{length(x)+1-i} - xj(j))^2 + (y{length(y)+1-i} - xi(j))^2) * cos(theta));
                        deltay = (3 - sqrt((x{length(x)+1-i} - xj(j))^2 + (y{length(y)+1-i} - xi(j))^2) * sin(theta));
                        
                        if (xj(j) - x{length(x)+1-i} < 0)
                            x{length(x)+1-i} = x{length(x)+1-i} + deltax;
                        elseif (xj(j) - x{length(x)+1-i} < 0)
                            x{length(x)+1-i} = x{length(x)+1-i} - deltax;
                        end
                        if (xi(j) - y{length(y)+1-i} < 0)
                            y{length(y)+1-i} = y{length(y)+1-i} - deltay;
                    
                        elseif (xi(j) - y{length(y)+1-i} < 0)
                            y{length(y)+1-i} = y{length(y)+1-i} + deltay;
                        end
                       
                    end
                end                
            end

            % Initialization
            next_point = zeros(2);
            pt_startX = start(1);
            pt_startY = start(2);
                
            % Loop to go to the goal. need also to update the map!!!
            for i=2:length(x)
                
                % Next point in the path. 
                next_point(1) = x{length(x)+1-i};
                next_point(2) = y{length(y)+1-i};

                % Point at the same abscisse and ordonate bigger => go forward
                if((next_point(1) - pt_startX) == 0 && next_point(2) > pt_startY)
                    fsm = 'forward';
                    angle = 0;
                    Move(fsm, angle, pt_startX, pt_startY, next_point(1), next_point(2), addI, addJ);
%                     while true
%                             tic % See end of loop to see why it's useful. 
% 
%                             if vrep.simxGetConnectionId(id) == -1
%                               error('Lost connection to remote API.');
%                             end
% 
%                             % Get the position and the orientation of the robot. 
%                             [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
%                             vrchk(vrep, res, true); % Check the return value from the previous V-REP call (res) and exit in case of error.
%                             [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
%                             vrchk(vrep, res, true);
% 
%                             %% Apply the state machine. 
%                             if strcmp(fsm, 'forward')
%                                 % Make the robot drive with a constant speed (very simple controller, likely to overshoot). 
%                                 % The sign indicates the direction to follow.
%                                 forwBackVel = -1;
% 
%                                 plot(gca, youbotPos(2)*10+addJ,youbotPos(1)*10+addI,'go');
%                                 drawnow;
% 
%                                 % Stop when the robot is close to the goal. The tolerance has been determined by experiments: if it is too
%                                 % small, the condition will never be met (the robot position is updated every 50 ms)
%                                 if (sqrt(round((youbotPos(2)*10 + addJ - x))^2 + round((youbotPos(1)*10 + addI - y))^2) < .1)
%                                     forwBackVel = 0; % Stop the robot. 
%                                     fsm = 'finished';
%                                 end
% 
%                             elseif strcmp(fsm, 'backward')
%                                 % A speed which is a function of the distance to the destination can also be used. This is useful to avoid
%                                 % overshooting: with this controller, the speed decreases when the robot approaches the goal.  
%                                 forwBackVel = - 1 * (round(youbotPos(1)*10 + addI) - y);
%                                 plot(gca, youbotPos(2)*10+addJ,youbotPos(1)*10+addI,'go');
%                                  drawnow;
%                                 % Stop when the robot is close to the goal. 
%                                 if abs(round(youbotPos(1)*10 + addI) - y) < .01
%                                     forwBackVel = 0; % Stop the robot. 
%                                     fsm = 'finished';
%                                 end
% 
%                             elseif strcmp(fsm, 'right')
%                                 % Move sideways, again with a proportional controller. 
%                                 rightVel = - 1 * (round(youbotPos(2)*10 + addJ) - x);
%                               plot(gca, youbotPos(2)*10+addJ,youbotPos(1)*10+addI,'go');
%                                  drawnow;
%                                 % Stop at the goal.
%                                 if abs(round(youbotPos(2)*10 + addJ) - x) < .01
%                                     rightVel = 0; % Stop the robot. 
%                                     fsm = 'finished';
%                                 end
% 
%                             elseif strcmp(fsm, 'rotateRight')
%                                 % Rotate until the robot has the right angle. 
%                                 % Again, use a proportional controller. In case of overshoot, the angle difference will change sign, 
%                                 % and the robot will correctly find its way back (e.g.: the angular speed is positive, the robot overshoots, 
%                                 % the anguler speed becomes negative). 
%                                 % youbotEuler(3) is the rotation around the vertical axis. 
%                                 plot(gca, youbotPos(2)*10+addJ,youbotPos(1)*10+addI,'go');
%                                  drawnow;
%                                 rotateRightVel = angdiff(- angle, youbotEuler(3)); % angdiff ensures the difference is between -pi and pi. 
%                                 % Stop when the robot is at an angle close to the right angle. 
%                                 if abs(angdiff(- angle, youbotEuler(3))) < .002
%                                     rotateRightVel = 0;
%                                     fsm = 'finished';
%                                 end
% 
%                             elseif strcmp(fsm, 'finished')
%                                 pause(3);
%                                 break
%                             else
%                                 error('Unknown state %s.', fsm)
%                             end
% 
%                             % Update wheel velocities. 
%                             h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel);
% 
%                             % What happens if you do not update the velocities? The simulator always considers the last speed you gave it,
%                             % until you set a new velocity. If you perform computations for several seconds in a row without updating the
%                             % speeds, the robot will continue to move --- even if it bumps into a wall. 
% 
%                             % Make sure that we do not go faster than the physics simulation (it is useless to go faster). 
%                             elapsed = toc;
%                             timeleft = timestep - elapsed;
%                             if (timeleft > 0)
%                               pause(min(timeleft, .01));
%                             end
%                     end
%                 %% end Move
%                 
                % Point at the same abscisse and ordonate smaller => go
                % backward
                elseif((next_point(1) - pt_startX) == 0 && next_point(2) < pt_startY)
                    fsm = 'backward';
                    angle = 0;
                    Move(fsm, angle, pt_startX, pt_startY, next_point(1), next_point(2), addI, addJ);
                     %% Move(fsm, angle, pt_startX, pt_startY, next_point(1), next_point(2), addI, addJ, id, h);
%                     while true
%                             tic % See end of loop to see why it's useful. 
% 
%                             if vrep.simxGetConnectionId(id) == -1
%                               error('Lost connection to remote API.');
%                             end
% 
%                             % Get the position and the orientation of the robot. 
%                             [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
%                             vrchk(vrep, res, true); % Check the return value from the previous V-REP call (res) and exit in case of error.
%                             [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
%                             vrchk(vrep, res, true);
% 
%                             %% Apply the state machine. 
%                             if strcmp(fsm, 'forward')
%                                 % Make the robot drive with a constant speed (very simple controller, likely to overshoot). 
%                                 % The sign indicates the direction to follow.
%                                 forwBackVel = -1;
% 
%                                 plot(gca, youbotPos(2)*10+addJ,youbotPos(1)*10+addI,'go');
%                                 drawnow;
% 
%                                 % Stop when the robot is close to the goal. The tolerance has been determined by experiments: if it is too
%                                 % small, the condition will never be met (the robot position is updated every 50 ms)
%                                 if (sqrt(round((youbotPos(2)*10 + addJ - x))^2 + round((youbotPos(1)*10 + addI - y))^2) < .1)
%                                     forwBackVel = 0; % Stop the robot. 
%                                     fsm = 'finished';
%                                 end
% 
%                             elseif strcmp(fsm, 'backward')
%                                 % A speed which is a function of the distance to the destination can also be used. This is useful to avoid
%                                 % overshooting: with this controller, the speed decreases when the robot approaches the goal.  
%                                 forwBackVel = - 1 * (round(youbotPos(1)*10 + addI) - y);
%                                 plot(gca, youbotPos(2)*10+addJ,youbotPos(1)*10+addI,'go');
%                                  drawnow;
%                                 % Stop when the robot is close to the goal. 
%                                 if abs(round(youbotPos(1)*10 + addI) - y) < .01
%                                     forwBackVel = 0; % Stop the robot. 
%                                     fsm = 'finished';
%                                 end
% 
%                             elseif strcmp(fsm, 'right')
%                                 % Move sideways, again with a proportional controller. 
%                                 rightVel = - 1 * (round(youbotPos(2)*10 + addJ) - x);
%                               plot(gca, youbotPos(2)*10+addJ,youbotPos(1)*10+addI,'go');
%                                  drawnow;
%                                 % Stop at the goal.
%                                 if abs(round(youbotPos(2)*10 + addJ) - x) < .01
%                                     rightVel = 0; % Stop the robot. 
%                                     fsm = 'finished';
%                                 end
% 
%                             elseif strcmp(fsm, 'rotateRight')
%                                 % Rotate until the robot has the right angle. 
%                                 % Again, use a proportional controller. In case of overshoot, the angle difference will change sign, 
%                                 % and the robot will correctly find its way back (e.g.: the angular speed is positive, the robot overshoots, 
%                                 % the anguler speed becomes negative). 
%                                 % youbotEuler(3) is the rotation around the vertical axis. 
%                                 plot(gca, youbotPos(2)*10+addJ,youbotPos(1)*10+addI,'go');
%                                  drawnow;
%                                 rotateRightVel = angdiff(- angle, youbotEuler(3)); % angdiff ensures the difference is between -pi and pi. 
%                                 % Stop when the robot is at an angle close to the right angle. 
%                                 if abs(angdiff(- angle, youbotEuler(3))) < .002
%                                     rotateRightVel = 0;
%                                     fsm = 'finished';
%                                 end
% 
%                             elseif strcmp(fsm, 'finished')
%                                 pause(3);
%                                 break
%                             else
%                                 error('Unknown state %s.', fsm)
%                             end
% 
%                             % Update wheel velocities. 
%                             h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel);
% 
%                             % What happens if you do not update the velocities? The simulator always considers the last speed you gave it,
%                             % until you set a new velocity. If you perform computations for several seconds in a row without updating the
%                             % speeds, the robot will continue to move --- even if it bumps into a wall. 
% 
%                             % Make sure that we do not go faster than the physics simulation (it is useless to go faster). 
%                             elapsed = toc;
%                             timeleft = timestep - elapsed;
%                             if (timeleft > 0)
%                               pause(min(timeleft, .01));
%                             end
%                     end
%                 %% end Move
%                 
                % Point at the same ordonnate => go right
                elseif((next_point(2) - pt_startY) == 0)
                   fsm = 'right';
                   angle = 0;
                   Move(fsm, angle, pt_startX, pt_startY, next_point(1), next_point(2), addI, addJ, id, h);
%                     %% Move(fsm, angle, pt_startX, pt_startY, next_point(1), next_point(2), addI, addJ, id, h);
%                     while true
%                             tic % See end of loop to see why it's useful. 
% 
%                             if vrep.simxGetConnectionId(id) == -1
%                               error('Lost connection to remote API.');
%                             end
% 
%                             % Get the position and the orientation of the robot. 
%                             [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
%                             vrchk(vrep, res, true); % Check the return value from the previous V-REP call (res) and exit in case of error.
%                             [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
%                             vrchk(vrep, res, true);
% 
%                             %% Apply the state machine. 
%                             if strcmp(fsm, 'forward')
%                                 % Make the robot drive with a constant speed (very simple controller, likely to overshoot). 
%                                 % The sign indicates the direction to follow.
%                                 forwBackVel = -1;
% 
%                                 plot(gca, youbotPos(2)*10+addJ,youbotPos(1)*10+addI,'go');
%                                 drawnow;
% 
%                                 % Stop when the robot is close to the goal. The tolerance has been determined by experiments: if it is too
%                                 % small, the condition will never be met (the robot position is updated every 50 ms)
%                                 if (sqrt(round((youbotPos(2)*10 + addJ - x))^2 + round((youbotPos(1)*10 + addI - y))^2) < .1)
%                                     forwBackVel = 0; % Stop the robot. 
%                                     fsm = 'finished';
%                                 end
% 
%                             elseif strcmp(fsm, 'backward')
%                                 % A speed which is a function of the distance to the destination can also be used. This is useful to avoid
%                                 % overshooting: with this controller, the speed decreases when the robot approaches the goal.  
%                                 forwBackVel = - 1 * (round(youbotPos(1)*10 + addI) - y);
%                                 plot(gca, youbotPos(2)*10+addJ,youbotPos(1)*10+addI,'go');
%                                  drawnow;
%                                 % Stop when the robot is close to the goal. 
%                                 if abs(round(youbotPos(1)*10 + addI) - y) < .01
%                                     forwBackVel = 0; % Stop the robot. 
%                                     fsm = 'finished';
%                                 end
% 
%                             elseif strcmp(fsm, 'right')
%                                 % Move sideways, again with a proportional controller. 
%                                 rightVel = - 1 * (round(youbotPos(2)*10 + addJ) - x);
%                               plot(gca, youbotPos(2)*10+addJ,youbotPos(1)*10+addI,'go');
%                                  drawnow;
%                                 % Stop at the goal.
%                                 if abs(round(youbotPos(2)*10 + addJ) - x) < .01
%                                     rightVel = 0; % Stop the robot. 
%                                     fsm = 'finished';
%                                 end
% 
%                             elseif strcmp(fsm, 'rotateRight')
%                                 % Rotate until the robot has the right angle. 
%                                 % Again, use a proportional controller. In case of overshoot, the angle difference will change sign, 
%                                 % and the robot will correctly find its way back (e.g.: the angular speed is positive, the robot overshoots, 
%                                 % the anguler speed becomes negative). 
%                                 % youbotEuler(3) is the rotation around the vertical axis. 
%                                 plot(gca, youbotPos(2)*10+addJ,youbotPos(1)*10+addI,'go');
%                                  drawnow;
%                                 rotateRightVel = angdiff(- angle, youbotEuler(3)); % angdiff ensures the difference is between -pi and pi. 
%                                 % Stop when the robot is at an angle close to the right angle. 
%                                 if abs(angdiff(- angle, youbotEuler(3))) < .002
%                                     rotateRightVel = 0;
%                                     fsm = 'finished';
%                                 end
% 
%                             elseif strcmp(fsm, 'finished')
%                                 pause(3);
%                                 break
%                             else
%                                 error('Unknown state %s.', fsm)
%                             end
% 
%                             % Update wheel velocities. 
%                             h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel);
% 
%                             % What happens if you do not update the velocities? The simulator always considers the last speed you gave it,
%                             % until you set a new velocity. If you perform computations for several seconds in a row without updating the
%                             % speeds, the robot will continue to move --- even if it bumps into a wall. 
% 
%                             % Make sure that we do not go faster than the physics simulation (it is useless to go faster). 
%                             elapsed = toc;
%                             timeleft = timestep - elapsed;
%                             if (timeleft > 0)
%                               pause(min(timeleft, .01));
%                             end
%                     end
%                 %% end Move 
                   
                % Make first a rotation then go forward to the next point
                else
                    fsm = 'rotateRight';
                    
                    angle = -atan((next_point(2) - pt_startY)/(next_point(1) - pt_startX));
                    Move(fsm, angle, pt_startX, pt_startY, next_point(1), next_point(2), addI, addJ);
%                      %% Move(fsm, angle, pt_startX, pt_startY, next_point(1), next_point(2), addI, addJ, id, h);
%                     while true
%                             tic % See end of loop to see why it's useful. 
% 
%                             if vrep.simxGetConnectionId(id) == -1
%                               error('Lost connection to remote API.');
%                             end
% 
%                             % Get the position and the orientation of the robot. 
%                             [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
%                             vrchk(vrep, res, true); % Check the return value from the previous V-REP call (res) and exit in case of error.
%                             [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
%                             vrchk(vrep, res, true);
% 
%                             %% Apply the state machine. 
%                             if strcmp(fsm, 'forward')
%                                 % Make the robot drive with a constant speed (very simple controller, likely to overshoot). 
%                                 % The sign indicates the direction to follow.
%                                 forwBackVel = -1;
% 
%                                 plot(gca, youbotPos(2)*10+addJ,youbotPos(1)*10+addI,'go');
%                                 drawnow;
% 
%                                 % Stop when the robot is close to the goal. The tolerance has been determined by experiments: if it is too
%                                 % small, the condition will never be met (the robot position is updated every 50 ms)
%                                 if (sqrt(round((youbotPos(2)*10 + addJ - x))^2 + round((youbotPos(1)*10 + addI - y))^2) < .1)
%                                     forwBackVel = 0; % Stop the robot. 
%                                     fsm = 'finished';
%                                 end
% 
%                             elseif strcmp(fsm, 'backward')
%                                 % A speed which is a function of the distance to the destination can also be used. This is useful to avoid
%                                 % overshooting: with this controller, the speed decreases when the robot approaches the goal.  
%                                 forwBackVel = - 1 * (round(youbotPos(1)*10 + addI) - y);
%                                 plot(gca, youbotPos(2)*10+addJ,youbotPos(1)*10+addI,'go');
%                                  drawnow;
%                                 % Stop when the robot is close to the goal. 
%                                 if abs(round(youbotPos(1)*10 + addI) - y) < .01
%                                     forwBackVel = 0; % Stop the robot. 
%                                     fsm = 'finished';
%                                 end
% 
%                             elseif strcmp(fsm, 'right')
%                                 % Move sideways, again with a proportional controller. 
%                                 rightVel = - 1 * (round(youbotPos(2)*10 + addJ) - x);
%                               plot(gca, youbotPos(2)*10+addJ,youbotPos(1)*10+addI,'go');
%                                  drawnow;
%                                 % Stop at the goal.
%                                 if abs(round(youbotPos(2)*10 + addJ) - x) < .01
%                                     rightVel = 0; % Stop the robot. 
%                                     fsm = 'finished';
%                                 end
% 
%                             elseif strcmp(fsm, 'rotateRight')
%                                 % Rotate until the robot has the right angle. 
%                                 % Again, use a proportional controller. In case of overshoot, the angle difference will change sign, 
%                                 % and the robot will correctly find its way back (e.g.: the angular speed is positive, the robot overshoots, 
%                                 % the anguler speed becomes negative). 
%                                 % youbotEuler(3) is the rotation around the vertical axis. 
%                                 plot(gca, youbotPos(2)*10+addJ,youbotPos(1)*10+addI,'go');
%                                  drawnow;
%                                 rotateRightVel = angdiff(- angle, youbotEuler(3)); % angdiff ensures the difference is between -pi and pi. 
%                                 % Stop when the robot is at an angle close to the right angle. 
%                                 if abs(angdiff(- angle, youbotEuler(3))) < .002
%                                     rotateRightVel = 0;
%                                     fsm = 'finished';
%                                 end
% 
%                             elseif strcmp(fsm, 'finished')
%                                 pause(3);
%                                 break
%                             else
%                                 error('Unknown state %s.', fsm)
%                             end
% 
%                             % Update wheel velocities. 
%                             h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel);
% 
%                             % What happens if you do not update the velocities? The simulator always considers the last speed you gave it,
%                             % until you set a new velocity. If you perform computations for several seconds in a row without updating the
%                             % speeds, the robot will continue to move --- even if it bumps into a wall. 
% 
%                             % Make sure that we do not go faster than the physics simulation (it is useless to go faster). 
%                             elapsed = toc;
%                             timeleft = timestep - elapsed;
%                             if (timeleft > 0)
%                               pause(min(timeleft, .01));
%                             end
%                     end
%                 %% end Move
                    
                    fsm = 'forward';
                    angle = 0;
                    Move(fsm, angle, pt_startX, pt_startY, next_point(1), next_point(2), addI, addJ);
%                      %% Move(fsm, angle, pt_startX, pt_startY, next_point(1), next_point(2), addI, addJ, id, h);
%                     while true
%                             tic % See end of loop to see why it's useful. 
% 
%                             if vrep.simxGetConnectionId(id) == -1
%                               error('Lost connection to remote API.');
%                             end
% 
%                             % Get the position and the orientation of the robot. 
%                             [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
%                             vrchk(vrep, res, true); % Check the return value from the previous V-REP call (res) and exit in case of error.
%                             [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
%                             vrchk(vrep, res, true);
% 
%                             %% Apply the state machine. 
%                             if strcmp(fsm, 'forward')
%                                 % Make the robot drive with a constant speed (very simple controller, likely to overshoot). 
%                                 % The sign indicates the direction to follow.
%                                 forwBackVel = -1;
% 
%                                 plot(gca, youbotPos(2)*10+addJ,youbotPos(1)*10+addI,'go');
%                                 drawnow;
% 
%                                 % Stop when the robot is close to the goal. The tolerance has been determined by experiments: if it is too
%                                 % small, the condition will never be met (the robot position is updated every 50 ms)
%                                 if (sqrt(round((youbotPos(2)*10 + addJ - x))^2 + round((youbotPos(1)*10 + addI - y))^2) < .1)
%                                     forwBackVel = 0; % Stop the robot. 
%                                     fsm = 'finished';
%                                 end
% 
%                             elseif strcmp(fsm, 'backward')
%                                 % A speed which is a function of the distance to the destination can also be used. This is useful to avoid
%                                 % overshooting: with this controller, the speed decreases when the robot approaches the goal.  
%                                 forwBackVel = - 1 * (round(youbotPos(1)*10 + addI) - y);
%                                 plot(gca, youbotPos(2)*10+addJ,youbotPos(1)*10+addI,'go');
%                                  drawnow;
%                                 % Stop when the robot is close to the goal. 
%                                 if abs(round(youbotPos(1)*10 + addI) - y) < .01
%                                     forwBackVel = 0; % Stop the robot. 
%                                     fsm = 'finished';
%                                 end
% 
%                             elseif strcmp(fsm, 'right')
%                                 % Move sideways, again with a proportional controller. 
%                                 rightVel = - 1 * (round(youbotPos(2)*10 + addJ) - x);
%                               plot(gca, youbotPos(2)*10+addJ,youbotPos(1)*10+addI,'go');
%                                  drawnow;
%                                 % Stop at the goal.
%                                 if abs(round(youbotPos(2)*10 + addJ) - x) < .01
%                                     rightVel = 0; % Stop the robot. 
%                                     fsm = 'finished';
%                                 end
% 
%                             elseif strcmp(fsm, 'rotateRight')
%                                 % Rotate until the robot has the right angle. 
%                                 % Again, use a proportional controller. In case of overshoot, the angle difference will change sign, 
%                                 % and the robot will correctly find its way back (e.g.: the angular speed is positive, the robot overshoots, 
%                                 % the anguler speed becomes negative). 
%                                 % youbotEuler(3) is the rotation around the vertical axis. 
%                                 plot(gca, youbotPos(2)*10+addJ,youbotPos(1)*10+addI,'go');
%                                  drawnow;
%                                 rotateRightVel = angdiff(- angle, youbotEuler(3)); % angdiff ensures the difference is between -pi and pi. 
%                                 % Stop when the robot is at an angle close to the right angle. 
%                                 if abs(angdiff(- angle, youbotEuler(3))) < .002
%                                     rotateRightVel = 0;
%                                     fsm = 'finished';
%                                 end
% 
%                             elseif strcmp(fsm, 'finished')
%                                 pause(3);
%                                 break
%                             else
%                                 error('Unknown state %s.', fsm)
%                             end
% 
%                             % Update wheel velocities. 
%                             h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel);
% 
%                             % What happens if you do not update the velocities? The simulator always considers the last speed you gave it,
%                             % until you set a new velocity. If you perform computations for several seconds in a row without updating the
%                             % speeds, the robot will continue to move --- even if it bumps into a wall. 
% 
%                             % Make sure that we do not go faster than the physics simulation (it is useless to go faster). 
%                             elapsed = toc;
%                             timeleft = timestep - elapsed;
%                             if (timeleft > 0)
%                               pause(min(timeleft, .01));
%                             end
%                     end
%                 %% end Move
                 end
                
                % New start point. take the value from the cell !!!!
                pt_startX = x{length(x)+1-i};
                pt_startY = y{length(y)+1-i};
            end
            
            
            
            
            
            
            
            
            
            % End new
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
        end
    %end
    
   figure
   subplot(1,2,1);
   plot(toSave(:,1),toSave(:,2),'*');
   title('Obstacles saved')
   axis([-10, 10, -10, 10]);
   axis equal;
   
   subplot(1,2,2);
   plot(toSave2(:,1),toSave2(:,2),'r');
   %fill(toSave2(:,1),toSave2(:,2),'b');
   title('Sensor area covered')
   axis([-10, 10, -10, 10]);
   axis equal;
  
   
end