function Move(fsm, angle,startx, starty, x, y, Addx, Addy)
% This function make a direction depending on the word fsm ('forward',
% 'right', 'backward', rotateRight' and 'finished').
% The angle is only useful for rotation.
% x and y are the coordonate of the point we want to reach. It is useless
% in the case of a rotation!

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

    vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

    % Retrieve all handles, mostly the Hokuyo.
    h = youbot_init(vrep, id);
    h = youbot_hokuyo_init(vrep, h);

    % Make sure everything is settled before we start (wait for the simulation to start). 
    pause(.2);

    % The time step the simulator is using (your code should run close to it). 
    timestep = .05;
    
    %% Preset values for the demo. 
    
    % Parameters for controlling the youBot's wheels: at each iteration, those values will be set for the wheels. 
    % They are adapted at each iteration by the code. 
    forwBackVel = 0; % Move straight ahead. 
    rightVel = 0; % Go sideways. 
    rotateRightVel = 0; % Rotate. 

    % Make sure everything is settled before we start. 
    pause(2);
    
    % First state of state machine
    fprintf('First state: %s\n', fsm);
    
    %% Start the demo. 
    while true
        tic % See end of loop to see why it's useful. 
        
        if vrep.simxGetConnectionId(id) == -1
          error('Lost connection to remote API.');
        end
    
        % Get the position and the orientation of the robot. 
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true); % Check the return value from the previous V-REP call (res) and exit in case of error.
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);

        %% Apply the state machine. 
        if strcmp(fsm, 'forward')
            % Make the robot drive with a constant speed (very simple controller, likely to overshoot). 
            % The sign indicates the direction to follow.
            forwBackVel = -1;
            
            plot(gca, youbotPos(2)*10+Addy,youbotPos(1)*10+Addx,'go');
            drawnow;
            
            % Stop when the robot is close to the goal. The tolerance has been determined by experiments: if it is too
            % small, the condition will never be met (the robot position is updated every 50 ms)
            if (sqrt(round((youbotPos(2)*10 + Addy - x))^2 + round((youbotPos(1)*10 + Addx - y))^2) < .1)
                forwBackVel = 0; % Stop the robot. 
                fsm = 'finished';
            end
             
        elseif strcmp(fsm, 'backward')
            % A speed which is a function of the distance to the destination can also be used. This is useful to avoid
            % overshooting: with this controller, the speed decreases when the robot approaches the goal.  
            forwBackVel = - 1 * (round(youbotPos(1)*10 + Addx) - y);
            plot(gca, youbotPos(2)*10+Addy,youbotPos(1)*10+Addx,'go');
             drawnow;
            % Stop when the robot is close to the goal. 
            if abs(round(youbotPos(1)*10 + Addx) - y) < .01
                forwBackVel = 0; % Stop the robot. 
                fsm = 'finished';
            end
            
        elseif strcmp(fsm, 'right')
            % Move sideways, again with a proportional controller. 
            rightVel = - 1 * (round(youbotPos(2)*10 + Addy) - x);
          plot(gca, youbotPos(2)*10+Addy,youbotPos(1)*10+Addx,'go');
             drawnow;
            % Stop at the goal.
            if abs(round(youbotPos(2)*10 + Addy) - x) < .01
                rightVel = 0; % Stop the robot. 
                fsm = 'finished';
            end
            
        elseif strcmp(fsm, 'rotateRight')
            % Rotate until the robot has the right angle. 
            % Again, use a proportional controller. In case of overshoot, the angle difference will change sign, 
            % and the robot will correctly find its way back (e.g.: the angular speed is positive, the robot overshoots, 
            % the anguler speed becomes negative). 
            % youbotEuler(3) is the rotation around the vertical axis. 
            plot(gca, youbotPos(2)*10+Addy,youbotPos(1)*10+Addx,'go');
             drawnow;
            rotateRightVel = angdiff(- angle, youbotEuler(3)); % angdiff ensures the difference is between -pi and pi. 
            % Stop when the robot is at an angle close to the right angle. 
            if abs(angdiff(- angle, youbotEuler(3))) < .002
                rotateRightVel = 0;
                fsm = 'finished';
            end
            
        elseif strcmp(fsm, 'finished')
            pause(3);
            break
        else
            error('Unknown state %s.', fsm)
        end

        % Update wheel velocities. 
        h = youbot_drive(vrep, h, forwBackVel, rightVel, rotateRightVel);
        
        % What happens if you do not update the velocities? The simulator always considers the last speed you gave it,
        % until you set a new velocity. If you perform computations for several seconds in a row without updating the
        % speeds, the robot will continue to move --- even if it bumps into a wall. 
        
        % Make sure that we do not go faster than the physics simulation (it is useless to go faster). 
        elapsed = toc;
        timeleft = timestep - elapsed;
        if (timeleft > 0)
          pause(min(timeleft, .01));
        end
    end
end 