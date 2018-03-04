function parcoursPieceSuite(arg)
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

        if toPlot == 1
        subplot(1,2,1)
        title('Subplot 1: Obstacles')
        hold on
        plot(pts(1, contacts), pts(2, contacts), '*');
        
        plot(youbotPos(1),youbotPos(2),'go');
        axis([-10, 10, -10, 10]);
        axis equal;
        drawnow;
        hold off
            
        subplot(1,2,2)
        title('Subplot 2: Sensor vision')
        hold on
        plot([h.hokuyo1Pos(1), pts(1, :), h.hokuyo2Pos(1)], [h.hokuyo1Pos(2), pts(2, :), h.hokuyo2Pos(2)])
        fill([h.hokuyo1Pos(1), pts(1, :), h.hokuyo2Pos(1)], [h.hokuyo1Pos(2), pts(2, :), h.hokuyo2Pos(2)],'b')
        plot(youbotPos(1),youbotPos(2),'go');
        axis([-10, 10, -10, 10]);
        axis equal;
        hold off
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
            subplot(1,2,2)
            hold on
            plot(boundaries(:,1),boundaries(:,2),'r*');
            hold off

            figure;
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
            
            map = sparse(nonegi,nonegj,1);
            map(map>1)=1;
            
            goal = [round(boundaries(1,1)*10) + addI,round(boundaries(1,2)*10)+addJ]
            start = [round(youbotPos(1)*10-11) + addI,round(youbotPos(2)*10)+addJ]
            %cm: !!!!!! j'ai remis un -1 pour que ça fonctionne avec 1, -11 avec 10, c'est ok
            %dans ce cas, peut poser des problèmes dans la suite!!!!!!
            ds = Dstar(map);    % create navigation object
            ds.plan(goal)       % create plan for specified goal
            ds.path(start)      % animate path from this start location
   
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