function fig = PlayTraj(robot, dt, robot_fig)
% Visualizes a robot's trajectory, displaying a series of robot joint positions and their corresponding
% times. If clicked before completion, will restart the playback.

% Arguments:
% robotplant - a RobotPlant object specifying the kinematics of the robot being simulated
% Q - a dimq x n matrix of robot joint positions
% T - either a single value corresponding to the timestep, OR
%     a 1 x n matrix of times corresponding to each joint position
% old_fig - optional, a figure in which to plot the trajectory (if not
%     specified another is generated)
% color - optional, the color of the trace being plotted (r, g, b, y,
%     etc.)
% 
% Outputs:
% fig - the figure in which the simulation was displated
% h - the axes on which the trace was plotted.
    
    if nargin < 4 % if axes unspecified
        fig = ShowRobot(robot);
    else
        fig = figure(robot_fig);
    end
    
    fields = fieldnames(robot);
    
    for i = 1:length(fields)
        traj{i} = [];
        for j = 2:size(robot,2)
            traj{i} =  [traj{i} robot(j).(fields{i})];
        end
    end
    
    nT = size(traj{1}, 2);
    
    %disable unwanted figure modes 
    zoom off
    rotate3d off
    pan off
    brush off
    datacursormode off

    set(fig,'WindowButtonDownFcn',@(h,e)restart_animation(h,e));
    set(fig,'WindowButtonUpFcn',[]);
    
    
    for i = 1:nT
        for j = 1:length(fields)
            robot(1).(fields{j}).robot.plot(transpose(traj{j}(:,i)));
            x = robot(1).(fields{j}).forward_kinematics(traj{j}(:,i));
            plot3(x(1), x(2), x(3), sprintf('.%s', 'r'));
            robot(1).(fields{j}).robot.delay = dt;
        end
    end
end

