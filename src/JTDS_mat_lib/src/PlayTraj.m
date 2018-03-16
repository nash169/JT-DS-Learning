function fig = PlayTraj(robot, type_traj, robot_fig)
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
    
    if nargin < 3 % if axes unspecified
        fig = ShowRobot(robot);
    else
        fig = figure(robot_fig);
    end
    
    fields = fieldnames(robot);
    n_components = length(fields);
    
    switch type_traj
        case 'demonstration'
            slot = 2;
        case 'learned'
            slot = 5;
        otherwise
            error('error');
    end
    
    for i = 1:n_components
        traj{i} = [];
        n_traj = length(robot(slot).(fields{i}));
        for j = 1:n_traj
            curr_traj = robot(slot).(fields{i}){j};
            curr_traj(end,:) = [diff(curr_traj(end,:)) 0];
            traj{i} = [traj{i} curr_traj];
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
            robot(1).(fields{j}).robot.plot(transpose(traj{j}(1:end-1,i)));
            x = robot(1).(fields{j}).forward_kinematics(traj{j}(1:end-1,i));
            plot3(x(1), x(2), x(3), sprintf('.%s', 'r'));
            robot(1).(fields{j}).robot.delay = traj{j}(end,i);
        end
    end
end

