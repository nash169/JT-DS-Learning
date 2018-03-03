function fig = ShowRobot(robot, fig)
%SHOWROBOT Summary of this function goes here
%   Detailed explanation goes here

plot_options = {'noshadow','nojaxes', 'nowrist', 'noname','jointdiam', 2,...
    'basewidth', 1, 'linkcolor',0.7*[1,1,1], 'ortho','noshading',...
    'notiles','jointcolor',0.6*[1,1,1]};

if nargin == 1
    fig = figure (100);
else
    axes(fig);
end

fields = fieldnames(robot); 

robot_handle = robot(1).(fields{1}).robot;
robot_handle.plotopt = plot_options;
robot_handle.plot(zeros(1, robot_handle.n));

hold on
view([0 90])

if length(fields) > 1
    for i = 2:length(fields)
        robot_handle = robot(1).(fields{i}).robot;
        robot_handle.plotopt = plot_options;
        robot_handle.plot(zeros(1, robot_handle.n));
    end
end

end