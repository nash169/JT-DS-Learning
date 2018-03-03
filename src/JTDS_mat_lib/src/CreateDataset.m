function robot = CreateDataset(robot, n_examples, dt, tol_cutting)
% This function preprocesses raw joint demonstration data and molds it into
% a suitable format.
% The function computes the first time derivative of demonstrations and
% trims the data. The function can be
% called using: 
%
%          [Data, index] = preprocess_demos(demos,time,tol_cutting)
%
% Inputs -----------------------------------------------------------------
%
%
%   o demos:   A variable containing all demonstrations (only
%              joint trajectories). The variable 'demos' should follow the
%              following format:
%              - demos{n}: qdim x T^n matrix representing the qdim dimensional
%                          joint trajectories. T^n is the number of datapoints in
%                          each demonstration (1 < n < N)
%
%   o time:    This variable can be provided in two ways. If time steps
%              between all demonstration points are the same, 'time' could be
%              given as a positive scalar (i.e. time = dt). If not, 'time'
%              should follow the following format:
%              - time{n}: 1 x T^n vector representing the time array of length
%                         T^n corresponding to each demo  (1 < n < N)
%
%   o tol_cutting:  A small positive scalar that is used to trim data. It
%                   removes the redundant datapoint from the begining and
%                   the end of each demonstration that their first time
%                   derivative is less than 'tol_cutting'. This is not
%                   strictly necessary for JT-DS; however from practical point of
%                   view, it is very useful. There are always lots of noisy
%                   data at the begining (before the user starts the
%                   demonstration) and the end (after the user finished the
%                   demonstration) of each demosntration that are not
%                   useful.
%
% Outputs ----------------------------------------------------------------
%
%   o xT:      qdim x N array representing the each of the demonstration's
%              final points (target points).
%
%   o Data:    A (2*qdim + xdim) x N_Total matrix containing all demonstration data points.
%              The rows are layed out as follows:
%              1:qdim = joint angles
%              qdim + 1: 2*qdim = joint velocities
%              2*qdim + 1:2*qdim + xdim = target position
%
%              Each column of Data stands
%              for a datapoint. All demonstrations are put next to each other 
%              along the second dimension. For example, if we have 3 demos
%              D1, D2, and D3, then the matrix Data is:
%                               Data = [[D1] [D2] [D3]]
%
%   o index:   A vector of N+1 components defining the initial index of each
%              demonstration. For example, index = [1 T1 T2 T3] indicates
%              that columns 1:T1-1 belongs to the first demonstration,
%              T1:T2-1 -> 2nd demonstration, and T2:T3-1 -> 3rd
%              demonstration.


fields = fieldnames(robot);

[n,m] = size(robot(2).(fields{1}));

z = size(robot,2);

% Generate random perturbations of these two base trajectories - increase
% the number of data for learning algorithm
for i = 1:length(fields)
    Data = [];
    for k = 1:n_examples
%         base = robot(j).(fields{i});
%         robot(j).(fields{i}) = zeros(n, m, n_examples);
        for j = 2:z
            q = robot(j).(fields{i}) + pi/30*(rand(n, m)-.5);
            q_d = diff(q,1,2)/dt;
            ind = find(sqrt(sum(q_d.*q_d,1))>tol_cutting);
            q = q(:,min(ind):max(ind)+1);
            q_d = [q_d(:,min(ind):max(ind)) zeros(n,1)];
            xT = repmat(robot(1).(fields{i}).forward_kinematics(q(:,end)), 1, m);
            Data = [Data [q; q_d; xT]];
        end
    end
    robot(z+1).(fields{i}) = Data;
end

end

