function L = CreateLinks(A, D, Alpha, Qmin, Qmax)
% CREATE LINKS 
%   Creates links for a robot used to feed the SerialLink class from
%   Peter Corke's Robotics Toolbox. The object created by the Serial Link
%   class can be used to generate jacobians and forward kinematics,
%   and to pass around robot parameters. The first 3 parameters are column
%   vectors of Denavit-Hartenberg parameters (with the "offset q" assumed 
%   to be 0). The last two are minimum and maximum joint angles with 
%   respect to the default "0" angle.

    qdim = length(A);
    if nargin < 5 % if joint limits unspecified, set them to the max possible
        Qmin = 2*pi*ones(qdim, 1);
        Qmax = -2*pi*ones(qdim, 1);
        warning('No joint limits specified, may cause unwanted behavior if using limit-enforcement methods.');
    end
    
    for i = 1:qdim
        L(i) = Link('d', D(i), 'a', A(i), 'alpha', Alpha(i), 'standard', 'revolute', 'qlim', [Qmin(i) Qmax(i)]);
    end
end
