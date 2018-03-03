function angles = GetJointAngles(target, origin, links, final_angle1)
%GETJOINTANGLES Summary of this function goes here
%   Detailed explanation goes here
angles = zeros(length(links),1);
target = abs(target - origin);

a = links(1);
b = links(2);
c = links(3);
d = norm(target);

if a+b+c < d
    error('error')
end

angle_link1_target = acos(([1 0]*target')/norm(target));
alpha = angle_link1_target - final_angle1;
limit_angle = acos((a^2 + d^2 - b^2 - c^2 -2*b*c)/(2*a*d));

if alpha > limit_angle || alpha < -limit_angle
    error('error')
end

r = sqrt(a^2 + d^2 - 2*a*d*cos(alpha));
gamma = acos((c^2 + b^2 - r^2)/(2*b*c));
beta = acos((b^2 + r^2 - c^2)/(2*b*r));
omega = acos((a^2 + r^2 - d^2)/(2*a*r));

angles(1) = final_angle1;
angles(2) = pi - omega + beta;
angles(3) = gamma -pi;
end

