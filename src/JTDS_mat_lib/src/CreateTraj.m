function LinkTraj = CreateTraj(varargin)
%CREATETRAJ Summary of this function goes here
%   Detailed explanation goes here

if nargin < 3 || rem(length(varargin),3)
    error('Incorrect number of argument')
end

n = length(varargin)/3;
LinkTraj = [];

for i = 1:n
    LinkTraj = [LinkTraj linspace(varargin{(i-1)*3 + 1},varargin{(i-1)*3 + 2},varargin{(i-1)*3 + 3})];
end

end

