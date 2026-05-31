function [rotation, translation] = traj_align(gndtruth, traj_est)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Rename the variables for better readability
P  = gndtruth';
Ph = traj_est';

% Dimension
N = size(P, 2);

% mean
muP  = mean(P,  2);
muPh = mean(Ph, 2);

P_bar  = P  - muP;
Ph_bar = Ph - muPh;

SIGMA = P_bar*Ph_bar'/N;

[U, D, V] = svd(SIGMA);

W = zeros(3, 3);
if det(U)*det(V) < 0
    W = diag([1, 1, -1]);
else
    W = eye(3);
end

rotation    = U*W*V';
translation = muP - rotation*muPh;

end

