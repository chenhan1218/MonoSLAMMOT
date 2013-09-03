%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ XYZ ] = XYZTPL2XYZ( XYZTPL )

% XYZ
rw = XYZTPL(1:3,:);
theta = XYZTPL(4,:);
phi = XYZTPL(5,:);
lambda = XYZTPL(6,:);
mi = [cos(phi).*sin(theta);-sin(phi);cos(phi).*cos(theta)];
XYZ = rw+mi./repmat(lambda,[3 1]);