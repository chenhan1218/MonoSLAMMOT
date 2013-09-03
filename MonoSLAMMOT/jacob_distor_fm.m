%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function J_distor = jacob_distor_fm( camera, uvd )
%
% Jacobian of the distortion of the image coordinates
%  presented in
%  Real-Time 3D SLAM with Wide-Angle Vision, 
%      Andrew J. Davison, Yolanda Gonzalez Cid and Nobuyuki Kita, IAV 2004.
% input
%    camera   -  camera calibration parameters
%    uv       -  distorted image points in pixels
% output
%    J_distor -  distorted coordinate points
%
% J.M.M Montiel
% August 2005

J_distor = inv( jacob_undistor_fm( camera, uvd ) );