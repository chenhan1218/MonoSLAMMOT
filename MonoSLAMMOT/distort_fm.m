%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function uvd = distort_fm( uv, camera )
%
% Distort image coordinates.
%  The function deals with two models:
%  1.- Real-Time 3D SLAM with Wide-Angle Vision, 
%      Andrew J. Davison, Yolanda Gonzalez Cid and Nobuyuki Kita, IAV 2004.
%  2.- Photomodeler full distortion model.
% input
%    camera -  camera calibration parameters
%    uvd    -  distorted image points in pixels
% output
%    uv     -  undistorted coordinate points
%
% J.M.M Montiel
% September 2005

nPoints = size( uv, 2 );
uvd = zeros( 2, nPoints );
for k = 1:nPoints;
    uvd( :, k ) = distor_a_point( uv( :, k ), camera );
end
