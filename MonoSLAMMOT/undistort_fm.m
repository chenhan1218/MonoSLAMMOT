%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function uvu = undistort_fm( uvd, camera )
%
% Undistort image coordinates
% Javier Civera, 16/11/05

nPoints = size( uvd, 2 );
uvu = zeros( 2, nPoints );
for k = 1:nPoints;
    uvu( :, k ) = undistor_a_point( uvd( :, k ), camera );
end
