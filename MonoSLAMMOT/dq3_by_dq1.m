%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dq3_by_dq1RES=dq3_by_dq1(q2_in)

 q2.R=q2_in(1); 
 q2.X=q2_in(2);
 q2.Y=q2_in(3);
 q2.Z=q2_in(4);
 
 dq3_by_dq1RES=[q2.R, -q2.X, -q2.Y, -q2.Z,
                q2.X,  q2.R, -q2.Z,  q2.Y,
		        q2.Y,  q2.Z,  q2.R, -q2.X,
		        q2.Z, -q2.Y,  q2.X,  q2.R];
 return
