%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function R=q2r(q)

%q=q/norm(q);

x=q(2);
y=q(3);
z=q(4);
r=q(1);

R=[r*r+x*x-y*y-z*z  2*(x*y -r*z)     2*(z*x+r*y)
   2*(x*y+r*z)      r*r-x*x+y*y-z*z  2*(y*z-r*x)
   2*(z*x-r*y)      2*(y*z+r*x)      r*r-x*x-y*y+z*z];