%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function uvu = undistor_a_point( uvd, camera )

Cx = camera.Cx;
Cy = camera.Cy;
k1 = camera.k1;
k2 = camera.k2;
dx = camera.dx;
dy = camera.dy;

ud = uvd(1);
vd = uvd(2);
rd = sqrt( ( dx*(ud-Cx) )^2 + (dy*(vd-Cy) )^2 );

uu = Cx + ( ud - Cx )*( 1 + k1*rd^2 + k2*rd^4 );
vu = Cy + ( vd - Cy )*( 1 + k1*rd^2 + k2*rd^4 );

uvu = [ uu; vu ];