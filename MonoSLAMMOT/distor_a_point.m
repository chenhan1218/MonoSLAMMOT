%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function uvd = distor_a_point( uvu, camera )

  Cx = camera.Cx;
  Cy = camera.Cy;
  k1 = camera.k1;
  k2 = camera.k2;
  dx = camera.dx;
  dy = camera.dy;
  
  
  xu=(uvu(1)-Cx)*dx;
  yu=(uvu(2)-Cy)*dy;
  
  ru=sqrt(xu*xu+yu*yu);
  rd=ru/(1+k1*ru^2+k2*ru^4);
  for k=1:100
      f=rd+k1*rd^3+k2*rd^5-ru;
      f_p=1+3*k1*rd^2+5*k2*rd^4;
      rd=rd -f/f_p;
  end
    
  D=1+k1*rd^2+k2*rd^4;
  xd=xu/D;
  yd=yu/D;
  
  uvd=[xd/dx+Cx; yd/dy+Cy];
