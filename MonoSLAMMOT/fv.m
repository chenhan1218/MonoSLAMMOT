%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function X_k_km1=fv(X_k_k,delta_t)

   rW =X_k_k(1:3,1);
   qWR=X_k_k(4:7,1);
   vW =X_k_k(8:10,1);
   wW =X_k_k(11:13,1);
   X_k_km1=[rW+vW*delta_t;
              reshape(qprod(qWR,v2q(wW*delta_t)),4,1);
               vW;
               wW];