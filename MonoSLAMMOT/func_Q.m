%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Q,G] = func_Q( Xv,u,Pn,delta_t )

omegaOld=Xv(11:13);
qOld=Xv(4:7);

G=zeros(13,6);

G(8:10,1:3)=eye(3);
G(11:13,4:6)=eye(3);
G(1:3,1:3)=eye(3)*delta_t;
G(4:7,4:6)=dq3_by_dq1(qOld)*dqomegadt_by_domega(omegaOld,delta_t);

Q=G*Pn*G';