%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plotUncertainEllip2D(C,nu,chi2,color)

hold on;

th=0:2*pi/100:2*pi;

x=[cos(th') sin(th')]'*sqrt(chi2);

if(min(eig(C))<0)
    C=eye(2);
    color=[0 0 0];
    fprintf('NPSD matrix, a black false ellipse has been plot\n');
end
K=chol(C)';

nPoints=size(th,2);
y=K*x+[ones(1,nPoints)*nu(1);ones(1,nPoints)*nu(2)];

set(plot(y(1,:),y(2,:)),'Color',color, 'LineWidth', 1.5 );