%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plotUncertainEllip3D( C, nu, chi2, color, alpha )

[ X, Y, Z ] = sphere;

nPoints = size(X,2)*size(X,1);

X = reshape( X, 1, nPoints )*sqrt(chi2);
Y = reshape( Y, 1, nPoints )*sqrt(chi2);
Z = reshape( Z, 1, nPoints )*sqrt(chi2);

K = chol( C )';

XYZprima = K*[ X; Y; Z ] + [ ones(1,nPoints)*nu(1); ones(1,nPoints)*nu(2); ones(1,nPoints)*nu(3)];

X = reshape( XYZprima(1,:), sqrt(nPoints), sqrt(nPoints) );
Y = reshape( XYZprima(2,:), sqrt(nPoints), sqrt(nPoints) );
Z = reshape( XYZprima(3,:), sqrt(nPoints), sqrt(nPoints) );

if alpha == 1
    mesh( X, Y, Z, 'FaceColor', color, 'EdgeColor', color );
else
    mesh( X, Y, Z, 'FaceColor', color , 'FaceAlpha', 0.2, 'EdgeColor', color, 'EdgeAlpha', 0.4);
end
hold on;

plot3( nu(1), nu(2), nu(3), 'Color', color, 'Marker', '+' );