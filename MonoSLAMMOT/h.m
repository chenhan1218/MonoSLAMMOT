%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ h_all, H ] = h( x_km1_k, cam )
% Measurement, derivatives and texture prediction

numFeatures = (size( x_km1_k, 1 ) - 13)/9;
XfeatToPredict = x_km1_k( 14:end );
h_all = ones( numFeatures, 2 )*(-1);
H = [];

% Extract camera motion parameters
xv_km1_k = x_km1_k( 1:13 );

for i = 1:numFeatures
    
    % Extract feature
    yi = XfeatToPredict( 1:6 );
    XfeatToPredict = XfeatToPredict( 10:end );
    % Measurement equation
    zi = hi( yi, xv_km1_k, cam );
    if (~isempty(zi)) % if is predicted in the image
        h_all(i,:) = zi';
        % Calculate derivatives
        Hi = calculate_Hi(  xv_km1_k, yi, zi, cam, i, numFeatures);
        H = [ H; Hi];
    end
   
end