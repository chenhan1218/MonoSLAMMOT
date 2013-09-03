%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ h_all, H_predicted, S ] = measurements_prediction( x_km1_k, p_km1_k, sigma_imageNoise, cam )
% Measurement prediction

[ h_all, H_predicted ] = h( x_km1_k, cam );

% Matrix S
numPredictedFeatures = size( find( h_all(:,1) > 0 ), 1 );

if numPredictedFeatures>0
    R_predicted = eye( 2*numPredictedFeatures )*sigma_imageNoise^2;
    S = H_predicted*p_km1_k*H_predicted' + R_predicted; 
else
    S = [];
end