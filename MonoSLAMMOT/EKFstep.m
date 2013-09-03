function [ x_k_k, p_k_k, h_all, S, z_all ] = EKFstep( x_k_k, p_k_k, featuresInfo, ...
	frame_idx, deltat, cam, sigma_aNoise, sigma_alphaNoise, sigma_imageNoise, sigma_ObjectAlphaNoise, ...
	chi2inv_table, static_container, moving_container )
%EKFSTEP Summary of this function goes here
% Detailed explanation goes here

% Monocular EKF step

% prediction
[ x_km1_k, p_km1_k ] = prediction( x_k_k, p_k_k, featuresInfo, deltat, sigma_aNoise, sigma_alphaNoise, sigma_ObjectAlphaNoise );

% measurements prediction
[ h_all, H_predicted, S ] = measurements_prediction( x_km1_k, p_km1_k, sigma_imageNoise, cam );

% matching
[ z, h, H_matching, R_matching, z_all ] = matching( h_all, H_predicted, S, featuresInfo, sigma_imageNoise, static_container, moving_container, frame_idx );

% update
[ x_k_k, p_k_k ] = update( x_km1_k, p_km1_k, H_matching, R_matching, z, h );

