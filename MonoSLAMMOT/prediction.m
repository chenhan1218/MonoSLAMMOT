%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ X_km1_k, P_km1_k ] = prediction( X_k, P_k, featuresInfo, delta_t, SD_A_component_filter, SD_alpha_component_filter, sigma_ObjectAlphaNoise )
% Prediction step

% Camera motion prediction
Xv_km1_k = fv( X_k(1:13,:), delta_t );
% moving object prediction
object_km1_k = ft( X_k(14:end),delta_t) ;

% The prediction for the features is to stay at the same place
X_km1_k = [ Xv_km1_k; object_km1_k ];

% Prediction for P
F = dfv_by_dxv( X_k(1:13,:),zeros(6,1),delta_t );
Ft = dft_by_dxt( X_k(14:end), delta_t );

Pn=diag([(SD_A_component_filter*delta_t)^2 (SD_A_component_filter*delta_t)^2 (SD_A_component_filter*delta_t)^2 ...
        (SD_alpha_component_filter*delta_t)^2 (SD_alpha_component_filter*delta_t)^2 (SD_alpha_component_filter*delta_t)^2]);

[Q,G] = func_Q( X_k(1:13,:), zeros(6,1), Pn, delta_t );

Qt=sparse(numel(X_k)-13,numel(X_k)-13);
for i=1:numel(featuresInfo)
	if featuresInfo(i).estType ~= 's'
		index = 9*(i-1);
		Qt(index+7:index+9,index+7:index+9) = eye(3)*(sigma_ObjectAlphaNoise*delta_t)^2; % set the acceleration be sigma_ObjectAlphaNoise m/s^2
	end
end

F_tot     = speye( size(X_k,1) );
F_tot(1:13,1:13)=F ;
F_tot(14:end,14:end) = Ft ;
    
P_km1_k = F_tot*P_k*F_tot' +...
	[Q						sparse(13,numel(X_k)-13);
	sparse(numel(X_k)-13,13)	Qt	] ;

