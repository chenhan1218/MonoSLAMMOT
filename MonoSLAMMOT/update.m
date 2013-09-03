%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ x_k_k, p_k_k ] = update( x_km1_k, p_km1_k, H, R, z, h )

if size(z,1)>0
    
    % filter gain
    K = p_km1_k*H'*inv(H*p_km1_k*H' + R);
    
    % updated state and covariance
    x_k_k = x_km1_k + K*( z - h );
    p_k_k = sparse(eye(size(p_km1_k,1)) - K*H )*p_km1_k;
    
    % normalize the quaternion
    x_k_k( 4:7 ) = x_k_k( 4:7 ) / norm( x_k_k( 4:7 ) );
    Jnorm = speye( size( x_k_k, 1 ) );
    Jnorm( 4:7, 4:7 ) = normJac( x_k_k( 4:7 ) );
    p_k_k = Jnorm*p_k_k*Jnorm';
    
else
    
    x_k_k = x_km1_k;
    p_k_k = p_km1_k;
    
end