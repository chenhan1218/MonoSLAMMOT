%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function zi = hi( yinit, Xv_km1_k, cam )

% Compute a single measurement

% Points 3D in camera coordinates
R_wc = q2r( Xv_km1_k(4:7) );
R_cw = inv( R_wc );
r_wc = Xv_km1_k(1:3);

yi = yinit(1:3);
theta = yinit(4);
phi = yinit(5);
lambda = yinit(6);

mi = [cos(phi)*sin(theta)   -sin(phi)  cos(phi)*cos(theta)]';

hrl = R_cw*( (yi - r_wc) + mi/lambda );

% Is in front of the camera?
if (hrl(3)<eps)
    zi = [];
    return;
end

% Image coordinates
uv_u = hu( hrl, cam );
% Add distortion
uv_d = distort_fm( uv_u , cam );

% excluded band
excluded_band = 2; % pxls

% Is visible in the image?
if ( uv_d(1)>0+excluded_band ) && ( uv_d(1)<cam.nCols-excluded_band ) &&...
        ( uv_d(2)>0+excluded_band ) && ( uv_d(2)<cam.nRows-excluded_band )
    zi = uv_d;
    return;
else
    zi = [];
    return;
end