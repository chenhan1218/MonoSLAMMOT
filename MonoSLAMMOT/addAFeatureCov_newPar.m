%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function P_RES = addAFeatureCov_newPar( P, uvd, Xv, rhoInit, std_pxl, std_rho, cam )
% Estimation of the feature from the measurement

fku = -cam.K(1,1);
fkv = -cam.K(2,2);
U0  =  cam.K(1,3);
V0  =  cam.K(2,3);

r_W = Xv(1:3);
q_wc = Xv(4:7);
R_wc = q2r(q_wc);

uvu = undistort_fm( uvd, cam );
uu = uvu(1);
vu = uvu(2);

X_c = (U0-uu)/fku;
Y_c = (V0-vu)/fkv;
Z_c = 1;

XYZ_c = [X_c; Y_c; Z_c];

XYZ_w = R_wc*XYZ_c;
X_w = XYZ_w(1);
Y_w = XYZ_w(2);
Z_w = XYZ_w(3);

% Derivatives

dtheta_dgw = [ Z_w/(X_w^2+Z_w^2) 0 -X_w/(X_w^2+Z_w^2) ];
dphi_dgw = [ (X_w*Y_w)/((X_w^2+Y_w^2+Z_w^2)*sqrt(X_w^2+Z_w^2))...
        -sqrt(X_w^2+Z_w^2)/(X_w^2+Y_w^2+Z_w^2) (Z_w*Y_w)/((X_w^2+Y_w^2+Z_w^2)*sqrt(X_w^2+Z_w^2)) ];
dgw_dqwr = dRq_times_a_by_dq( q_wc, XYZ_c );

dtheta_dqwr = dtheta_dgw*dgw_dqwr;
dphi_dqwr = dphi_dgw*dgw_dqwr;
dy_dqwr = [ zeros(3,4); dtheta_dqwr; dphi_dqwr; zeros(1,4) ];
dy_drw = [ eye(3); zeros(3,3) ];

dy_dxv = [ dy_drw dy_dqwr zeros(6,6) ];

dyprima_dgw = [ zeros(3,3); dtheta_dgw; dphi_dgw ];
dgw_dgc = R_wc;
dgc_dhu = [ -1/fku      0       0;
                0   -1/fkv      0]';
dhu_dhd = jacob_undistor_fm( cam , uvd );

dyprima_dhd = dyprima_dgw*dgw_dgc*dgc_dhu*dhu_dhd;

dy_dhd = [ dyprima_dhd zeros(5,1);
            zeros(1,2)      1 ];

% Jacobian
J = sparse( [speye(size(P,2))                                   zeros(size(P,1),3);
    dy_dxv                zeros(6,(size(P,2)-13))       dy_dhd  ] );

Ri = eye(2)*std_pxl^2;
if size(P,1)==13 
Padd = [Ri  zeros(2,1);
    zeros(1,2)  std_rho^2];  
else
Padd = [Ri  zeros(2,1);
    zeros(1,2)  std_rho^2];
end
    
P = [P  zeros(size(P,2),3);
    zeros(3,size(P,2))   Padd];
    
P_RES = J*P*J';