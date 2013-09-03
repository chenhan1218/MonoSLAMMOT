%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Hi = calculate_Hi( Xv_km1_k, yi, zi, camera, i, numFeatures )

Hi = dh_dxv( camera, Xv_km1_k, yi, zi );

Hi=[Hi sparse(2,numFeatures*9)];
Hi(:,(i-1)*9+13+[1:6])=dh_dy( camera, Xv_km1_k, yi, zi );

return



function Hii = dh_dy( camera, Xv_km1_k, yi, zi )

    Hii = dh_dhrl( camera, Xv_km1_k, yi, zi ) * dhrl_dy( camera, Xv_km1_k, yi, zi );

return



function a = dhrl_dy( camera, Xv_km1_k, yi, zi )

    rw = Xv_km1_k( 1:3 );
    Rrw = inv( q2r( Xv_km1_k( 4:7 ) ) );
    lambda = yi(6);
    phi = yi(5);
    theta = yi(4);
    
    dmi_dthetai = Rrw*[cos(phi)*cos(theta)  0   -cos(phi)*sin(theta)]';
    dmi_dphii = Rrw*[-sin(phi)*sin(theta)  -cos(phi)   -sin(phi)*cos(theta)]';
    
    a = [lambda*Rrw  dmi_dthetai dmi_dphii Rrw*(yi(1:3)-rw) ];
    
return



function Hi1 = dh_dxv( camera, Xv_km1_k, yi, zi )

    Hi1 = [ dh_drw( camera, Xv_km1_k, yi, zi )  dh_dqwr( camera, Xv_km1_k, yi, zi ) zeros( 2, 6 )];

return



function Hi12 = dh_dqwr( camera, Xv_km1_k, yi, zi )

    Hi12 = dh_dhrl( camera, Xv_km1_k, yi, zi ) * dhrl_dqwr( camera, Xv_km1_k, yi, zi );
    
return



function a = dhrl_dqwr( camera, Xv_km1_k, yi, zi )

    rw = Xv_km1_k( 1:3 );
    qwr = Xv_km1_k( 4:7 );
    lambda = yi(6);
    phi = yi(5);
    theta = yi(4);
    mi = [cos(phi)*sin(theta)   -sin(phi)  cos(phi)*cos(theta)]';
    
    a = dRq_times_a_by_dq( qconj(qwr), ((yi(1:3) - rw)*lambda + mi) )*dqbar_by_dq;
    
return



function Hi11 = dh_drw( camera, Xv_km1_k, yi, zi )

    Hi11 = dh_dhrl( camera, Xv_km1_k, yi, zi ) * dhrl_drw( camera, Xv_km1_k, yi, zi );

return



function a = dhrl_drw( camera, Xv_km1_k, yi, zi )

    a = -( inv( q2r( Xv_km1_k(4:7) ) ) )*yi(6);

return



function a = dh_dhrl( camera, Xv_km1_k, yi, zi )

    a = dhd_dhu( camera, Xv_km1_k, yi, zi )*dhu_dhrl( camera, Xv_km1_k, yi, zi );

return



function a = dhd_dhu( camera, Xv_km1_k, yi, zi_d )

    inv_a = jacob_undistor_fm( camera, zi_d );
    a=inv(inv_a);
    
return
  
    
function a = dhu_dhrl( camera, Xv_km1_k, yi, zi )
    
    f = camera.f;
    ku = 1/camera.dx;
    kv = 1/camera.dy;
    rw = Xv_km1_k( 1:3 );
    Rrw = inv(q2r( Xv_km1_k( 4:7 ) ));
    
    theta = yi(4);
    phi = yi(5);
    rho = yi(6);
    mi = [cos(phi)*sin(theta)   -sin(phi)  cos(phi)*cos(theta)]';
    
    hc = Rrw*( (yi(1:3) - rw)*rho + mi );
    hcx = hc(1);
    hcy = hc(2);
    hcz = hc(3);
    a = [-f*ku/(hcz)       0           hcx*f*ku/(hcz^2);
        0               -f*kv/(hcz)    hcy*f*kv/(hcz^2)];

return