function [ local_x_k_k, local_p_k_k, local_featuresInfo ] = extractLocalEKF( handles, z_all )
%EXTRACTLOCALEKF Summary of this function goes here
%  Detailed explanation goes here

numOfFeature = size(handles.featuresInfo,2);

x_k_k = handles.x_k_k(1:13+numOfFeature*6,1);
p_k_k = handles.p_k_k(1:13+numOfFeature*6,1:13+numOfFeature*6);
featuresInfo = handles.featuresInfo;

selected = [];

index = 1;
while size(selected,2) < 30 && index <= numOfFeature
    if ( index <= size(z_all,1) && z_all(index,1) ~= -1 )
        selected(end+1) = index;
    end
    index = index + 1;
end

for i=1:numOfFeature
    if  isempty( find( selected == (numOfFeature - i + 1) ) )
        index = 13+((numOfFeature - i + 1)-1)*6+1;
        x_k_k(index:index+5) = [];
        p_k_k(index:index+5,:) = [];
        p_k_k(:,index:index+5) = [];
        featuresInfo((numOfFeature - i + 1)) = [];
    end
end

% for i=size(selected,1):1
%     index = 13+(selected(i)-1)*6+1;
%     x_k_k(index:index+5) = [];
%     p_k_k(index:index+5,:) = [];
%     p_k_k(:,index:index+5) = [];
%     featuresInfo(selected(i)) = [];
% end

for i=1:size(featuresInfo,2)
    featuresInfo(i).positionInStateVector = i;
end

Xv = x_k_k(1:13);
R = q2r(Xv(4:7));
R = inv(R);
r_wc = Xv(1:3);

local_x_k_k = [0 0 0 1 0 0 0]';
local_x_k_k(8:10) = R*x_k_k(8:10);
local_x_k_k(11:13) = R*x_k_k(11:13);

F = zeros(size(x_k_k,1));

F(1:7,1:7) = eps*eye(7);

dR_dr = dR_by_dr(Xv(4:7));
dR_dx = dR_by_dx(Xv(4:7));
dR_dy = dR_by_dy(Xv(4:7));
dR_dz = dR_by_dz(Xv(4:7));

F(8:10,8:10) = R;
F(8:10,4) = dR_dr*x_k_k(8:10);
F(8:10,5) = dR_dx*x_k_k(8:10);
F(8:10,6) = dR_dy*x_k_k(8:10);
F(8:10,7) = dR_dz*x_k_k(8:10);

F(11:13,11:13) = R;
F(11:13,4) = dR_dr*x_k_k(11:13);
F(11:13,5) = dR_dx*x_k_k(11:13);
F(11:13,6) = dR_dy*x_k_k(11:13);
F(11:13,7) = dR_dz*x_k_k(11:13);

numOfFeature = (size(x_k_k,1)-13)/6;
for i=1:numOfFeature
    index = 13+(i-1)*6+1;
    yi = x_k_k(index:index+2);
    theta = x_k_k(index+3);
    phi = x_k_k(index+4);
    lambda = x_k_k(index+5);
    
    local_x_k_k(index:index+2) = R*( yi - r_wc );
    m1 = [cos(phi)*sin(theta)   -sin(phi)  cos(phi)*cos(theta)]';
    m = R*m1;
    local_x_k_k(index+3) = atan2(m(1),m(3));
    local_x_k_k(index+4) = atan2(-m(2),sqrt(m(1)*m(1)+m(3)*m(3)));
    local_x_k_k(index+5) = lambda;
    
    F(index+5,index+5) = 1;
    
    F(index:index+2,index:index+2) = R;
    F(index:index+2,1:3) = -R;
    F(index:index+2,4) = dR_dr*(yi-r_wc);
    F(index:index+2,5) = dR_dx*(yi-r_wc);
    F(index:index+2,6) = dR_dy*(yi-r_wc);
    F(index:index+2,7) = dR_dz*(yi-r_wc);
  
    dm1_dtheta = [ cos(phi)*cos(theta); 0; -cos(phi)*sin(theta) ];
    dm1_dphi = [ -sin(phi)*sin(theta); -cos(phi); -sin(phi)*cos(theta) ];
    local_dtheta_dm = [ m(3) 0 -m(1) ]/(m(1)*m(1)+m(3)*m(3));
    local_dphi_dm = [ m(1)*m(2)/sqrt(m(1)*m(1)+m(3)*m(3)) -sqrt(m(1)*m(1)+m(3)*m(3)) m(2)*m(3)/sqrt(m(1)*m(1)+m(3)*m(3)) ]/(m(1)*m(1)+m(2)*m(2)+m(3)*m(3));
    F(index+3,index+3) = local_dtheta_dm * R * dm1_dtheta;
    F(index+3,index+4) = local_dtheta_dm * R * dm1_dphi;
    F(index+4,index+3) = local_dphi_dm * R * dm1_dtheta;
    F(index+4,index+4) = local_dphi_dm * R * dm1_dphi;
    F(index+3,4) = local_dtheta_dm * dR_dr * m1;
    F(index+3,5) = local_dtheta_dm * dR_dx * m1;
    F(index+3,6) = local_dtheta_dm * dR_dy * m1;
    F(index+3,7) = local_dtheta_dm * dR_dz * m1;
    F(index+4,4) = local_dphi_dm * dR_dr * m1;
    F(index+4,5) = local_dphi_dm * dR_dx * m1;
    F(index+4,6) = local_dphi_dm * dR_dy * m1;
    F(index+4,7) = local_dphi_dm * dR_dz * m1;
%     dm1_dtheta1 = [ cos(phi1)*cos(theta1); 0; -cos(phi1)*sin(theta1) ];
%     dm1_dphi1 = [ -sin(phi1)*sin(theta1); -cos(phi1); -sin(phi1)*cos(theta1) ];
%     dtheta3_dm3 = [ m3(3) 0 -m3(1) ]/(m3(1)*m3(1)+m3(3)*m3(3));
%     dphi3_dm3 = [ m3(1)*m3(2)/sqrt(m3(1)*m3(1)+m3(3)*m3(3)) -sqrt(m3(1)*m3(1)+m3(3)*m3(3)) m3(2)*m3(3)/sqrt(m3(1)*m3(1)+m3(3)*m3(3)) ]/(m3(1)*m3(1)+m3(2)*m3(2)+m3(3)*m3(3));

end

local_p_k_k = F * p_k_k * F';
local_p_k_k = local_p_k_k + eye(size(local_p_k_k,1))*eps;
local_featuresInfo = featuresInfo;

return

function Hii = dht_dy( camera, Xv_km1_k, yi, zi )

    Hii = dh_dhrl( camera, Xv_km1_k, yi(7:12), zi ) * dhrlt_dy( camera, Xv_km1_k, yi, zi );

return


function R = dR_by_dr( q )
    r = q(1);   x = -q(2);   y = -q(3);   z = -q(4);
    R = [ 2*r -2*z 2*y;
        2*z 2*r -2*x;
        -2*y 2*x 2*r    ];
return

function R = dR_by_dx(q)
    r = q(1);   x = -q(2);   y = -q(3);   z = -q(4);
    R = [ 2*x 2*y 2*z;
        2*y -2*x -2*r;
        2*z 2*r -2*x    ];
return

function R = dR_by_dy(q)
    r = q(1);   x = -q(2);   y = -q(3);   z = -q(4);
    R = [ -2*y 2*x 2*r;
        2*x 2*y 2*z;
        -2*r 2*z -2*y    ];
return

function R = dR_by_dz(q)
    r = q(1);   x = -q(2);   y = -q(3);   z = -q(4);
    R = [ -2*z -2*r 2*x;
        2*r -2*z 2*y;
        2*x 2*y 2*z    ];
return
