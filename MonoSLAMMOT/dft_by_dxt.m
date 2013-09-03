function [ dft_by_dxtRES ] = dft_by_dxt( X_k, delta_t )
%DFT_BY_DXT Summary of this function goes here
%  Detailed explanation goes here

dim = size(X_k,1);

element = zeros(9);
element(1:3,1:3) = eye(3); 
element(7:9,7:9) = eye(3);

dft_by_dxtRES = sparse(dim,dim);
num = dim/9;

for i=1:num
    index = 9*(i-1);
    dft_by_dxtRES(index+1:index+9,index+1:index+9) = element;
    
    r1 = X_k(index+1:index+3);
    theta1 = X_k(index+4,1);
    phi1 = X_k(index+5,1);
    rho1 = X_k(index+6,1);
    m1 = [cos(phi1)*sin(theta1)   -sin(phi1)  cos(phi1)*cos(theta1)]';
    dm1_dtheta1 = [ cos(phi1)*cos(theta1); 0; -cos(phi1)*sin(theta1) ];
    dm1_dphi1 = [ -sin(phi1)*sin(theta1); -cos(phi1); -sin(phi1)*cos(theta1) ];
    
	v = X_k(index+7:index+9);
	
	vector = m1/rho1 + v * delta_t ;
	m2 = vector / norm(vector ) ;
	rho2 = 1/norm(vector) ;
	
    dtheta2_dvector = [ vector(3) 0 -vector(1) ]/(vector(1)*vector(1)+vector(3)*vector(3));
    dphi2_dvector = [ vector(1)*vector(2)/sqrt(vector(1)*vector(1)+vector(3)*vector(3)) -sqrt(vector(1)*vector(1)+vector(3)*vector(3)) vector(2)*vector(3)/sqrt(vector(1)*vector(1)+vector(3)*vector(3)) ]/(vector(1)*vector(1)+vector(2)*vector(2)+vector(3)*vector(3));
	drho2_dvector = -vector' / (norm(vector)^3) ;
	
	dvector_dm1 = eye(3)/rho1 ;
	dvector_dv = eye(3)*delta_t ;
	
	
    dft_by_dxtRES(index+4,index+4) = dtheta2_dvector * dvector_dm1 * dm1_dtheta1; %dtheta2_dtheta1
    dft_by_dxtRES(index+4,index+5) = dtheta2_dvector * dvector_dm1 * dm1_dphi1; %dtheta2_dphi1
    dft_by_dxtRES(index+4,index+6) = dtheta2_dvector * m1 / (-rho1^2);
    dft_by_dxtRES(index+4,index+7:index+9) = dtheta2_dvector * dvector_dv;
	
    dft_by_dxtRES(index+5,index+4) = dphi2_dvector * dvector_dm1 * dm1_dtheta1; %dphi2_dtheta1
    dft_by_dxtRES(index+5,index+5) = dphi2_dvector * dvector_dm1* dm1_dphi1; %dphi2_dphi1
    dft_by_dxtRES(index+5,index+6) = dphi2_dvector * m1 / (-rho1^2);
    dft_by_dxtRES(index+5,index+7:index+9) = dphi2_dvector * dvector_dv;
	
    dft_by_dxtRES(index+6,index+4) = drho2_dvector * dvector_dm1 * dm1_dtheta1; %dphi2_dtheta1
    dft_by_dxtRES(index+6,index+5) = drho2_dvector * dvector_dm1* dm1_dphi1; %dphi2_dphi1
    dft_by_dxtRES(index+6,index+6) = drho2_dvector * m1 / (-rho1^2);
    dft_by_dxtRES(index+6,index+7:index+9) = drho2_dvector * dvector_dv;
end
