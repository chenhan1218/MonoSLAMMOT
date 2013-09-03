function [ X_k_km1 ] = ft( X_k, delta_t )
%FT Summary of this function goes here
%  Detailed explanation goes here

num = size(X_k,1)/9;
X_k_km1 = [];

for i=1:num
    index = 9*(i-1);
    r1 = X_k(index+1:index+3);
    theta1 = X_k(index+4,1);
    phi1 = X_k(index+5,1);
    rho1 = X_k(index+6,1);
    m1 = [cos(phi1)*sin(theta1)   -sin(phi1)  cos(phi1)*cos(theta1)]';
    
	v = X_k(index+7:index+9);
	
	vector = m1/rho1 + v * delta_t ;
	m2 = vector / norm(vector ) ;
	rho2 = 1/norm(vector) ;
    
    X_k_km1(index+1:index+9,1) = [ r1;
                                  atan2(m2(1),m2(3));
%                                   theta2;
                                  atan2(-m2(2),sqrt(m2(1)*m2(1)+m2(3)*m2(3)));
%                                   phi2;
                                  rho2;
								  v];
end

