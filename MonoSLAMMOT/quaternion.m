function q=quaternion(v,theta)
% Iquaternion(v,theta) creates a quaternion
% input data
%     v the directional vector, not neccesarily unit vector
%    theta the rotation angle in radians
%
% output data
%     1x4 vector codifying the quaternion
%
% jmmm 12 july 2005
%


q=[cos(theta/2)  sin(theta/2)*reshape(v,1,3)/norm(v)];
    
