function outputTrackingPar(x_k_k, p_k_k, featuresInfo, z_all, fid, frame_idx, cam_container, static_container, moving_container );
%OUTPUTTRACKINGPAR Summary of this function goes here
%  Detailed explanation goes here

for i= 1:numel(featuresInfo)
	if featuresInfo(i).estType ~= 's'
		index = 13+9*(i-1);
		yi = x_k_k(index+1:index+9);
		xyztpl = yi(1:6);
		
		r=xyztpl(1:3);
		theta=xyztpl(4);
		phi=xyztpl(5);
		rho=xyztpl(6);
		m1 = [cos(phi)*sin(theta)   -sin(phi)  cos(phi)*cos(theta)]';
		dm1_dtheta = [ cos(phi)*cos(theta); 0; -cos(phi)*sin(theta) ];
		dm1_dphi = [ -sin(phi)*sin(theta); -cos(phi); -sin(phi)*cos(theta) ];
		
		% Global position
		featurePos = r+m1/rho;
		F=[eye(3) dm1_dtheta/rho dm1_dphi/rho -m1/(rho^2) zeros(3,3)];
		featurePos_p_k_k=F*p_k_k(index+1:index+9,index+1:index+9)*F';
		
		chiSquare = chi2cdf( (0-yi(7:9))' * inv(p_k_k(index+7:index+9,index+7:index+9)) * (0-yi(7:9)), 3);
		l=chol(p_k_k(index+7:index+9,index+7:index+9));
		a=l'*l;
		velocityPdf = mvnpdf( zeros(3,1), yi(7:9), a );
		
		pointIdx = featuresInfo(i).positionInGroundTruth;
		if featuresInfo(i).type == 'm'
			frame_index = find( moving_container(pointIdx).frame <= frame_idx );
			% set the value of GroundTruthFeature
			if isempty(frame_index)==1 | moving_container(pointIdx).frame(frame_index(end))~=frame_idx
				GroundTruthFeature=[NaN;NaN;NaN];
			else
				GroundTruthFeature = moving_container(pointIdx).position(:,frame_index(end));
			end

			if numel(frame_index) < 2
				GroundTruthFeatureVelocity=[NaN;NaN;NaN];
			else
				GroundTruthFeatureVelocity = (moving_container(pointIdx).position(:,frame_index(end))-moving_container(pointIdx).position(:,frame_index(end-1)))/...
					((moving_container(pointIdx).frame(frame_index(end))-moving_container(pointIdx).frame(frame_index(end-1)))/30);
			end
		else
			frame_index = find( static_container(pointIdx).frame <= frame_idx );
			% set the value of GroundTruthFeature
			GroundTruthFeature = static_container(pointIdx).position;
			GroundTruthFeatureVelocity = zeros(3,1);
		end
		
		% output the result to the file
		fprintf(fid,'%d\t',featuresInfo(i).pointIndex);
		fprintf(fid,'%d\t',featuresInfo(i).type=='s');
		fprintf(fid,'%d\t',frame_idx);
		fprintf(fid,'%d\t',(i <= size(z_all,1) && isequal(z_all(i,:),[-1,-1])==0 ));
		fprintf(fid,'%.4f\t',featurePos);
		fprintf(fid,'%.4f\t',diag(featurePos_p_k_k));
		fprintf(fid,'%.4f\t',GroundTruthFeature);
		fprintf(fid,'%.4f\t',x_k_k(index+7:index+9));
		fprintf(fid,'%.4f\t',full(diag(p_k_k(index+7:index+9,index+7:index+9))));
		fprintf(fid,'%.4f\t',GroundTruthFeatureVelocity);
		fprintf(fid,'%.4f\t',chiSquare);
		fprintf(fid,'%.4f\t',velocityPdf);
		fprintf(fid,'\n');
	end
end

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
