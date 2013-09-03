function [x_k_k, p_k_k, featuresInfo, detection ] = detection( fid, x_k_k, p_k_k, featuresInfo, frame_idx, detection )
%DETECTION Summary of this function goes here
%  Detailed explanation goes here

% for i=1:numel(featuresInfo)
% 	if featuresInfo(i).estType == 'u' %&& frame_idx - featuresInfo(i).firstFrame >= 60
% 		if featuresInfo(i).type == 's'
% 			x_k_k(13+(i-1)*9+[7:9])=0;
% 			p_k_k(13+(i-1)*9+[7:9],:)=0;
% 			p_k_k(:,13+(i-1)*9+[7:9])=0;
% 		end
% 		featuresInfo(i).estType = featuresInfo(i).type;
% 	end
% end

for i=1:numel(featuresInfo)
	if featuresInfo(i).estType == 'u'
		index = 13+9*(i-1);
		yi = x_k_k(index+1:index+9);
		chiSquare = chi2cdf( (0-yi(7:9))' * inv(p_k_k(index+7:index+9,index+7:index+9)) * (0-yi(7:9)), 3);
		l=chol(p_k_k(index+7:index+9,index+7:index+9));
		a=l'*l;
		velocityPdf = mvnpdf( zeros(3,1), yi(7:9), a );
		
		if velocityPdf>=4.0636
			featuresInfo(i).estType='s';
			
			if featuresInfo(i).type=='m'
				detection.movingAsStatic = detection.movingAsStatic + 1 ;
			else
				detection.staticAsStatic = detection.staticAsStatic + 1 ;
			end
			x_k_k(13+(i-1)*9+[7:9])=0;
			p_k_k(13+(i-1)*9+[7:9],:)=0;
			p_k_k(:,13+(i-1)*9+[7:9])=0;
		elseif chiSquare>=0.9986
			featuresInfo(i).estType='m';
			
			if featuresInfo(i).type=='s'
				detection.staticAsMoving = detection.staticAsMoving + 1 ;
			else
				detection.movingAsMoving = detection.movingAsMoving + 1 ;
			end
		end
	end
end
