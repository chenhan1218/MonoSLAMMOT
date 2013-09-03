function [x_k_k, p_k_k, featuresInfo, h_all, z_all, detection ] = removeTrackObject(x_k_k, p_k_k, featuresInfo, h_all, z_all, detection )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=1:numel(featuresInfo)
	nonStatic(i,1)=(featuresInfo(i).estType~='s');
end

% establish the reserve index for tfeaturesInfo and th_all
%reserveIndex=find(h_all(:,1)==-1 & nonStatic)';
reserveIndex=find(h_all(:,1)==-1)';
reserveIndex(reserveIndex<8)=[];

for i=1:numel(reserveIndex)
	if featuresInfo( reserveIndex(i) ).estType == 'u' && featuresInfo( reserveIndex(i) ).numMeasurement >= 20
		if featuresInfo( reserveIndex(i) ).type == 'm'
			detection.movingAsNone = detection.movingAsNone + 1 ;
		else
			detection.staticAsNone = detection.staticAsNone + 1 ;
		end
	end
end

if numel(reserveIndex)>0
	% establish the removed index for tx_k_k and tp_k_k
	reserveVectorIndex=13+repmat((reserveIndex-1)*9,[9 1])+repmat([1:9]',[1 numel(reserveIndex)]);
	% sort the list (Important!!)
	reserveVectorIndex=sort(reserveVectorIndex(:),2);
	
	x_k_k(reserveVectorIndex)=[];
	p_k_k(reserveVectorIndex,:)=[];
	p_k_k(:,reserveVectorIndex)=[];
	
	featuresInfo(reserveIndex)=[];
	h_all(reserveIndex,:)=[];
	z_all(reserveIndex,:)=[];
end
