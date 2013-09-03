function [ handles ] = initNewStaticFeatures( handles )
%INITNEWSTATICFEATURE Summary of this function goes here
%  Detailed explanation goes here

frame_idx = handles.frame_idx;

static_container = handles.static_container;

x_k_k = handles.x_k_k;
p_k_k = handles.p_k_k;
Xv = x_k_k(1:13);
cam = handles.cam;
lambdaInit = handles.lambdaInit;
std_lambda = handles.std_lambda;
sigma_imageNoise = handles.sigma_imageNoise;
featuresInfo = handles.featuresInfo;
tracked_points = handles.tracked_points;

for i=1:size(static_container,2)
	if ~isempty( find( tracked_points.static == i ))
		continue;
	end
	
	index = find( static_container(i).frame == frame_idx );
	if ~isempty(index) & isequal( static_container(i).projection(:,index), [-1;-1] ) == 0
		pos = static_container(i).projection(:,index);
		
		newFeature = hinv( pos, Xv, cam, lambdaInit );
		if ismember(static_container(i).pointIndex,[9 10 11 12])==1
			newFeature(6,1) = 1/sqrt(sum((static_container(i).position-x_k_k(1:3)).^2));
		end
		x_k_k = [ x_k_k; newFeature; 0 ;0 ;0 ];
		p_k_k = addAFeatureCov_newPar( p_k_k, pos, Xv, lambdaInit, sigma_imageNoise, std_lambda, cam );
		if ismember(static_container(i).pointIndex,[9 10 11 12])==1
			p_k_k(size(p_k_k,1),size(p_k_k,2)) = 0.000001; %0.0017361;
		end
		p_k_k(end+1:end+3,end+1:end+3)=zeros(3,3);
		
		featuresInfo(end+1).type = 's';
		featuresInfo(end).estType = 's';
		featuresInfo(end).positionInGroundTruth = i;
		featuresInfo(end).pointIndex = static_container(i).pointIndex;
		featuresInfo(end).firstFrame = frame_idx;
		featuresInfo(end).numMeasurement = 0;
		tracked_points.static(end+1) = i;
	end
end

handles.x_k_k = x_k_k;
handles.p_k_k = p_k_k;
handles.featuresInfo = featuresInfo;
handles.tracked_points = tracked_points;