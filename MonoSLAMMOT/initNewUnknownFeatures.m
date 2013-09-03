function [ handles ] = initNewUnknownFeatures( handles )
%INITNEWUNKNOWNFEATURES Summary of this function goes here
%  Detailed explanation goes here

frame_idx = handles.frame_idx;

static_container = handles.static_container;
moving_container = handles.moving_container;

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
		x_k_k = [ x_k_k; newFeature; 0 ;0 ;0 ];
		p_k_k = addAFeatureCov_newPar( p_k_k, pos, Xv, lambdaInit, sigma_imageNoise, std_lambda, cam );
		p_k_k(end+1:end+3,end+1:end+3)=eye(3)*1 ;
		
		featuresInfo(end+1).type = 's';
		featuresInfo(end).estType = 'u';
		featuresInfo(end).positionInGroundTruth = i;
		featuresInfo(end).pointIndex = static_container(i).pointIndex;
		featuresInfo(end).firstFrame = frame_idx;
		featuresInfo(end).numMeasurement = 0;
		tracked_points.static(end+1) = i;
	end
end


for i=1:size(moving_container,2)
	if ~isempty( find( tracked_points.moving == i ))
		continue;
	end
	
	index = find( moving_container(i).frame == frame_idx );
	if ~isempty(index) & isequal( moving_container(i).projection(:,index), [-1;-1] ) == 0
		pos = moving_container(i).projection(:,index);
		
		newFeature = hinv( pos, Xv, cam, lambdaInit );
		x_k_k = [ x_k_k; newFeature; 0 ;0 ;0 ];
		p_k_k = addAFeatureCov_newPar( p_k_k, pos, Xv, lambdaInit, sigma_imageNoise, std_lambda, cam );
		p_k_k(end+1:end+3,end+1:end+3)=eye(3)*1 ;
		
		featuresInfo(end+1).type = 'm';
		featuresInfo(end).estType = 'u';
		featuresInfo(end).positionInGroundTruth = i;
		featuresInfo(end).pointIndex = moving_container(i).pointIndex;
		featuresInfo(end).firstFrame = frame_idx;
		featuresInfo(end).numMeasurement = 0;
		tracked_points.moving(end+1) = i;
	end
end

handles.x_k_k = x_k_k;
handles.p_k_k = p_k_k;
handles.featuresInfo = featuresInfo;
handles.tracked_points = tracked_points;
