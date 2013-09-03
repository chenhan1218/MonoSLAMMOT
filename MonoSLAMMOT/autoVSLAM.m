function [ output_args ] = autoVSLAM( input_args )
%AUTOVSLAM Summary of this function goes here
%  Detailed explanation goes here

setParameters;

handles.h_all=[];
handles.S=[];
handles.z_all=[];

handles.tracked_points.static = [];
handles.tracked_points.moving = [];

handles.detection(1).staticAsStatic=0;
handles.detection(1).staticAsMoving=0;
handles.detection(1).staticAsNone=0;
handles.detection(1).movingAsStatic=0;
handles.detection(1).movingAsMoving=0;
handles.detection(1).movingAsNone=0;


%first step
%select features in the first step
handles = initNewStaticFeatures( handles );
handles.camHistory(:,end+1) = [handles.x_k_k(1:7)];
handles.camGrnHistory(:,end+1)= [ handles.cam_container.position(:,handles.frame_idx+1) ; handles.cam_container.orientation(:,handles.frame_idx+1) ];

set(gcf,'PaperPosition',[0.6345    6.3452   20.3046   15.2284]*2);
% looping through all frame
while 1
	handles.step = handles.step+1;
	handles.frame_idx = handles.frame_idx + handles.frameInteval ;
	
	% if the frame_idx exceed the number of images, then break the loop
	if handles.frame_idx >= handles.numberOfImages
		break ;
	end
	
	fprintf(1,'image %6d		step %6d\n', handles.frame_idx, handles.step);
	
	[ handles.x_k_k, handles.p_k_k, handles.h_all, handles.S, handles.z_all ] = EKFstep( handles.x_k_k, handles.p_k_k, handles.featuresInfo, ...
	handles.frame_idx, handles.step_deltat, handles.cam, handles.sigma_aNoise, handles.sigma_alphaNoise, handles.sigma_imageNoise, ...
	handles.chi2inv_table, handles.static_container, handles.moving_container );

	for i=1:numel(handles.featuresInfo)
		handles.featuresInfo(i).numMeasurement=handles.featuresInfo(i).numMeasurement+(~isequal(handles.z_all(i,:),[-1 -1]));
	end

	%remove the feature that can't be seen by the camera
	[handles.x_k_k, handles.p_k_k, handles.featuresInfo, handles.h_all, handles.z_all, handles.detection ] = removeTrackObject(handles.x_k_k,...
		handles.p_k_k, handles.featuresInfo, handles.h_all, handles.z_all, handles.detection );
	
	[handles.x_k_k, handles.p_k_k, handles.featuresInfo, handles.detection ] = detection( handles.outputFile, handles.x_k_k, ...
		handles.p_k_k, handles.featuresInfo, handles.frame_idx, handles.detection );
	
	handles = initNewUnknownFeatures( handles );
	%[handles.local_x_k_k, handles.local_p_k_k,
	%handles.local_featuresInfo]=extractLocalEKF(handles,handles.z_all);
	
	outputTrackingPar(handles.x_k_k, handles.p_k_k, handles.featuresInfo, handles.z_all, handles.outputFile, handles.frame_idx, handles.cam_container, handles.static_container, handles.moving_container );
	
	% save camera location
	handles.camHistory(:,end+1) = [handles.x_k_k(1:7)];
	handles.camGrnHistory(:,end+1)= [ handles.cam_container.position(:,handles.frame_idx+1) ; handles.cam_container.orientation(:,handles.frame_idx+1) ];
	
	% plots
	if handles.frame_idx>=0 && mod(handles.step,30)==0
		fprintf(1,'printing..\n');
		plotresults( handles );
		outImage( handles );
		handles.detection
	end
end

handles.detection
fclose(handles.outputFile);
