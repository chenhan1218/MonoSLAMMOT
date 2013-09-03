%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plotresults( handles )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot features in image and 3D world
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
step			=handles.step;

axes_image		=handles.axes_image;
axes_topView	=handles.axes_topView;
axes_3dView		=handles.axes_3dView;
axes_covariance	=handles.axes_covariance;

x_k_k			=handles.x_k_k;
p_k_k			=handles.p_k_k;
h_all			=handles.h_all;
S				=handles.S;
z_all			=handles.z_all;
featuresInfo	=handles.featuresInfo;
camHistory		=handles.camHistory;
camGrnHistory	=handles.camGrnHistory;
randSphere6D	=handles.randSphere6D;
nPointsRand		=handles.nPointsRand;
axisLimits		=handles.axisLimits;
chi2inv_table	=handles.chi2inv_table;

static_container=handles.static_container;
moving_container=handles.moving_container;
frame_idx		=handles.frame_idx;

% axes_image
axes(axes_image);
cla;
im = getImage( handles.sequencePath, handles.frame_idx );
image(im);
hold on;
axis image
set( gca, 'XTick', []);
set( gca, 'YTick', []);

% axes_topView
axes( axes_topView );
cla
hold on;
plotUncertainEllip3D( p_k_k( 1:3, 1:3 ), x_k_k( 1:3 ), chi2inv_table( 2, 3 ), [0.5 0.5 0.5], 0 );

%plot estimate of camera history
plot3(camHistory(1,1:step),camHistory(2,1:step),camHistory(3,1:step), 'LineWidth', 1.5, 'Color', [0.5 0.5 0.5]);
plotCameraWires( camHistory(:,step), [0.5 0.5 0.5]);

%plot ground truth of camera history
plot3(camGrnHistory(1,1:step),camGrnHistory(2,1:step),camGrnHistory(3,1:step), 'LineWidth', 1.5, 'Color', [0 0 0]);
plotCameraWires( camGrnHistory(:,step), [ 0 0 0 ]);

for i=1:numel(featuresInfo)
	% not plot the new initialize feature.
	if i > size(z_all,1)
		continue;
	end
	
	% location and uncertainty of the static feature
	xyztpl = x_k_k(13+9*(i-1)+1:13+9*(i-1)+6);
	p_xyztpl = p_k_k(13+9*(i-1)+1:13+9*(i-1)+6,13+9*(i-1)+1:13+9*(i-1)+6);
	[ xyz ] = XYZTPL2XYZ( xyztpl );
	
	% draw the point and the uncertainy into the 3d view
	plot3( xyz(1), xyz(2), xyz(3), 'o', 'MarkerEdgeColor','k',  'MarkerSize', 3 );
	pointIdx = featuresInfo(i).positionInGroundTruth;
	
	if featuresInfo(i).estType=='s'
		plotUncertainSurfaceXZ( p_xyztpl, xyztpl, chi2inv_table(2,3), [0 0 1], randSphere6D, nPointsRand );
		text( xyz(1), xyz(2), xyz(3), sprintf( '%d', featuresInfo(i).pointIndex ), 'color', [0 0 1], 'FontSize', 12 );
	elseif featuresInfo(i).estType=='m'
		plotUncertainSurfaceXZ( p_xyztpl, xyztpl, chi2inv_table(2,3), [1 0 0], randSphere6D, nPointsRand );
		text( xyz(1), xyz(2), xyz(3), sprintf( '%d', featuresInfo(i).pointIndex ), 'color', [1 0 0], 'FontSize', 12 );
	else
		plotUncertainSurfaceXZ( p_xyztpl, xyztpl, chi2inv_table(2,3), [0 1 0], randSphere6D, nPointsRand );
		text( xyz(1), xyz(2), xyz(3), sprintf( '%d', featuresInfo(i).pointIndex ), 'color', [0 1 0], 'FontSize', 12 );
	end
	
	if featuresInfo(i).type == 's'
		gt = static_container(pointIdx).position;
		plot3( gt(1), gt(2), gt(3), 's', 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b', 'MarkerSize', 3 );
	else
		index = find( moving_container(pointIdx).frame == frame_idx );
		if ~isempty(index)
			gt = moving_container(pointIdx).position(:,index);
			plot3( gt(1), gt(2), gt(3), 's', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r', 'MarkerSize', 3 );
			gt = moving_container(pointIdx).position(:,max(index-30,1):index);
			plot3( gt(1,:), gt(2,:), gt(3,:), '-r', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r', 'MarkerSize', 3, 'LineWidth', 2 );
		end
	end
	
end

view(180,0);
axis(axisLimits);
xlabel('X[m]');
ylabel('Y[m]');
zlabel('Z[m]');

% copy the 3d view into another axes and change view to top view
axes( axes_3dView );
cla;
compcopy(axes_topView,axes_3dView)
view(180,-90);
axis(axisLimits);

plotImage( axes_image, h_all, S, z_all, 'b', 'c', 'g', chi2inv_table(1,3), featuresInfo );

