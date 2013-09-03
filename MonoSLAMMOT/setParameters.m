%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Numerical values and graphics initialisation

warning('off');

% Sequence path and initial image
handles.sequencePath = 'D:\VSLAM2\synth19_input\\';
handles.outputPath = 'D:\VSLAM2\synth19_input\output\\';
%handles.sequencePath = '/nfs/m1/97/r97120/htdocs/synth19_input/';
%handles.outputPath = '/nfs/m1/97/r97120/htdocs/synth19_input/output/';

addpath(handles.sequencePath);
%execute the script setVSLAMParameters in the sequencePath
setVSLAMParameters;

S = loadGroundTruth([handles.sequencePath 'groundTruth.txt']);
handles.outputFile = fopen([handles.outputPath 'tracking.txt'],'wt');

handles.cam_container = S.cam_container;
handles.static_container = S.static_container;
handles.moving_container = S.moving_container;

% General
handles.step = 1;

%create a figure for the program
handles.figure_image = figure;
set( handles.figure_image, 'Position', [40, 80, 1200, 900] );

%break the figure into 4 subplot
%image in the subplot
handles.axes_image=subplot(2,2,1);

%top view in the subplot
handles.axes_topView=subplot(2,2,2);
view(180,0);
title('Top view of the 3D world.');
xlabel('X[m]');
ylabel('Y[m]');
zlabel('Z[m]');

%3d view in the subplot
handles.axes_3dView=subplot(2,2,3);
view(180,-90);
title('');
xlabel('X[m]');
ylabel('Y[m]');
zlabel('Z[m]');

%covariance matrix in the subplot
handles.axes_covariance=subplot(2,2,4);

handles.chi2inv_table = [0.95 2 5.99146454710798;
						 0.95 3 7.81472790325116;
						 0.95 6 12.59158724374398];

% random points to draw the points covariance
nPointsRand = 1000;
X = rand( 1, nPointsRand )-0.5;
Y = rand( 1, nPointsRand )-0.5;
Z = rand( 1, nPointsRand )-0.5;
theta = rand( 1, nPointsRand )-0.5;
phi = rand( 1, nPointsRand )-0.5;
lambda = rand( 1, nPointsRand )-0.5;
for i = 1:nPointsRand
	a = [X(i) Y(i) Z(i) theta(i) phi(i) lambda(i)];
	a = a/norm(a)*sqrt(handles.chi2inv_table(3,3));
	X(i) = a(1); Y(i) = a(2); Z(i) = a(3);
	theta(i) = a(4); phi(i) = a(5); lambda(i) = a(6);
end
handles.randSphere6D = [ X; Y; Z; theta; phi; lambda ];
handles.nPointsRand = nPointsRand;

% gui variables
handles.camHistory = [];
handles.camGrnHistory = [];
handles.featuresInfo = [];
