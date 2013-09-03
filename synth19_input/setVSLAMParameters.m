handles.cam.k1 =	0;
handles.cam.k2 =	0;
handles.cam.nRows = 240;
handles.cam.nCols = 320;
handles.cam.Cx =	160;
handles.cam.Cy =	120;
handles.cam.f =	 0.85;
handles.cam.dx =	0.005;
handles.cam.dy =	0.005;
handles.cam.model = 1;


handles.cam.K =	 [	-170	0		160;
					0		-170	120;
					0		0		1 ];

handles.numberOfImages = 777;
handles.frame_idx=0;
handles.frameInteval=2;
% Time between frames
handles.deltat = 1/30*handles.frameInteval ; %s

handles.lambdaInit = 0.06;
handles.std_lambda = handles.lambdaInit/2;

% Covariance definitions
handles.sigma_imageNoise = 2; % pixels
handles.sigma_aNoise	 = 6; % m s^-2
handles.sigma_alphaNoise = 6; % rad s^-2

handles.axisLimits=[-5 10 -5 5 -5 10];
