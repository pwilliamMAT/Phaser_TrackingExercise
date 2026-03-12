% Create a detection generator
detectionGenerator = RdgDetectionGenerator();

% Create track generator
trackGenerator = TrackGenerator();

% Get Initial Position for Plot Initialization
targetpos = detectionGenerator.targetPoses();

% Reference Det for Plot Initialization
refDet = objectDetection.empty();

% Leverage visualization helper function
display = HelperTIRadarTrackingDisplay('XLimits',[-10 40],...
    'YLimits',[-10 10],...
    'MaxRange',30,...
    'CameraReferenceLines',zeros(2,0),...
    'RadarReferenceLines',zeros(2,0),...
    'PlotReferenceImage',0); % Turn off ref image
display(refDet,objectTrack.empty(0,1),targetpos);

% Init tracks
tracks = objectTrack.empty(0,1);

%% Track

for i = 1:100
    % Generate detections
    [dets,targets,t] = detectionGenerator.detect();

    % Generate tracks
    tracks = trackGenerator.track(dets,t);

    % Update display
    display(dets, tracks, targets); % clusteredDets,   refImage

    pause(0.4);
end

