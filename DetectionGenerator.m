classdef DetectionGenerator < handle
    methods (Abstract)
        % Return the known target positions for the detection generator.
        % The detection generator may or may not know the true positions
        % depending on if it is using real IQ data or simulation.
        targetPose = targetPoses(obj);

        % Create detections. Output the known target position if that
        % information is available.
        [dets,targets] = detect(obj);
    end
end