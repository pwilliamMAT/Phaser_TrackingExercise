classdef RdgDetectionGenerator < DetectionGenerator
    % Create detections using the radar data generator.

    properties (Access = private)
        TargetMotion
        RadarDataGenerator
        ScanTime
        CurrentTime;
    end

    methods
        function obj = RdgDetectionGenerator()
            % Create the target motion
            obj.TargetMotion = WalkingTargetMotion(2,1,[0 30;-10 10]);

            % Create the rdg
            fc = 10e9;
            bw = 500e6;
            prf = 1000;
            pri = 1/prf;
            npulse = 32;
            cpi = pri*npulse;
            nscans = 14;
            scantime = 2*cpi*nscans; % Estimate for "true" hardware scantime
            obj.ScanTime = scantime;
            obj.CurrentTime = scantime;

            % Create radar model
            azres = 10;
            azbias = 0.5;
            far = 1e-5;
            rmax = 50;
            rres = bw2rangeres(bw);
            rrateres = dop2speed(prf/(2*npulse),freq2wavelen(fc));
            rratemax = dop2speed(prf/2,freq2wavelen(fc));
            obj.RadarDataGenerator = radarDataGenerator(1,UpdateRate=1/scantime,ScanMode="No scanning",...
                FieldOfView=[180 5],RangeLimits=[0 rmax],...
                RangeRateLimits=[-rratemax rratemax],...
                AzimuthResolution=azres,AzimuthBiasFraction=azbias,...
                RangeResolution=rres,RangeRateResolution=rrateres,...
                DetectionCoordinates="Sensor spherical",FalseAlarmRate=far,...
                CenterFrequency=fc,Bandwidth=bw,HasRangeRate=true);
        end

        function [dets,targets,t] = detect(obj)
            % Get detections
            advanceTime(obj);
            t = obj.CurrentTime;
            targets = targetPoses(obj);
            dets = obj.RadarDataGenerator(targets,t);
        end

        function targetPose = targetPoses(obj)
            targetPose = obj.TargetMotion.targetPoses(obj.ScanTime);
        end
    end

    methods (Access = private)
        function advanceTime(obj)
            obj.CurrentTime = obj.CurrentTime + obj.ScanTime;
        end
    end
end