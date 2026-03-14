classdef TransceiverDetectionGenerator < DetectionGenerator
    properties (Access = private)
        TargetMotion
        CurrentTime = 0
        ScanTime = 1
        Fc
        RangeDoppler
        SteeringVector
    end

    properties
        Radar
    end

    methods
        function obj = TransceiverDetectionGenerator(nTargets)
            % Create the target motion
            obj.TargetMotion = WalkingTargetMotion(nTargets,1,[0 30;-10 10]);

            % Setup a radar transceiver for iq modeling
            fc = 10e9;
            prf = 500;
            nPulses = 16;
            rampbandwidth = 500e6;
            fs = 2*rampbandwidth;

            % Setup scan time property
            obj.ScanTime = nPulses/prf;
            obj.Fc = fc;
            
            % Setup waveform
            waveform = phased.LinearFMWaveform(SampleRate=fs,...
                PulseWidth=1/(2*prf),PRF=prf,SweepBandwidth=rampbandwidth,...
                SweepInterval="Symmetric");

            % Setup transmitter
            transmitter = phased.Transmitter(PeakPower=1e-4,Gain=0);

            % Transmit antenna is just isotropic
            txel = phased.IsotropicAntennaElement;
            txant = phased.Radiator(Sensor=txel,OperatingFrequency=fc);

            % Receiver is an 8 element ula
            rxula = phased.ULA(NumElements=8,ElementSpacing=freq2wavelen(fc)/2);
            rxant = phased.Collector(Sensor=rxula,OperatingFrequency=fc);

            % Setup receiver
            receiver = phased.ReceiverPreamp(SampleRate=fs,Gain=0,NoiseFigure=10);

            % Setup transceiver
            obj.Radar = radarTransceiver(Waveform=waveform,Transmitter=transmitter,...
                TransmitAntenna=txant,ReceiveAntenna=rxant,Receiver=receiver,...
                NumRepetitions=nPulses);

            % Setup steering vector
            obj.SteeringVector = phased.SteeringVector(SensorArray=rxula);

            % Setup range-Doppler
            obj.RangeDoppler = phased.RangeDopplerResponse(SampleRate=rampbandwidth);
        end

        function [data,targets,t] = detect(obj)
            
        end

        function data = receiveIQ(obj,pointdir)
            % We loop through a number of pointing directions to get the
            % angle of arrival
            npointdir = length(pointdir);
            data = cell(1,npointdir);
            for i = 1:npointdir
                % Current point direction
                cdir = pointdir(i);

                % Get IQ data
                advanceTime(obj);
                t = obj.CurrentTime;
                targets = targetPoses(obj);
                iq = obj.Radar(targets,t);

                % Analog bfs
                sv = obj.SteeringVector(obj.Fc,cdir);

                % Beamform iq into 2 channels
                iqsize = size(iq);
                iqbf = zeros(iqsize(1),2,iqsize(3));
                for j = 1:iqsize(3)
                    iqbf(:,1,j) = iq(:,1:4,j)*conj(sv(1:4));
                    iqbf(:,2,j) = iq(:,5:8,j)*conj(sv(5:8));
                end

                data{i} = struct('IQ',iqbf,'PointDir',cdir,'Targets',targets,'Radar',obj.Radar);
            end
        end

        function targetPose = targetPoses(obj)
            targetPose = obj.TargetMotion.targetPoses(obj.ScanTime);

            % Add signatures
            for i = 1:numel(targetPose)
                targetPose(i).Signatures = rcsSignature(Pattern=5);
            end
        end
    end

    methods (Access = private)
        function advanceTime(obj)
            obj.CurrentTime = obj.CurrentTime + obj.ScanTime;
        end
    end
end