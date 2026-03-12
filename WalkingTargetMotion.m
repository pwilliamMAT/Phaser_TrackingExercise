classdef WalkingTargetMotion < handle
    
    % Handles target motion for some people walking around

    properties (Access = private)
        Targets
        Velocities
        BoundingBox
        WalkingSpeed
    end

    methods
        function obj = WalkingTargetMotion(ntargets,walkingSpeed,boundingBox)
            obj.BoundingBox = boundingBox;
            obj.WalkingSpeed=  walkingSpeed;

            % Define targets walking around in our bounding box
            targets = cell(1,ntargets);
            vels = cell(1,ntargets);
            for i = 1:ntargets
                pos = randomPos(obj);
                vel = randomVel(obj);
                targets{i} = phased.Platform(InitialPosition=pos,VelocitySource="Input port"); 
                vels{i} = vel;
            end

            % Save current targets and velocities
            obj.Targets = targets;
            obj.Velocities = vels;
        end

        function targetPose = targetPoses(obj,time)
            % Initialize targets
            targetPose = struct('PlatformID',[], ...
                              'Position',[], ...
                              'Velocity',[]);

            % Get position for each target
            ntargets = length(obj.Targets);
            for itarget = 1:ntargets
                % Get position and velocity
                ctarget = obj.Targets{itarget};
                cvel = obj.Velocities{itarget};
                cpos = ctarget(time,cvel);
        
                % Update target input
                targetPose(itarget) = struct('PlatformID',itarget, ...
                                           'Position',cpos', ...
                                           'Velocity',cvel');

                % Update if outside of bounding box
                if obj.oob(cpos)
                    % Update position and velocity if out of bounds
                    obj.Velocities{itarget} = obj.newVelocity(cpos);
                end
            end
        end
    end

    methods (Access = private)
        function pos = randomPos(obj)
            % Get a random position within the bounding box
            boundingBox = obj.BoundingBox;
            xstart = boundingBox(1,1);
            xend = boundingBox(1,2);
            x = rand*(xend-xstart)+xstart;
            ystart=boundingBox(2,1);
            yend = boundingBox(2,2);
            y = rand*(yend-ystart)+ystart;
            pos = [x;y;0];
        end
        
        function vel = randomVel(obj)
            % Get a random velocity at the given speed
            speed = obj.WalkingSpeed;
            velinit = [randn;randn;0];
            vel = velinit/norm(velinit)*speed;
        end
        
        function flag = oob(obj,pos)
            % Check if a pos is oob
            boundingBox = obj.BoundingBox;
            flag = pos(1) <= boundingBox(1,1) || pos(1) >= boundingBox(1,2) || pos(2) <= boundingBox(2,1) || pos(2) >= boundingBox(2,2);
        end
        
        function vel = newVelocity(obj,pos)
            % Get a unit vector pointing to center
            speed = obj.WalkingSpeed;
            boundingBox = obj.BoundingBox;
            xstart = boundingBox(1,1);
            xstop = boundingBox(1,2);
            xspan = xstop-xstart;
            xc = (xspan)/2+xstart;
            ystart = boundingBox(2,1);
            ystop = boundingBox(2,2);
            yspan = ystop-ystart;
            yc = (yspan)/2+ystart;
            
            % Point velocity towards center with some randomness
            newvec = [(xc+randn*xspan/4)-pos(1);(yc+randn*yspan/4)-pos(2);0];
            vel = newvec/norm(newvec)*speed;
        end
    end
end