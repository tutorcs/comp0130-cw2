% This class implements an event-based estimation system using g2o and
% the barebones for building up a minimal, ideal SLAM system. The system is
% event-based and responds to a sequence of events which are time stamped
% and served in order. To implement your SLAM system, you will need to
% implement various methods which mostly involve working with the graph.
% These methods are initially stubbed out and will generate exceptions if
% you try to call them.

classdef G2OSLAMSystem < minislam.slam.VehicleSLAMSystem
    
    properties(Access = protected)
                
        % Flag to run the detailed graph validation checks
        validateGraphOnInitialization;        
        
        % The graph used for performing estimation.
        graph;
        
        % The optimization algorithm
        optimizationAlgorithm;
        
        % The most recently created vehicle vertex.
        currentVehicleVertex;
        
        % The set of all vertices associated with the vehicle state over
        % time.
        vehicleVertices;
        vehicleVertexId;
        
        % The landmark vertices. Confusingly enough, "Map" here refers to
        % the data structure which is used to store the landmarks. (It
        % allows random access of landmarkID to landmark object.)
        landmarksMap;
        
        % Q4b
        % Delete vehicle prediction edges
        DeleteEdges;
    end
        
    methods(Access = public)
        
        % Create the localization system and start it up.
        function this = G2OSLAMSystem()
            
            % Call the base class constructor
            this = this@minislam.slam.VehicleSLAMSystem();
            
            % Create the graph and the optimization algorithm
            this.graph = g2o.core.SparseOptimizer();
            algorithm = g2o.core.LevenbergMarquardtOptimizationAlgorithm();
            this.graph.setAlgorithm(algorithm);
            
            % Do detailed checking by default
            this.validateGraphOnInitialization = true;
            
            % Preallocate; this is a lower bound on size
            this.vehicleVertices = cell(1, 10000);
            
            % No vehicle vertices initally set
            this.vehicleVertexId = 0;
            
            % Allocate the landmark map
            this.landmarksMap = containers.Map('KeyType', 'int64', 'ValueType', 'any');
            
            % Q4b
            % if set to 0, all edges
            this.DeleteEdges = 0;
        end
        
        % Get the underlying g2o graph
        function graph = optimizer(this)
            graph = this.graph;
        end
        
        % Set the flag to enable / disable validation. This spots errors in
        % graph construction, but repeated calls can slow things down by
        % 20% or more.
        function setValidateGraph(this, validateGraphOnInitialization)
            this.validateGraphOnInitialization = validateGraphOnInitialization;
        end
        
        function validateGraphOnInitialization = validateGraph(this)
            validateGraphOnInitialization = this.validateGraphOnInitialization;
        end
        
        % Recommend if an optimization is a good idea. Based on an event,
        % some activities (e.g., such as loop closing) can have a very big
        % impact on the estimates. Therefore, this method returns if the
        % localization algorithm thinks optimizing is a good idea. Here we
        % always return false because this gives the fastest results
        % because you just optimize once, right and the very end
        function recommendation = recommendOptimization(this)
             %recommendation = false;
            
            % This is how to do it after every 500 steps
            recommendation = rem(this.stepNumber, 100) == 0;
        end
        
          % ------------------------Q4.b-----------------------------------
        function question4b(this, remain)
            edges = this.graph.edges();
            edges_count = length(edges);
            fprintf('number of edge: %d',edges_count);
            vehicle_edges_count = 0;
            % remove vehicle edges using for loop
            for i = 1:edges_count
                
                current_edge_temp = edges(i);
                current_edge = current_edge_temp{1};
                % compare the class of the edge
                if class(current_edge) == "minislam.slam.g2o.VehicleKinematicsEdge"
                    % if the current edge is vehicle edges, count += 1
                    vehicle_edges_count = vehicle_edges_count + 1;
                    % if we want to remain the first edge
                    if vehicle_edges_count == 1 && remain == 1
                        disp("Remain first vehicle edge and remove the rest");
                    else
                        disp("Remove all vehicle edges");
                        this.graph.removeEdge(current_edge);
                        fprintf('number of vehicle edge: %d',vehicle_edges_count);
                    end
                end
            end
            fprintf('number of vehicle edge: %d',vehicle_edges_count);
        end
        
        % This method runs the graph optimiser and the outputs of the
        % optimisation are used as the initial conditions for the next
        % round of the graph. This makes it possible to iterative compute
        % solutions as more information becomes available over time.
        function optimize(this, maximumNumberOfOptimizationSteps)
            % ------------------------Q4.b-----------------------------------
            % Check if we want to do question 4b first
            % 1 means we want to do q4b
            % 0 means we don't want to do q4b
            if (this.DeleteEdges == 1)
                % 1 means we want first vehicle edge remains
                % 0 means we want all vehicle edges removed
                this.question4b(0);
            end    
            this.graph.initializeOptimization(this.validateGraphOnInitialization);
            if (nargin > 1)
                this.graph.optimize(maximumNumberOfOptimizationSteps);
            else
                this.graph.optimize();
            end
        end
                
        % Return the current mean and covariance estimate of the robot.
        % This is only valid after optimization has been called.
        function [x, P] = robotEstimate(this)
            [xS, PS] = this.graph.computeMarginals(this.currentVehicleVertex);
            x=full(xS);
            P=full(PS);
        end
        
        function [T, X, P] = robotEstimateHistory(this)
            
            % Extract the graph
            [xS, PS] = this.graph.computeMarginals();
            
            % Create the output array
            X = zeros(3, this.vehicleVertexId);
            P = zeros(3, this.vehicleVertexId);
            T = zeros(1, this.vehicleVertexId);
            
            % Copy the outputs over
            for v = 1 : this.vehicleVertexId
                idx = this.vehicleVertices{v}.hessianIndex();
                
                T(v) = this.vehicleVertices{v}.time();
                
                % Copy the estimate into the array. If the vertices is
                % fixed (conditioned), its estimate is okay. The covariance
                % is not explicitly defined, but has a value of zero.
                % Therefore we fill this manually.
                if (isempty(idx) == true)
                    X(:, v) = this.vehicleVertices{v}.estimate();
                    P(:, v) = zeros(3, 1);
                else
                    X(:, v) = full(xS(idx));
                    P(:, v) = full(diag(PS(idx, idx)));
                end
            end
        end
        
        % Return the means and covariances of the landmark estimates. These
        % are only valid after optimization has been called.
        function [x, P, landmarkIds] = landmarkEstimates(this) 
            
            landmarkVertices = values(this.landmarksMap);
            
            numberOfLandmarks = length(landmarkVertices);
            
            landmarkIds = NaN(1, numberOfLandmarks);
            x = NaN(2, numberOfLandmarks);
            P = NaN(2, 2, numberOfLandmarks);
            
            [xS, PS] = this.graph.computeMarginals();
            
            for l = 1 : numberOfLandmarks
                landmarkIds(l) = landmarkVertices{l}.landmarkId();
                idx = landmarkVertices{l}.hessianIndex();
                x(:, l) = full(xS(idx));
                if (isempty(idx == true))
                    P(:, :, l) = zeros(3, 3);
                else
                    P(:, :, l) = full(PS(idx, idx));
                end
            end
        end
    end
        
    % These are the methods you will need to overload
    methods(Access = protected)
                        
        % Declare bodies all of these methods. This makes it possible to
        % instantiate this object.
         
        function handleInitialConditionEvent(this, event)
            
            % Add the first vertex and the initial condition edge
            this.currentVehicleVertex = minislam.slam.g2o.VehicleStateVertex(this.currentTime);
            this.currentVehicleVertex.setEstimate(event.data);
            this.currentVehicleVertex.setFixed(true);
            this.graph.addVertex(this.currentVehicleVertex);
            this.vehicleVertexId = 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;
       end
       
        function handleNoPrediction(~)
            % Nothing to do
        end
        
        function handlePredictToTime(this, time, dT)
            
            % Create the next vehicle vertex and add it to the graph
            this.currentVehicleVertex = minislam.slam.g2o.VehicleStateVertex(time);
            this.graph.addVertex(this.currentVehicleVertex);
            
            % Update vehicle vertex
            previousX = this.vehicleVertices{this.vehicleVertexId}.estimate;
            
            % Define M matrix for the intermediate pose
            previousHeading = previousX(3);
            Mi = [cos(previousHeading), -sin(previousHeading), 0;
                sin(previousHeading), cos(previousHeading), 0;
                0,0,1];
            
            u = this.u;% input
            Q = this.uCov;% covariance matrix
%             v = sqrtm(Q) * randn(3, 1); % zero mean (unit variance) gaussian noise
            v = transpose(mvnrnd([0,0,0], this.uCov, 1));
            x_predicted = previousX + dT*Mi*(u+v); % update states
            % process model
            
            x_predicted(3) = g2o.stuff.normalize_theta(x_predicted(3)); % normalise the angles
               
            this.currentVehicleVertex.setEstimate(x_predicted) %the prediction is just a 3x1 matrix - X,Y,Angle 
            
            %now add the edges
            odometry = this.u;
            omegaQ = inv(Q); % information matrix is the inverse of Q
            processEdge = minislam.slam.g2o.VehicleKinematicsEdge(dT); % get kinematic edge
            processEdge.setVertex(1,this.vehicleVertices{this.vehicleVertexId}); % set the previous vertex
            processEdge.setVertex(2,this.currentVehicleVertex); % set the current vertex
            processEdge.setMeasurement(odometry) %set the measurements
            processEdge.setInformation(omegaQ) % set the information matrix
            this.graph.addEdge(processEdge); % add the new edge
     
            % Bump the indices
            this.vehicleVertexId = this.vehicleVertexId + 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;
        end
        
        function handleGPSObservationEvent(this, event)
            
            % Handle a GPS measurement
            sigmaR = event.covariance;
            z = event.data;
            
            omegaR = inv(sigmaR); %information matrix for measurements
            
            currentV = this.vehicleVertices{this.vehicleVertexId}; % get the latest updated vertex
            
            gpsEdge = minislam.slam.g2o.GPSMeasurementEdge();
            gpsEdge.setVertex(1,currentV); %only need to add once vertex as it is a unary measurement edge
            gpsEdge.setMeasurement(z);
            gpsEdge.setInformation(omegaR);
            this.graph.addEdge(gpsEdge);
        end
        
        function handleLandmarkObservationEvent(this, event)
            
            % Iterate over all the landmark measurements
            for l = 1 : length(event.landmarkIds)
                
                % Get the landmark vertex associated with this measurement.
                % If necessary, a new landmark vertex is created and added
                % to the graph.
                [landmarkVertex, newVertexCreated] = this.createOrGetLandmark(event.landmarkIds(l));
                z = event.data(:, l);
                
                %error('handleLandmarkObservationEvent: implement me');

                if newVertexCreated
                    this.graph.addVertex(landmarkVertex);
                    landmarkVertex.setEstimate([0;0]);
                end

                processModelEdge = minislam.slam.g2o.LandmarkRangeBearingEdge();
                processModelEdge.setVertex(1,this.vehicleVertices{this.vehicleVertexId}); 
                processModelEdge.setVertex(2,this.landmarksMap(event.landmarkIds(l)));
                processModelEdge.setMeasurement(z)                
                omegaQ = inv(event.covariance);          
                processModelEdge.setInformation(omegaQ)
                this.graph.addEdge(processModelEdge);
                
            end
                
        end
   end
    
    methods(Access = protected)
        
        % This method returns a landmark associated with landmarkId. If a
        % landmark exists already, it is returned. If it does not exist, a
        % vertex is created and is added to the graph.
        function [landmarkVertex, newVertexCreated] = createOrGetLandmark(this, landmarkId)
            
            % If the landmark exists already, return it
            if (isKey(this.landmarksMap, landmarkId) == true)
                landmarkVertex = this.landmarksMap(landmarkId);
                newVertexCreated = false;
                return
            else
                landmarkVertex = minislam.slam.g2o.LandmarkVertex(landmarkId);
                newVertexCreated = true;
                this.landmarksMap(landmarkId) = landmarkVertex; 
            end
            
        end
                
    end
end
