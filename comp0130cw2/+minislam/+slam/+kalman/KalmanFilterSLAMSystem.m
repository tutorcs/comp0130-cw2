classdef KalmanFilterSLAMSystem < minislam.slam.VehicleSLAMSystem
    
    % This is a very minimal Kalman filter; its purpose is to let me do
    % some debugging and comparison. It is not fully featured.
    
    properties(Access = protected)
        
        % Kalman filter mean
        xEst;
        PEst;
        
        % Kalman filter covariance
        xPred;
        PPred;
        
        % Store of the mean and covariance values for the vehicle
        timeStore;
        xEstStore;
        PEstStore;
    end
    
    methods(Access = public)
        
        function this = KalmanFilterSLAMSystem()
            this = this@minislam.slam.VehicleSLAMSystem();
            this.xEstStore = NaN(3, 1);
            this.PEstStore = NaN(3, 1);
            this.xEst = NaN(3, 1);
            this.PEst = NaN(3, 3);
        end
        
        function [x,P] = robotEstimate(this)
            x = this.xEst(1:3);
            P = this.PEst(1:3, 1:3);
        end
        
        function [T, X, PX] = robotEstimateHistory(this)
            T = this.timeStore;
            X = this.xEstStore;
            PX = this.PEstStore;
        end
        
        function [x, P, landmarkIds] = landmarkEstimates(this)
            landmarkIds = [];
            x = NaN(2, 0);
            P = NaN(2, 2, 0);
        end
        
        function recommendation = recommendOptimization(this)
            recommendation = true;
        end
        
        function processEvents(this, events)
            % Handle the events
            processEvents@minislam.slam.VehicleSLAMSystem(this, events);
            
            % Store the estimate for the future
            this.timeStore(:, this.stepNumber) = this.currentTime;
            this.xEstStore(:, this.stepNumber) = this.xEst(1:3);
            this.PEstStore(:, this.stepNumber) = diag(this.PEst(1:3, 1:3));
        end
        
        
        function optimize(~, ~)
            % Nothing to do
        end
    end
       
    methods(Access = protected)
                    
        function handleInitialConditionEvent(this, event)
            this.xEst = event.data;
            this.PEst = event.covariance;
            this.xPred = this.xEst;
            this.PPred = this.PEst;
        end
        
        function handleNoPrediction(this)
            this.xPred = this.xEst;
            this.PPred = this.PEst;
        end
        
        function handlePredictToTime(this, time, dT)

            % You will need to write the code to implement the process
            % model which:
            % 1. Computes this.xPred
            % 2. Computes the Jacobian
            % 3. Computes the process noise

            v = transpose(mvnrnd([0,0,0], this.uCov, 1));
            Q = diag(v.^2); % Q: covariance of the nosie
            u = this.u; % wheel input
            psi_k = this.xEst(3); % heading from the state

            s_psi_k = sin(psi_k);
            c_psi_k = cos(psi_k);
            M_k = [c_psi_k , -s_psi_k , 0;
                   s_psi_k, c_psi_k, 0;
                    0,0,1];
            
            % next state prediction
            this.xPred = this.xEst+ dT*M_k*(u+v);
            this.xEst = this.xPred;
            
            % Jacobian matrix of state x
            Jx = [1, 0, -dT*s_psi_k*(u(1)+v(1)) - dT*v(2)*c_psi_k;
                  0, 1, dT*c_psi_k*(u(1)+v(1)) - dT*v(2)*s_psi_k
                  0, 0, 1];
              
            % Jacobian matrix of noise v
            Jv = [dT*c_psi_k, -dT*s_psi_k, 0;
                  dT*s_psi_k, dT*c_psi_k, 0;
                  0,0, dT];
              
            F = Jx;
            B = Jv;
            
            % covariance prediction
            this.PPred = F*this.PEst*transpose(F)+ B*Q*transpose(B);
            this.PEst = this. PPred;
            
            
        end
        
        function handleGPSObservationEvent(this, event)
            
            % You will need to write the code to implement the fusing the
            % platform position estimate with a GPS measurement
            R = event.covariance; %2 by 2 matrix
            z = event.data; % 2 by 1 vector
            x = this.xPred; % 3 by 1 vector
            P = this.PPred; % P is a 3 by 3 matrix
            
            % implementation of the update procedure
            
            % H is a 2 by 3 matrix that selects the first two items from x
            % ?? or -1 instead of 1 in the first row??
            H = [1,0,0
                0,1,0];
            
            % based on the shape here, H should be a 2 by 3 matrix to change from a 3 by 1 vector to a 2 by 1 vector
            z_hat = H * x; 
            r = z - z_hat; % r is a 2 by 1 vector
            C = P * transpose(H); % intermediate term; C is a 3 by 2 matrix
            S = H * C + R; % S is a 2 by 2 matrix
            W = C * inv(S); % W is a 3 by 2 matrix
            
            x = x + W * r % x is back to 3
            P = P - W * S * transpose(W); % W * S * transpose(W) shape: 3*2 * 2*2 * 2*3
            
            % update the estimate to the calculated 
            this.xEst = x;
            this.PEst = P;
        end
        
        function handleLandmarkObservationEvent(this, event)
            error('Not supported; this should only be used to implement odometry and GPS')
        end
 
    end
end