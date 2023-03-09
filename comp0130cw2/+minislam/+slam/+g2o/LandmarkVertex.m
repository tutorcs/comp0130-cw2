% The object state vertex:
% x(1) - x
% x(2) - y

% Note that the oplus method has to be used to account for angular
% discontinuities

classdef LandmarkVertex < g2o.core.BaseVertex
   
    properties(Access = public)
    landmarkId
    end
    
    methods(Access = public)
        function this = LandmarkVertex(landmarkId)
            this=this@g2o.core.BaseVertex(2);
            this.landmarkId = landmarkId;
        end
        
        
        function oplus(this, update)
            
            % Add the update
            this.x = this.x + update;
        end
    end
end