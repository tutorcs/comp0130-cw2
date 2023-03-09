% This script can be used to compare your SLAM system

import minislam.slam.g2o.*;

% Configure to disable other sensor types
parameters = minislam.event_generators.simulation.Parameters();

% Magic tuning for the no-prediction case
parameters.laserDetectionRange = 20;

% By setting true / false you can enable different combinations of sensors
parameters.enableGPS = false;
parameters.enableLaser = false;

% Set up the simulator and the output
simulator = minislam.event_generators.simulation.Simulator(parameters, 'q3-large-test');

% Create and run the different localization systems
g2oSLAMSystem = G2OSLAMSystem();
results = minislam.mainLoop(simulator, g2oSLAMSystem);


% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')

% Plot trajectory for predicted 
plotName = "Trajectory comparison for GPS measurement of " + num2str(parameters.gpsMeasurementPeriod) + "s";
minislam.graphics.FigureManager.getFigure(plotName);
clf
plot(results{1}.vehicleTrueStateHistory(1, :), results{1}.vehicleTrueStateHistory(2, :), 'LineWidth', 2)
hold on
plot(results{1}.vehicleStateHistory(1, :), results{1}.vehicleStateHistory(2, :))
hold off
legend('true trajectory','estimated trajectory by g2o');

g2oGraph = g2oSLAMSystem.optimizer();

numVertices = length(g2oGraph.vertices())
numEdges = length(g2oGraph.edges())
