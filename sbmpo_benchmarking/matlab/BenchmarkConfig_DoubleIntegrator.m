%% Double Integrator Benchmark
% Jonathan Boylan
% 2023-03-01

clear
close all
clc

%% Parameters

% Non-configured parameters
InitialState = [0, 0];
GoalState = [10, 0];

% Configured states
MaxIterations = 1000;
MaxGenerations = 500;
SampleHorizonTime = 0.1;

NumberOfStates = 2;
NumberOfControls = 1;
NumberOfGriddedStates = 2;
GridResolution = [0.01, 0.001];

Controls = [-1.0 -0.67 -0.33 0 0.33 0.67 1.0];
NumberOfSamples = length(Controls);

%% Configurration

Configuration = {[...
        MaxIterations MaxGenerations SampleHorizonTime...
        NumberOfStates NumberOfControls GridResolution...
        NumberOfSamples Controls]};

    
 writecell(Configuration, '../csv/config.csv', 'Delimiter', ',')
