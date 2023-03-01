%% Simple Steering Configuration - Single Runs

clear
close all
clc

%% Parameters

% Set total number of runs
runs = 1000;

MaxIterations = 10000;
MaxGenerations = 100;
SampleHorizonTime = 0.5;
GoalThreshold = 0.25;

NumberOfStates = 3;
NumberOfControls = 2;
NumberOfGriddedStates = 3;
InitialState = [-3, -3, 1.5707];
GoalState = [3, 3, 0];
GridActiveStates = [1, 1, 1];
GridResolution = [0.3536, 0.3536, 0.0982];

RotationControls = {
        [0 -0.3927 0.3927 -0.1963 0.1963];
      };
  
LinearControls = {
        [0.5 1.0];
      };

%% Configuration
  
V = @(arr,r) arr(ceil(r * size(arr,1) / runs),:);

Configuration = cell(runs,1);
NumberOfSamples = zeros(1,runs);
for r = 1:runs

    LinearControl = cell2mat(V(LinearControls, r));
    RotationControl = cell2mat(V(RotationControls, r));
    
    SizeLinear = length(LinearControl);
    SizeRotation = length(RotationControl);
    NumberOfSamples(r) = SizeRotation * SizeLinear;
    Samples = zeros(1, NumberOfSamples(r)*V(NumberOfControls,r));

    for v = 1:SizeLinear
        for u = 1:SizeRotation
            Samples(2*(v-1)*SizeRotation + 2*(u-1) + 1) = LinearControl(v);
            Samples(2*(v-1)*SizeRotation + 2*(u-1) + 2) = RotationControl(u);
        end
    end 

    Configuration(r) = {[...
            V(MaxIterations,r) V(MaxGenerations,r) V(SampleHorizonTime,r)...
            V(NumberOfStates,r) V(NumberOfControls,r) V(GridResolution,r)...
            NumberOfSamples(r) Samples]};

end
    
 writecell(Configuration, '../csv/config.csv', 'Delimiter', ',')
