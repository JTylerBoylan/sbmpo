%% Book Model Configuration - Grid Rresolution vs Horizon Time Benchmark

clear
close all
clc

%% Parameters

% Set total number of runs
runs = 1840;

MaxIterations = 30000;
MaxGenerations = 300;
SampleHorizonTime = repmat((0.1:0.1:4.0)',[46 1]);
SampleHorizonTimeIncrement = SampleHorizonTime ./ 2;
GoalThreshold = 0.3;

NumberOfStates = 3;
NumberOfControls = 2;
NumberOfGriddedStates = 2;
InitialState = [0, 0, 1.5707];
GoalState = [5, 5, 0];
InitialControl = [0, 0];
GridActiveStates = [1, 1, 0];
GridResolution = [0.05:0.01:0.5; 0.05:0.01:0.5]';

RotationControls = {
        [0 -0.3927 0.3927];
      };
  
LinearControls = {
        [0.3 0.5];
      };

%% Configurration
  
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

    Configuration(r) = {[V(MaxIterations,r) V(MaxGenerations,r) V(SampleHorizonTime,r)...
            V(SampleHorizonTimeIncrement,r)  V(GoalThreshold,r) ...
            V(NumberOfStates,r) V(NumberOfControls,r) V(NumberOfGriddedStates,r) ...
            V(InitialState,r) V(GoalState,r) V(InitialControl,r) ...
            V(GridActiveStates,r) V(GridResolution,r)...
            NumberOfSamples(r) Samples]};

end
    
 writecell(Configuration, '../config/book_model_config.csv', 'Delimiter', ',')
