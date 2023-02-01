%% Book Model Configuration - Grid Rresolution vs Horizon Time Benchmark

clear
close all
clc

%% Parameters

GridResolutionXY = linspace(0.05, 0.50, 30);
HorizonTime = linspace(0.1, 1.5, 30);
GridResolutionTheta = 0.1963 * ones(1,length(GridResolutionXY));

MaxIterations = 30000;
MaxGenerations = 300;
SampleHorizonTime = repmat(HorizonTime',[length(GridResolutionXY) 1]);
GoalThreshold = 0.25;

NumberOfStates = 3;
NumberOfControls = 2;
NumberOfGriddedStates = 3;
InitialState = [0, 0, 1.5707];
GoalState = [5, 5, 0];
GridActiveStates = [1, 1, 1];
GridResolution = [GridResolutionXY; GridResolutionXY; GridResolutionTheta]';

RotationControls = {
        [0 -0.3927 0.3927];
      };
  
LinearControls = {
       [0.5 1.0];
      };

%% Configurration

% Set total number of runs
runs = length(GridResolutionXY)*length(HorizonTime);
  
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

    Configuration(r) = {[V(MaxIterations,r) V(MaxGenerations,r)...
            V(SampleHorizonTime,r) V(GoalThreshold,r) ...
            V(NumberOfStates,r) V(NumberOfControls,r) V(NumberOfGriddedStates,r) ...
            V(InitialState,r) V(GoalState,r)...
            V(GridActiveStates,r) V(GridResolution,r)...
            NumberOfSamples(r) Samples]};

end
    
 writecell(Configuration, '../config/book_model_config.csv', 'Delimiter', ',')
