%% Book Model Configuration - Grid Rresolution vs Horizon Time Benchmark

clear
close all
clc

%% Parameters

omega_bar = pi / 5;

% Set total number of runs
runs = 4;

MaxIterations = 10000;
MaxGenerations = 100;
SampleHorizonTime = 2.5;
GoalThreshold = 0.25;

NumberOfStates = 3;
NumberOfControls = 2;
NumberOfGriddedStates = 3;
InitialState = [0, 0, 1.5707];
GoalState = [5, 5, 0];
GridActiveStates = [1, 1, 1];
GridResolution = [0.125, 0.125, 0.1963];

RotationControls = {
        [0, +omega_bar, -omega_bar];
        [0, +omega_bar, -omega_bar, +omega_bar/2, -omega_bar/2];
        [0, +omega_bar, -omega_bar, +omega_bar/2, -omega_bar/2,...
            +omega_bar/4, -omega_bar/4];
        [0, +omega_bar, -omega_bar, +omega_bar/2, -omega_bar/2,...
            +omega_bar/4, -omega_bar/4, +omega_bar*3/4, -omega_bar*3/4];
      };
  
LinearControls = {
        [0.1 0.3 0.5]
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

    Configuration(r) = {[V(MaxIterations,r) V(MaxGenerations,r)...
            V(SampleHorizonTime,r)...
            V(NumberOfStates,r) V(NumberOfControls,r) V(NumberOfGriddedStates,r) ...
            V(GridActiveStates,r) V(GridResolution,r)...
            NumberOfSamples(r) Samples]};

end
    
 writecell(Configuration, '../config/book_model_config.csv', 'Delimiter', ',')
