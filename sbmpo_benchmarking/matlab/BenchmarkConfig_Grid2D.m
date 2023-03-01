%% Book Model Configuration - Grid Rresolution vs Horizon Time Benchmark

clear
close all
clc

%% Parameters

% Set total number of runs
runs = 10;

MaxIterations = 10000;
MaxGenerations = 100;
SampleHorizonTime = 0.25;
GoalThreshold = 0.25;

NumberOfStates = 2;
NumberOfControls = 2;
NumberOfGriddedStates = 2;
InitialState = [-3.0, -3.0];
GoalState = [3.0, 3.0];
GridResolution = [0.125, 0.125];

XControls = {
        [0 -1.0 1.0];
      };
  
YControls = {
        [0 -1.0 1.0];
      };

%% Configurration
  
Configuration = cell(runs,1);
NumberOfSamples = zeros(1,runs);
for r = 1:runs
V = @(arr,r) arr(ceil(r * size(arr,1) / runs),:);


    XControl = cell2mat(V(XControls, r));
    YControl = cell2mat(V(YControls, r));
    
    SizeVX = length(XControl);
    SizeVY = length(YControl);
    NumberOfSamples(r) = SizeVY * SizeVX;
    Samples = zeros(1, NumberOfSamples(r)*V(NumberOfControls,r));

    for v = 1:SizeVX
        for u = 1:SizeVY
            Samples(2*(v-1)*SizeVY + 2*(u-1) + 1) = XControl(v);
            Samples(2*(v-1)*SizeVY + 2*(u-1) + 2) = YControl(u);
        end
    end 

    Configuration(r) = {[...
            V(MaxIterations,r) V(MaxGenerations,r) V(SampleHorizonTime,r)...
            V(NumberOfStates,r) V(NumberOfControls,r) V(GridResolution,r)...
            NumberOfSamples(r) Samples]};

end
    
 writecell(Configuration, '../csv/config.csv', 'Delimiter', ',')
