clear
close all

%%

b = 10; % bounds
res = 100; % resolution
xlin = linspace(-b,b,res);
ylin = linspace(-b,b,res);

% Map
[X, Y] = meshgrid(xlin,ylin);

% Heading difference
dQ = abs( atan2(Y,X) );

% Position difference
dP = sqrt(X.^2 + Y.^2);

% Path length
LP = dP + dQ;

figure
contourf(X,Y,LP);
title("Path Length")