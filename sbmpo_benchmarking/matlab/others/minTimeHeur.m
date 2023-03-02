clear
close all
clc

%%

u_min = -1;
u_max = 1;
[q_10, q_20] = meshgrid(-5:0.1:5,-5:0.1:5);

use_eqA = q_10 + q_20.*abs(q_20)./(2*u_max) >= 0;
use_eqB = q_10 - q_20.*abs(q_20)./(2*u_min) < 0;

BA = 2.*q_20./u_min;
CA = (q_20.^2 + 2.*(u_max-u_min).*q_10)./(u_min*u_max);

BB = 2.*q_20./u_max;
CB = (q_20.^2 - 2.*(u_max-u_min).*q_10)./(u_min*u_max);

B = use_eqA.*BA + use_eqB.*BB;
C = use_eqA.*CA + use_eqB.*CB;

tf = (-B + sqrt(B.^2 - 4.*C))/2;

figure
hold on
contourf(q_10,q_20,tf)
colorbar
xlabel('X');
ylabel('V');
title('t_f')