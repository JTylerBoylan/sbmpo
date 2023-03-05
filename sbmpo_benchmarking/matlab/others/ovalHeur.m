clear

%%

v_max = 1;
u_max = 1;

[q_10, q_20] = meshgrid(-5:0.1:5,-5:0.1:5);

tf = sqrt((q_10./v_max).^2 + (q_20./u_max).^2);

figure
contourf(q_10,q_20,tf)