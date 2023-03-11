


w = linspace(1, 1E3, 100);

T = 4.8*exp(-0.115*w) + 0.5;

figure
plot(log10(w),T)
axis([0 3 -4 6])