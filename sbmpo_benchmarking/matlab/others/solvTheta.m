clear
close all

%%

syms x y q r

eq1 = (y - r *sin(q))/(x - r*cos(q)) == tan(q + pi/2);
eq2 = subs(eq1, x, -x);

sol1 = solve(eq1, q);
sol1 = sol1(2);

sol2 = solve(eq2, q);
sol2 = sol2(2);

fun1 = matlabFunction(sol1);
fun2 = matlabFunction(sol2);

%%

b = 5;
res = 100;
xlin = linspace(-b,b,res);
ylin = linspace(-b,b,res);
R = ones(res);

[X, Y] = meshgrid(xlin,ylin);

qb1 = fun1(R,X+1,Y);
qb2 = fun2(R,X-1,Y);

qb1(abs(imag(qb1)) > 0.01) = NaN;
qb1 = real(qb1);

qb2(abs(imag(qb2)) > 0.01) = NaN;
qb2 = real(qb2);

qb1(qb1 < 0) = qb1(qb1 < 0) + 2.*pi;
qb2(qb2 < 0) = qb2(qb2 < 0) + 2.*pi;

figure
contourf(X,Y,qb1)

figure
contourf(X,Y,qb2)

%%

s1 = R.*qb1;
l1 = sqrt((X - R.*cos(qb1)).^2 + (Y - R.*sin(qb1)).^2);
d1 = s1+l1;

figure
contourf(X,Y,d1);

s2 = R.*qb2;
l2 = sqrt((X + R.*cos(qb2)).^2 + (Y - R.*sin(qb2)).^2);
d2 = s2+l2;

figure
contourf(X,Y,d2);

comb = min(d1, d2);
figure
contourf(X,Y,comb)
