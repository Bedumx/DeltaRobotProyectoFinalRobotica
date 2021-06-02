function Coordenadas=FPK(theta1,theta2,theta3)


L = 220; % upper legs length
l = 334.92; % lower legs parallelogram length
Wb = 89.82; % planar distance from {0} to near base side
Wp = 27.5; % planar distance from {P} to near platform side
Up = 54.9983; % planar distance from {P} to a platform vertex
Sp = 95.26; % platform equilateral triangle side
%centre of the figure
hx=0.3;
ky=0.2;
%time
t0=0;
tf=1;
%sampling distance
p_1=.1;
%frequency
w1 = 1;
w2 = 4;
% sampling=(tf-t0)/p_1;
% 
% t=s*p_1;
% %equation for a obtaining lissajous figure
% xd(s) = 0.17*sin(w1*t)+hx;
% yd(s) = 0.17*cos(w1*t)+ky;
% zd(s) = 0.1*sin(w2*t)-0.7;
% % desired position
% d = [xd(s), yd(s), zd(s)];
% %IPK
% a = Wb - Up;
% b = (Sp/2.0) - ((sqrt(3.0)/2.0) * Wb);
% c = Wp - (Wb/2.0);
% E1 = (2.0 * L) * (d(2) + a);
% F1 = (2.0 * d(3)) * L;
% G1 = d(1)^2 + d(2)^2 + d(3)^2 + a^2 + L^2 + (2.0 * d(2) * a) - l^2;
% E2 = -L * ((sqrt(3.0) * (d(1) + b)) + d(2) + c);
% F2 = 2.0 * d(3) * L;
% G2 = d(1)^2 + d(2)^2 + d(3)^2 + b^2 + c^2 + L^2 + (2.0 * ((d(1) * b) +(d(2) * c))) - l^2;
% E3 = L * ((sqrt(3.0) * (d(1) - b)) - d(2) - c);
% F3 = 2.0 * d(3) * L;
% G3 = d(1)^2 + d(2)^2 + d(3)^2 + b^2 + c^2 + L^2 + (2.0 * (-(d(1) * b) + (d(2) * c))) - l^2;
% t1 = (-F1 - sqrt(F1^2 - G1^2 + E1^2)) / (G1 - E1);
% t2 = (-F2 - sqrt(F2^2 - G2^2 + E2^2)) / (G2 - E2);
% t3 = (-F3 - sqrt(F3^2 - G3^2 + E3^2)) / (G3 - E3);
% theta1 = 2*atan(t1);
% theta2 = 2*atan(t2);
% theta3 = 2*atan(t3);
%FPK
x1 = 0;
y1 = -Wb - (L*(cos(theta1))) + Up;
z1 = -L * (sin(theta1));
r1 = l;
x2 = (((sqrt(3))/2)*(Wb+(L*(cos(theta2))))) - (Sp/2);
y2 = 0.5*(Wb+(L*(cos(theta2)))) - Wp;
z2 = -L*(sin(theta2));
r2 = l;
x3 = (((-sqrt(3))/2)*(Wb+(L*(cos(theta3))))) + (Sp/2);
y3 = 0.5*(Wb+(L*(cos(theta3)))) - Wp;
z3 = -L*(sin(theta3));
r3 = l;
a11 = 2*(x3-x1);
a12 = 2*(y3-y1);
a13 = 2*(z3-z1);
a21 = 2*(x3-x2);
a22 = 2*(y3-y2);
a23 = 2*(z3-z2);
b1 = r1^2-r3^2-x1^2-y1^2-z1^2+x3^2+y3^2+z3^2;
b2 = r2^2-r3^2-x2^2-y2^2-z2^2+x3^2+y3^2+z3^2;
a1 = a11/a13 - a21/a23;
a2 = a12/a13 - a22/a23;
a3 = b2/a23 - b1/a13;
a4 = -a2/a1;

a5 = -a3/a1;
a6 = ((-a21*a4)-a22)/a23;
a7 = (b2-(a21*a5))/a23;
a = a4^2 + 1 + a6^2;
b = (2*a4)*(a5-x1) - (2*y1) + (2*a6)*(a7-z1);
c = a5*(a5-(2*x1)) + a7*(a7-(2*z1)) + x1^2 + y1^2 + z1^2 - r1^2;
yp = (-b + sqrt((b^2)-(4*a*c)))/(2*a);
yn = (-b - sqrt((b^2)-(4*a*c)))/(2*a);
xp = a4*yp + a5;
xn = a4*yn + a5;
zp = a6*yp + a7;
zn = a6*yn + a7;
%end effector's position
px1 = xp;
py1 = yp;
pz1 = zp;

coordenadas=[xp,yp,zp]
end 