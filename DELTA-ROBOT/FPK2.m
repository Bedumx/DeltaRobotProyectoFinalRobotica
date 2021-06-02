function Coordenadas=FPK2(thetha1,thetha2,thetha3)


L = 220; % upper legs length
l = 334.92; % lower legs parallelogram length
Wb = 89.82; % planar distance from {0} to near base side
Wp = 27.5; % planar distance from {P} to near platform side
Up = 54.9983; % planar distance from {P} to a platform vertex
Sp = 95.26; % platform equilateral triangle side
h = 65.53
Sb = 311.17
Ub = 179.65
%centre of the figure
	x0 = 0.0
	y0 = 0.0
	z0 = 0.0
	
	y1 = -Wb - L * cos(thetha1) + Up
	z1 = -L * sin(thetha1)
	
	x2 = (sqrt(3)/2.0) * (Wb + L * cos(thetha2)) - Sp/2.0
	y2 = 0.5 * (Wb + L * cos(thetha2)) - Wp
	z2 = -L * sin(thetha2)
	
	x3 = (sqrt(3)/2.0) * (Wb + L * cos(thetha3)) + Sp/2.0
	y3 = 0.5 * (Wb + L * cos(thetha3)) - Wp
	z3 = -L * sin(thetha3)
	

	
	dnm = (y2 - y1)  * x3 - (y3 - y1) * x2
	
	w1 = power(y1,2) + power(z1,2)
	w2 = power(x2,2) +power(y2,2) + power(z2,2)
	w3 = power(x3,2) + power(y3,2) + power(z3,2)
	
	
	a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
	b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) /2.0 
	
	a2 = -(z2-z1)*x3 + (z3-z1)*x2 
	b2 = ( (w2-w1)*x3 - (w3-w1)*x2) / 2.0  
	
	a = a1 * a1 + a2 * a2 + power(dnm,2)
	b = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * power(dnm,2))  
	c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + power(dnm,2) * (z1 * z1 - L * L)
	
	d = power(b,2) - 4.0 * a * c
    

	
	x0 = -0.5 * (b + sqrt(d)) / a
	y0 = (a1*z0 + b1) / dnm
	z0 = (a2*z0 + b2) / dnm

coordenadas=[x0,y0,z0]
end 