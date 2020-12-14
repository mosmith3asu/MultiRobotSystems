function x_dot = ODE_Fun(t,species)
%ODE_FUN Summary of this function goes here
%   Detailed explanation goes here
x = species(1);
y1 = species(2);
y2 = species(3);
z1 = species(4);
z2 = species(5);
w1 = species(6);
w2 = species(7);


a1 =2.0; a2 = 4.0;
b1=1.2; b2=0.7;
g1=0.5; g2=1.0;
k1=1.5; k2=0.4;
u1=0.7; u2=1.2;
e1=0.9; e2=0.4;
n1 =1.1; n2=0.6;

M=[
1	0	1	0	0	0	0	0	0	0	0;
1	1	0	0	1	0	0	0	1	0	0;
0	0	1	1	0	0	1	0	0	1	0;
0	1	0	0	0	1	0	0	0	0	0;
0	0	0	1	0	0	0	1	0	0	0;
0	0	0	0	0	0	0	0	0	1	1;
0	0	0	0	0	0	0	0	1	0	1;
];

K = [
a1	-b1	0	0	0	0	0	0	0       0       0;
-a1	b1	0	0	0	0	0	0	0       0       0;
0	0	a2	-b2	0	0	0	0	0       0       0;
0	0	-a2	b2	0	0	0	0	0       0       0;
0	0	0	0	k1	-g1	0	0	0       0       0;
0	0	0	0	-k1	g1	0	0	0       0       0;
0	0	0	0	0	0	k2	-g2	0       0       0;
0	0	0	0	0	0	-k2	g2	0       0       0;
0	0	0	0	0	0	0	0	(e1+n1)	-e2     -u1;
0	0	0	0	0	0	0	0	-e1     (e2+n2)	-u2;
0	0	0	0	0	0	0	0	-n1     -n2     (u1+u2);
];

x_dot = -1*M*K*[
    x*y1;
    z1*y1;
    x*y2;
    z2*y2;
    y1;
    z1;
    y2;
    z2;
    w2*y1;
    w1*y2;
    w1*w2];

end

