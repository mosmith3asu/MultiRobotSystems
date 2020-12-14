% Function that integrates the ODE model dx/dt = -L*x

function dx = digraphConsODE(t,x)

%% Variable Definition
% Summary of example objective
a = [
0	0.3	0	0	0	0;
0.2	0	0	0	0	0;
0.1	0	0	0	0.8	0;
0	0.4	0	0	0	0.6;
0	0	0.9	0.5	0	0;
0	0	0	0	0.7	0;
];
%% HW1.A
% Description of first code block
L = [
(a(1,2)+a(1,3)+a(1,4)+a(1,5)+a(1,6))	-a(1,2)	-a(1,3)	-a(1,4)	-a(1,5)	-a(1,6)
-a(2,1)	(a(2,1)+a(2,3)+a(2,4)+a(2,5)+a(2,6))	-a(2,3)	-a(2,4)	-a(2,5)	-a(2,6)
-a(3,1)	-a(3,2)	(a(3,1)+a(3,2)+a(3,4)+a(3,5)+a(3,6))	-a(3,4)	-a(3,5)	-a(3,6)
-a(4,1)	-a(4,2)	-a(4,3)	(a(4,1)+a(4,2)+a(4,3)+a(4,5)+a(4,6))	-a(4,5)	-a(4,6)
-a(5,1)	-a(5,2)	-a(5,3)	-a(5,4)	(a(5,1)+a(5,2)+a(5,3)+a(5,4)+a(5,6))	-a(5,6)
-a(6,1)	-a(6,2)	-a(6,3)	-a(6,4)	-a(6,5)	(a(6,1)+a(6,2)+a(6,3)+a(6,4)+a(6,5))
];

dx = -L*x;

