clc;
clear all;
%% Homework Matlab Code 2 Mason Smith - MAE 598 MRS
%% Part (b) Code
% Homework Problem B - macroscopic model

x_0 = [.4; 0.1; 0.1; 0.1; 0.1; 0.1; 0.1];

t_span= [0 400];

%Function Defined in other file ODE_Fun.mat
[t,species] = ode45(@ODE_Fun,t_span,x_0);

%unpack species
x = species(:,1);
y1 = species(:,2);
y2 = species(:,3);
z1 = species(:,4);
z2 = species(:,5);
w1 = species(:,6);
w2 = species(:,7);

% Display species at Tf = 400
disp(x(end));
disp(y1(end));  
disp(y2(end));  
disp(z1(end));
disp(z2(end)); 
disp(w1(end)); 
disp(w2(end)); 

%Plot data
p = plot(t,y1,t,y2,t,z1,t,z2);
lw=2;
p(1).LineWidth = lw;
p(2).LineWidth = lw;
p(3).LineWidth = lw;
p(4).LineWidth = lw;

% Configure Plot
p(1).Color = "red";
p(2).Color = "green";
p(3).Color = "blue";
p(4).Color = "yellow";
%title('Population Proportion of Species over 400s (part b)')
%xlabel('Time (s)') 
%ylabel('Population Proportion') 
hold on;


%% Part (c) Code
% Homework Problem C - mesoscopic model using Gillespie's Direct Method

%Variable Definitions
N = 200; % number of agents
Tf = 400; % final time

Stoich_M = [
-1	1	-1	1	0	0	0	0	0	0	0	0	0	0;
0	0	0	0	-1	1	0	0	-1	1	-1	1	0	0;
0	0	0	0	0	0	-1	1	1	-1	0	0	1	-1;
1	-1	0	0	1	-1	0	0	0	0	0	0	0	0;
0	0	1	-1	0	0	1	-1	0	0	0	0	0	0;
0	0	0	0	0	0	0	0	1	-1	1	-1	0	0;
0	0	0	0	0	0	0	0	-1	1	0	0	-1	1;
];

[Ni,T] = GillespiesDirectMethod(Stoich_M,N,x_0,Tf);

x = Ni(1,:)/N;
y1 = Ni(2,:)/N;
y2 = Ni(3,:)/N;
z1 = Ni(4,:)/N;
z2 = Ni(5,:)/N;
w1 = Ni(6,:)/N;
w2 = Ni(7,:)/N;

disp(x(end));
disp(y1(end));  
disp(y2(end));  
disp(z1(end));
disp(z2(end)); 
disp(w1(end)); 
disp(w2(end)); 

p2 = plot(T,y1,T,y2,T,z1,T,z2);
lw = 0.2;
p2(1).LineWidth = lw;
p2(2).LineWidth = lw;
p2(3).LineWidth = lw;
p2(4).LineWidth = lw;
  
p2(1).Color = "red";
p2(2).Color = "green";
p2(3).Color = "blue";
p2(4).Color = "yellow";
title('Population Proportion of Species over 400s (part b)')
xlabel('Time (s)') 
ylabel('Population Proportion') 
legend('y_1(t)','y_2(t)','z_1(t)','z_2(t)')
%% Part (d) code
% Description of second code block
R=[
 1	0	0	-1	0	0	0;
-1	0	0	1	0	0	0;
1	0	0	0	-1	0	0;
-1	0	0	0	1	0	0;
0	1	0	-1	0	0	0;
0	-1	0	1	0	0	0;
0	0	1	0	-1	0	0;
0	0	-1	0	1	0	0;
0	1	-1	0	0	-1	1;
0	-1	1	0	0	1	-1;
0	1	0	0	0	-1	0;
0	-1	0	0	0	1	0;
0	0	-1	0	0	0	1;
0	0	1	0	0	0	-1;
    ];
Rank_R= rank(R)

hold off;
disp("Finished")