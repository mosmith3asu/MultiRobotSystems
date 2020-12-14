clc
clear all
close all
hold off
%% Variable Definition
% Summary of example objective
N=6;    %number of robots
T = 50; %simulation time

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
disp("L = ")
disp(L)

%% HW1.D
% Description of second code block
x0 = [ 3;1.5;0.25;-1;-2.5;-3.5];
[t,x] = ode45(@digraphConsODE,[0 T],x0);
figure('Name','Problem 1 part C')
hold on
set(gca,'Fontsize',20);
grid on

plot(t,x(:,1),'k','Linewidth',2)
plot(t,x(:,2),'b','Linewidth',2)
plot(t,x(:,3),'r','Linewidth',2)
plot(t,x(:,4),'g','Linewidth',2)
plot(t,x(:,5),'y','Linewidth',2)
plot(t,x(:,6),'c','Linewidth',2)

cons_reached = x(end,1)

ave_cons = [];
for i = 1:size(t,1)
    ave_cons = [ave_cons mean(x0)];
end

plot(t,ave_cons,'k','Linewidth',2,'LineStyle',':')

legend('x_1(t)','x_2(t)','x_3(t)','x_4(t)','x_5(t)','x_6(t)','Average Consensus');
xlabel('Time')
ylabel('Information states')

%% HW 2.D(i)
x0 = 10*rand(6,1);  % Initial x coordinates of robots
y0 = 10*rand(6,1);  % Initial y coordinates of robots
vx0 = -1 + 2*rand(6,1);  % Initial vx velocity components of robots
vy0 = -1 + 2*rand(6,1);  % Initial vy velocity components of robots
z0 = [x0; y0; vx0; vy0];  % Initial state

[t,z] = ode45(@flockingODE,[0 T],z0);

figure('Name','Problem 2 part D(i)')
hold on
set(gca,'Fontsize',20);
grid on

plot(z(:,1),z(:,7),'k','Linewidth',2)  % trajectory of robot 1
plot(z(:,2),z(:,8),'b','Linewidth',2)  % trajectory of robot 2
plot(z(:,3),z(:,9),'r','Linewidth',2)  % trajectory of robot 3
plot(z(:,4),z(:,10),'g','Linewidth',2)  % trajectory of robot 1
plot(z(:,5),z(:,11),'y','Linewidth',2)  % trajectory of robot 2
plot(z(:,6),z(:,12),'c','Linewidth',2)  % trajectory of robot 3

plot(z(1,1),z(1,7),'k*','Markersize',10,'Linewidth',2)  % initial position of robot 1
plot(z(1,2),z(1,8),'b*','Markersize',10,'Linewidth',2)  % initial position of robot 2
plot(z(1,3),z(1,9),'r*','Markersize',10,'Linewidth',2)  % initial position of robot 3
plot(z(1,4),z(1,10),'g*','Markersize',10,'Linewidth',2)  % initial position of robot 1
plot(z(1,5),z(1,11),'y*','Markersize',10,'Linewidth',2)  % initial position of robot 2
plot(z(1,6),z(1,12),'c*','Markersize',10,'Linewidth',2)  % initial position of robot 3

legend('Robot 1','Robot 2','Robot 3','Robot 4','Robot 5','Robot 6');
xlabel('x position')
ylabel('y position')
title('Robot paths over 50 s, starting from stars')

%% HW 2.D(ii)
% Plot distances between neighboring robots over time

figure('Name','Problem 2 part D(ii)')
hold on
set(gca,'Fontsize',20);
grid on

Ri = [z(:,1:N) z(:,N+1:N*2)];

r1 = [z(:,1) z(:,N+1)];
r2 = [z(:,2) z(:,N+2)];
r3 = [z(:,3) z(:,N+3)];
r4 = [z(:,4) z(:,N+4)];
r5 = [z(:,5) z(:,N+5)];
r6 = [z(:,6) z(:,N+6)];

%plot neigboring distances
colors = ['k','b','r','g','y','c','m'];
styles = ['-',':','-.'];
legend_list = [];

Nd = dist_array(r1,r2);
plot(t,Nd,colors(1),'Linewidth',2,'LineStyle','-')
legend_list = [legend_list "r1~r2"];

Nd = dist_array(r2,r4);
plot(t,Nd,colors(2),'Linewidth',2,'LineStyle','-')
legend_list = [legend_list "r2~r4"];

Nd = dist_array(r4,r6);
plot(t,Nd,colors(3),'Linewidth',2,'LineStyle','-')
legend_list = [legend_list "r4~r6"];

Nd = dist_array(r6,r5);
plot(t,Nd,colors(4),'Linewidth',2,'LineStyle','-')
legend_list = [legend_list "r6~r5"];

Nd = dist_array(r5,r3);
plot(t,Nd,colors(5),'Linewidth',2,'LineStyle','-')
legend_list = [legend_list "r5~r3"];

Nd = dist_array(r3,r1);
plot(t,Nd,colors(6),'Linewidth',2,'LineStyle','-')
legend_list = [legend_list "r3~r1"];

Nd = dist_array(r4,r5);
plot(t,Nd,colors(7),'Linewidth',2,'LineStyle','-')
legend_list = [legend_list "r4~r5"];

legend(legend_list);
xlabel('Time')
ylabel('Distance')
title('Distance Between Neighboring Sets')


    % Define distance function
    function distances = dist_array(ri,rj)
        distances = [];
        for t = 1:size(ri,1)
            distances = [distances; pdist([ri(t,:); rj(t,:)],'euclidean')];
            %distances = [distances; norm(ri-rj)];
        end
    end
