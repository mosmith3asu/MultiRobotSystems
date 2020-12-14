
x0 = 5*rand(3,1)  % Initial x coordinates of robots
y0 = 5*rand(3,1)  % Initial y coordinates of robots
vx0 = -1 + 2*rand(3,1);  % Initial vx velocity components of robots
vy0 = -1 + 2*rand(3,1);  % Initial vy velocity components of robots

z0 = [x0; y0; vx0; vy0];  % Initial state

[t,z] = ode45(@flockingODE_orig,[0 20],z0);

figure
hold on
set(gca,'Fontsize',20);
grid on
plot(z(:,1),z(:,4),'k','Linewidth',2)  % trajectory of robot 1
plot(z(:,2),z(:,5),'b','Linewidth',2)  % trajectory of robot 2
plot(z(:,3),z(:,6),'r','Linewidth',2)  % trajectory of robot 3
plot(z(1,1),z(1,4),'k*','Markersize',10,'Linewidth',2)  % initial position of robot 1
plot(z(1,2),z(1,5),'b*','Markersize',10,'Linewidth',2)  % initial position of robot 2
plot(z(1,3),z(1,6),'r*','Markersize',10,'Linewidth',2)  % initial position of robot 3
legend('Robot 1','Robot 2','Robot 3');
xlabel('x position')
ylabel('y position')
title('Robot paths over 20 s, starting from stars')

