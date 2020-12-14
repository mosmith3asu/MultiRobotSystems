% Function that integrates the ODE model governing the robots' dynamics

function dz = flockingODE(t,z)

L = [2 -1 -1; 
     -1 2 -1; 
     -1 -1 2];

x = z(1:3,1);    % x coordinates of robots
y = z(4:6,1);    % y coordinates of robots
vx = z(7:9,1);   % vx velocity components of robots
vy = z(10:12,1); % vy velocity components of robots

dx = vx;
dy = vy;
dvx = -L*vx;
dvy = -L*vy;
 
dz = [dx; dy; dvx; dvy];

