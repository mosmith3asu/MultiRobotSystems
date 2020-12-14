% Function that integrates the ODE model governing the robots' dynamics

function dz = flockingODE(t,z)
N=6;
L= [
2	-1	-1	0	0	0;
-1	2	0	-1	0	0;
-1	0	2	0	-1	0;
0	-1	0	3	-1	-1;
0	0	-1	-1	3	-1;
0	0	0	-1	-1	2	

];

%x = z(1:6,1);    % x coordinates of robots
%y = z(7:12,1);   % y coordinates of robots
%vx = z(13:18,1);   % vx velocity components of robots
%vy = z(19:24,1); % vy velocity components of robots

x = z(1:N,1);    % x coordinates of robots
y = z(1*N+1:2*N,1);   % y coordinates of robots
vx = z(2*N+1:3*N,1);   % vx velocity components of robots
vy = z(3*N+1:4*N,1); % vy velocity components of robots


ri = [x y];

%Neighboring sets
% i = 1; N_j = {2,3}
ddi = [];
ddi = [ddi; directional_deriv(ri(1,:)-ri(2,:))];
%ddi = [ddi; directional_deriv(ri(1,:)-ri(3,:))];
dir_derivs = [sum(ddi(:,1)) sum(ddi(:,2))];

% i = 2; N_j = {1,4}
ddi = [];
%ddi = [ddi; directional_deriv(ri(2,:)-ri(1,:))];
ddi = [ddi; directional_deriv(ri(2,:)-ri(4,:))];
dir_derivs = [dir_derivs; sum(ddi(:,1)) sum(ddi(:,2))];

% i = 3; N_j = {1,5}
ddi = [];
ddi = [ddi; directional_deriv(ri(3,:)-ri(1,:))];
%ddi = [ddi; directional_deriv(ri(3,:)-ri(5,:))];
dir_derivs = [dir_derivs; sum(ddi(:,1)) sum(ddi(:,2))];

% i = 4; N_j = {2,5,6}
ddi = [];
%ddi = [ddi directional_deriv(ri(4,:)-ri(2,:))];
ddi = [ddi; directional_deriv(ri(4,:)-ri(5,:))];
ddi = [ddi; directional_deriv(ri(4,:)-ri(6,:))];
dir_derivs = [dir_derivs; sum(ddi(:,1)) sum(ddi(:,2))];

% i = 5; N_j = {3,4,6}
ddi = [];
ddi = [ddi; directional_deriv(ri(5,:)-ri(3,:))];
%ddi = [ddi; directional_deriv(ri(5,:)-ri(4,:))];
%ddi = [ddi; directional_deriv(ri(5,:)-ri(6,:))];
dir_derivs = [dir_derivs; sum(ddi(:,1)) sum(ddi(:,2))];

% i = 6; N_j = {4,5}
ddi = [];
%ddi = [ddi; directional_deriv(ri(6,:)-ri(4,:))];
ddi = [ddi; directional_deriv(ri(6,:)-ri(5,:))];
dir_derivs = [dir_derivs; sum(ddi(:,1)) sum(ddi(:,2))];

dx = vx;
dy = vy;
dvx = -L*vx-L*dir_derivs(:,1);
dvy = -L*vy-L*dir_derivs(:,2);


dz = [dx; dy; dvx; dvy];
end

function ans = directional_deriv(rij)
    norm_rij = norm(rij);
    ans = (2*log(norm_rij)/norm_rij - 1/norm_rij^2)*(rij/norm_rij);
end
