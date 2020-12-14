
function dz = Final_FlockingODE(t,z)
global goToX;
global goToY;
global spill;
%spill
%class(spill)
spill.step(1);
map = spill.current_map();


N=size(z,1)/4;
scalar = 10;

x = z(1:N,1);    % x coordinates of robots
y = z(1*N+1:2*N,1);   % y coordinates of robots
vx = z(2*N+1:3*N,1);   % vx velocity components of robots
vy = z(3*N+1:4*N,1); % vy velocity components of robots

xy =[x*100,y*100];
for ix = 1:size(xy,2)
    for iy = 1:size(xy,1)
        if xy(iy,ix)<1
            xy(iy,ix)=1;
        elseif xy(iy,ix)>100
            xy(iy,ix)=100;
        end
            
    end
end

concs = spill.sample_concentration(round(xy));
max_conc = max(concs);

if max_conc < spill.conc_thresh

    dx = zeros(N,1);
    dy = zeros(N,1);


    for countRobots = 1:N
        xDistance = goToX-x(countRobots);
        yDistance = goToY-y(countRobots);
        dx(countRobots) = (vx(countRobots)+(scalar*xDistance));         % dx/dt for all robots
        dy(countRobots) = (vy(countRobots)+(scalar*yDistance));         % dy/dt for all robots
    end


    dvx = zeros(N,1);  % dvx/dt for all robots
    dvy = zeros(N,1);  % dvy/dt for all robots

    % Define the distances rij between each pair of neighboring agents i, j
    %    (agents i and j are connected by an edge in the graph)
    % Here, m and n are neighbors of robot 1 (substitute the indices of its 
    %    actual neighbors)

    %m=2 n=3
    %y: 1 2 3 4  5  6
    %    7 8 9 10 11 12
    distanceDivider = 6;
    %neigborDivider = 100;

    r12 = (sqrt(((x(1)-x(2))^2) + ((y(1)-y(2))^2)))*distanceDivider;
    r13 = (sqrt(((x(1)-x(3))^2) + ((y(1)-y(3))^2)))*distanceDivider;

    r24 = (sqrt(((x(2)-x(4))^2) + ((y(2)-y(4))^2)))*distanceDivider;
    r21 = (sqrt(((x(2)-x(1))^2) + ((y(2)-y(1))^2)))*distanceDivider;

    r46 = (sqrt(((x(4)-x(6))^2) + ((y(4)-y(6))^2)))*distanceDivider;
    r45 = (sqrt(((x(4)-x(5))^2) + ((y(4)-y(5))^2)))*distanceDivider;
    r42 = (sqrt(((x(4)-x(2))^2) + ((y(4)-y(2))^2)))*distanceDivider;

    r64 = (sqrt(((x(6)-x(4))^2) + ((y(6)-y(4))^2)))*distanceDivider;
    r65 = (sqrt(((x(6)-x(5))^2) + ((y(6)-y(5))^2)))*distanceDivider;

    r53 = (sqrt(((x(5)-x(3))^2) + ((y(5)-y(3))^2)))*distanceDivider;
    r54 = (sqrt(((x(5)-x(4))^2) + ((y(5)-y(4))^2)))*distanceDivider;
    r56 = (sqrt(((x(5)-x(6))^2) + ((y(5)-y(6))^2)))*distanceDivider;

    r31 = (sqrt(((x(3)-x(1))^2) + ((y(3)-y(1))^2)))*distanceDivider;
    r35 = (sqrt(((x(3)-x(5))^2) + ((y(3)-y(5))^2)))*distanceDivider;


%     r12 = (sqrt(((x(1)-x(2))^2) + ((y(1)-y(2))^2)));
%     r13 = (sqrt(((x(1)-x(3))^2) + ((y(1)-y(3))^2)));
% 
%     r24 = (sqrt(((x(2)-x(4))^2) + ((y(2)-y(4))^2)));
%     r21 = (sqrt(((x(2)-x(1))^2) + ((y(2)-y(1))^2)));
% 
%     r46 = (sqrt(((x(4)-x(6))^2) + ((y(4)-y(6))^2)));
%     r45 = (sqrt(((x(4)-x(5))^2) + ((y(4)-y(5))^2)));
%     r42 = (sqrt(((x(4)-x(2))^2) + ((y(4)-y(2))^2)));
% 
%     r64 = (sqrt(((x(6)-x(4))^2) + ((y(6)-y(4))^2)));
%     r65 = (sqrt(((x(6)-x(5))^2) + ((y(6)-y(5))^2)));
% 
%     r53 = (sqrt(((x(5)-x(3))^2) + ((y(5)-y(3))^2)));
%     r54 = (sqrt(((x(5)-x(4))^2) + ((y(5)-y(4))^2)));
%     r56 = (sqrt(((x(5)-x(6))^2) + ((y(5)-y(6))^2)));
% 
%     r31 = (sqrt(((x(3)-x(1))^2) + ((y(3)-y(1))^2)));
%     r35 = (sqrt(((x(3)-x(5))^2) + ((y(3)-y(5))^2)));

    % Define dvx/dt, dvy/dt for each robot
    % V = potential function = ln^2(rij) + 1/rij
    % dV/drij = derivative of V with respect to rij. This is an expression in terms of rij.

    dV_dr12 = (2*(r12*log(r12)-1)/(r12^2)); % The expression dV/drij, with rij set to r1m
    dV_dr13 = (2*(r13*log(r13)-1)/(r13^2)); % The expression dV/drij, with rij set to r1n
    dV_dr24 = (2*(r24*log(r24)-1)/(r24^2));
    dV_dr21 = (2*(r21*log(r21)-1)/(r21^2));
    dV_dr46 = (2*(r46*log(r46)-1)/(r46^2));
    dV_dr45 = (2*(r45*log(r45)-1)/(r45^2));
    dV_dr42 = (2*(r42*log(r42)-1)/(r42^2));
    dV_dr64 = (2*(r64*log(r64)-1)/(r64^2));
    dV_dr65 = (2*(r65*log(r65)-1)/(r65^2));
    dV_dr53 = (2*(r53*log(r53)-1)/(r53^2));
    dV_dr54 = (2*(r54*log(r54)-1)/(r54^2));
    dV_dr56 = (2*(r56*log(r56)-1)/(r56^2));
    dV_dr31 = (2*(r31*log(r31)-1)/(r31^2));
    dV_dr35 = (2*(r35*log(r35)-1)/(r35^2));



    dvx(1) = - (vx(1)-vx(2)) - (vx(1)-vx(3)) ... % velocity alignment terms: x components        
        - dV_dr12 * (1/r12)*(x(1)-x(2)) ... % cohesion terms: x components         
        - dV_dr13 * (1/r13)*(x(1)-x(3));
    dvy(1) = - (vy(1)-vy(2)) - (vy(1)-vy(3)) ... % velocity alignment terms: y components         
        - dV_dr12 * (1/r12)*(y(1)-y(2)) ... % cohesion terms: y components         
        - dV_dr13 * (1/r13)*(y(1)-y(3));

    dvx(2) = - (vx(2)-vx(1)) - (vx(2)-vx(4)) ... % velocity alignment terms: x components        
        - dV_dr21 * (1/r21)*(x(2)-x(1)) ... % cohesion terms: x components         
        - dV_dr24 * (1/r24)*(x(2)-x(4));
    dvy(2) = - (vy(2)-vy(1)) - (vy(2)-vy(4)) ... % velocity alignment terms: x components        
        - dV_dr21 * (1/r21)*(y(2)-y(1)) ... % cohesion terms: x components         
        - dV_dr24 * (1/r24)*(y(2)-y(4));

    dvx(3) = - (vx(3)-vx(1)) - (vx(3)-vx(5)) ... % velocity alignment terms: x components        
        - dV_dr31 * (1/r31)*(x(3)-x(1)) ... % cohesion terms: x components         
        - dV_dr35 * (1/r35)*(x(3)-x(5));
    dvy(3) = - (vy(3)-vy(1)) - (vy(3)-vy(5)) ... % velocity alignment terms: x components        
        - dV_dr31 * (1/r31)*(y(3)-y(1)) ... % cohesion terms: x components         
        - dV_dr35 * (1/r35)*(y(3)-y(5));

    dvx(4) = - (vx(4)-vx(2)) - (vx(4)-vx(5)) - (vx(4)-vx(6)) ... % velocity alignment terms: x components        
        - dV_dr42 * (1/r42)*(x(4)-x(2)) ... % cohesion terms: x components         
        - dV_dr45 * (1/r45)*(x(4)-x(5)) ...
        - dV_dr46 * (1/r46)*(x(4)-x(6));
    dvy(4) = - (vy(4)-vy(2)) - (vy(4)-vy(5)) - (vy(4)-vy(6)) ... % velocity alignment terms: x components        
        - dV_dr42 * (1/r42)*(y(4)-y(2)) ... % cohesion terms: x components         
        - dV_dr45 * (1/r45)*(y(4)-y(5)) ...
        - dV_dr46 * (1/r46)*(y(4)-y(6));

    dvx(5) = - (vx(5)-vx(4)) - (vx(5)-vx(3)) - (vx(5)-vx(6)) ... % velocity alignment terms: x components        
        - dV_dr54 * (1/r54)*(x(5)-x(4)) ... % cohesion terms: x components         
        - dV_dr53 * (1/r53)*(x(5)-x(3)) ...
        - dV_dr56 * (1/r56)*(x(5)-x(6));
    dvy(5) = - (vy(5)-vy(4)) - (vy(5)-vy(3)) - (vy(5)-vy(6)) ... % velocity alignment terms: x components        
        - dV_dr54 * (1/r54)*(y(5)-y(4)) ... % cohesion terms: x components         
        - dV_dr53 * (1/r53)*(y(5)-y(3)) ...
        - dV_dr56 * (1/r56)*(y(5)-y(6));

    dvx(6) = - (vx(6)-vx(4)) - (vx(6)-vx(5)) ... % velocity alignment terms: x components        
        - dV_dr64 * (1/r64)*(x(6)-x(4)) ... % cohesion terms: x components         
        - dV_dr65 * (1/r65)*(x(6)-x(5));
    dvy(6) = - (vy(6)-vy(4)) - (vy(6)-vy(5)) ... % velocity alignment terms: x components        
        - dV_dr64 * (1/r64)*(y(6)-y(4)) ... % cohesion terms: x components         
        - dV_dr65 * (1/r65)*(y(6)-y(5));

    dz = [(dx); (dy); (dvx); (dvy)];%*distanceDivider;
else
    dx = zeros(N,1);
    dy = zeros(N,1);
    dvx = zeros(N,1);  % dvx/dt for all robots
    dvy = zeros(N,1);  % dvy/dt for all robots
    dz = [(dx); (dy); (dvx); (dvy)];
    
end


end

function dist_ij = distance_to_neighors(i,all_agents)
    [agent_i,agent_j] = seperate_agent_i(i,all_agents);
    dist_ij = [];
    for j = 1:size(agent_j,1)
        xi = agent_i(1);
        yi=agent_i(2);
        xj = agent_j(j,1);
        yj =agent_j(j,2);
        dist_ij = [dist_ij  norm([xj-xi,yj-yi])];
    end
end
function [agent_i, agent_j] = seperate_agent_i(i,all_agents)
agent_i = all_agents(i,:);
agent_j = all_agents;
% Remove agent_i from list of all agent locations
agent_j(i,:) = [];
end
