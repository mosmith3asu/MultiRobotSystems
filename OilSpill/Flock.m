% close all
% clc
% clear all
function [final_loc,final_vel]=Flock()
    %[value,index] = max(map)
    numRobots = 6;
    N=numRobots;
    global goToX;
    global goToY;
    global spill;
    %spill = Enviornment
    figure
    hold on
%     %%%%%%%%%%%%%%%%%%%%%%%%%
% 

% 
% 
%     disp("Initializing Spill")
%     n_initial = 30;
%     spill = Enviorment;
%     init_env = spill.initialize(n_initial);
%     [val,loc]= spill.max_conc();
%     goToX = loc(1)/100;
%     goToY = loc(2)/100;
%     disp("Advancing Timestep")
%     advance_n_steps = 1;
%     spill.step(advance_n_steps);
    map = spill.current_map();

    disp("Plotting")
    pcolor(spill.x_grid,spill.y_grid,spill.env);
    shading interp
    colorbar
    colormap(jet)


%     %%%%%%%%%%%%%%%%%%%%%%%%%%%
    xrange=0.3;
    yrange = 0.2;
    vx0 = zeros(numRobots,1);
    vy0 = zeros(numRobots,1);

    %robot initial location
    %x0 = xrange*(rand(numRobots,1));  % Initial x coordinates of robots
    y0 = yrange*(rand(numRobots,1));  % Initial y coordinates of robots
    x0= linspace(0.05,xrange,numRobots)';
    %y0=linspace(0.05,yrange,numRobots)'
    %Sets the inital Velocity of all robots
    for countRobots = 1:numRobots 
        velX = 0.2*(rand(1,1));
        velY = 0.2*(rand(1,1));
    %     velX = 0.1;
    %     velY = 0.1;
        %sets initial velocity
        vx0(countRobots) = velX;
        vy0(countRobots) = velY;

    end

    z0 = [x0; y0; vx0; vy0];  % Initial state

    [t,z] = ode45(@Final_FlockingODE,[0 100],z0);

    % Trim Stop Condition
    z = unique(z, 'rows');
    s=size(z,1);
    t= t(1:s,:);

    x = z(s,1:N)';    % x coordinates of robots
    y = z(s,1*N+1:2*N)';   % y coordinates of robots
    vx = z(s,2*N+1:3*N)';   % vx velocity components of robots
    vy = z(s,3*N+1:4*N)'; % vy velocity components of robots

    final_loc = [x,y];
    final_vel = [vx,vy];

    set(gca,'Fontsize',20);
    grid on

    for countRobots = 1:numRobots
        plot(z(:,countRobots),z(:,countRobots+numRobots),'-','Linewidth',2)
        plot(z(1,countRobots),z(1,countRobots+numRobots),'k*','Markersize',10,'Linewidth',2)  % initial position of robot 1
    end
    plot(goToX,goToY,'b*','Markersize',30,'Linewidth',2)  % Where we want to be

    %legend('Robot 1','Robot 2','Robot 3','Robot 4','Robot 5','Robot 6');
    xlabel('x position')
    ylabel('y position')
    title('Robot paths over 100 s, where * is the location of the spill')

    % 
    % %Figure 2 of distance versus time
    % 
    figure
    hold on
    set(gca,'Fontsize',20);
    grid on

    %y: 1 2 3 4  5  6
    %x  7 8 9 10 11 12
    plot(t,(sqrt(((z(:,1)-z(:,2)).^2) + ((z(:,7)-z(:,8)).^2))),'k','Linewidth',2)  
    plot(t,(sqrt(((z(:,1)-z(:,3)).^2) + ((z(:,7)-z(:,9)).^2))),'b','Linewidth',2) 
    plot(t,(sqrt(((z(:,3)-z(:,5)).^2) + ((z(:,9)-z(:,11)).^2))),'r','Linewidth',2)  
    plot(t,(sqrt(((z(:,5)-z(:,6)).^2) + ((z(:,11)-z(:,12)).^2))),'g','Linewidth',2) 
    plot(t,(sqrt(((z(:,5)-z(:,4)).^2) + ((z(:,11)-z(:,10)).^2))),'y','Linewidth',2)  
    plot(t,(sqrt(((z(:,4)-z(:,6)).^2) + ((z(:,10)-z(:,12)).^2))),'c','Linewidth',2) 
    plot(t,(sqrt(((z(:,4)-z(:,2)).^2) + ((z(:,10)-z(:,8)).^2))),'k','Linewidth',2) 

    legend('Robot12','Robot13','Robot35','Robot56','Robot54','Robot46','Robot42');
    xlabel('Time')
    ylabel('Distance')
    title('Distance Between Robots')
end
