% clear all
% clc
% close all
% hold off
function Pot(agents_loc,agents_vel,T)
    global spill;
    global f;
    global N_agents;
    agents_loc=agents_loc*100;
    %agents_acc = 0.01*ones(size(agents_loc));
    agents_acc = zeros(size(agents_loc));
    map = spill.current_map();
    pot_vec =f.agent_potentials(agents_loc,map); % get current potentials for each agent

    %% Plot initial enviorment
    disp("Plotting Initial Env")
    f.plot_agents_spill(agents_loc,spill,'Initial Spill and Agents')

    %% Iterate through time
    num_contour=3;
    level = spill.conc_thresh;
    concentrations_t = [];
    nearest_neighors = [];
    location_tracker_x = [];
    location_tracker_y = [];
    location_tracker_x2 = [];
    location_tracker_y2 = [];
    it_tracker = [];
    figure('Name',"Contour and Agent Travel")
    hold on

    capture_n =20;
    capture = capture_n;
    for it=1:T
        capture = capture +1;
        ITERATION=it
        % update spill %%%%%%%%%%%%%%%%%%%
        spill.step(1);
        map = spill.current_map();

        % Calculate potentials and move robots %%%%%%%%%%%%%%%%
        pot_vec =f.agent_potentials(agents_loc,map); % get current potentials for each agent
        [agents_loc,agents_vel,agents_acc]= f.update_locations(pot_vec,agents_loc,agents_vel,agents_acc);
        x_v_a = [agents_loc,agents_vel,agents_acc]

        % Analysis %%%%%%%%%%%%%%%%%%%
        if capture>=capture_n
            capture=0;
            % Concentrations of each agent
            %concentrations_t = [concentrations_t; f.get_concs(agents_loc,map)];
            current_conc = spill.sample_concentration(agents_loc)
            concentrations_t = [concentrations_t current_conc];
            nearest2 = [];
            for i = 1:N_agents
                 nearest2 = [nearest2,f.distance_to_neighors(i,agents_loc)];
            end

            % Nearest Neighors
            nearest_neighors =[nearest_neighors; nearest2];
            location_tracker_x = [location_tracker_x, agents_loc(:,1)/100];
            location_tracker_y = [location_tracker_y, agents_loc(:,2)/100];
        end

        %Contour over time
        if rem(it,round(T/num_contour))==0 || it==1
        %if mod(it,round(T/3))==0 
            f.plot_mapcontour(spill,it);

            location_tracker_x2 = [location_tracker_x2, agents_loc(:,1)/100];
            location_tracker_y2 = [location_tracker_y2, agents_loc(:,2)/100];
            it_tracker = [it_tracker;it];

        end

    end

    % plot locations of robots on contour map
    f.plot_mapcontour_agents(N_agents,it_tracker,location_tracker_x,location_tracker_y,location_tracker_x2,location_tracker_y2)
    Agent_Locations = agents_loc
%    dloc = agents_loc - init_loc % Display change in location of all agents
    hold off

    %% Plot Map after iteration
    % plot oil spill
    disp("Plotting Initial Env")
    f.plot_agents_spill(agents_loc,spill,'Spill and Agents After Iteration')

    %% Diffusion Estimation
    % figure('Name',"Estimated Map")
    % est_map = diff_est.map(agents_loc,spill);
    % 
    % subplot(1,2,1)
    % surf(spill.x_grid,spill.y_grid,map)
    % title('Actual Diffusion Map')
    % zlabel('Oil Concentration')
    % 
    % subplot(1,2,2)
    % surf(spill.x_grid,spill.y_grid,est_map)
    % title(' Estimated Diffusion Map')
    % zlabel('Oil Concentration')

    %% Analysis Plots
    % Concentrations
    figure
    plot(concentrations_t')
    title('Concentrations of Measured by Each Agent')
    xlabel('Time Step') 
    ylabel('Concentration') 

    % Nearest Neighors
    figure
    plot(nearest_neighors)
    title('Distance to Neighbors')
    xlabel('Time Step') 
    ylabel('Distance') 

end



















