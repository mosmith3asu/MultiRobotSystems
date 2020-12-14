%% Initialize Enviorment
% spill = Enviorment; % Define object
% spill.initialize(5); % Initialize Spill at t=0 (intial_iterations)

%% Callable functions
% spill.step(1); % advance spill (n=1) timesteps
% spill.step(2); % advance spill (n=2) timesteps
% spill.current_map(); % returns current spill map (same as spill.env)

%% Plotting spill colormap
% spill = pcolor(spill.x_grid,spill.y_grid,spill.env);
% shading interp
% colorbar
% colormap(jet)

classdef SimpleSpillEstimator < handle
    properties
        est_map=[];
        data = []; %[locations,concentrations]
        memory_size = 3;
    end
    methods
        function est_map = map(obj,new_agent_locations,spill)
            pps = 3; % pins per side
%             pin_corners = [
%                 1,1,0;
%                 1,100,0;
%                 100,1,0;
%                 100,100,0;
%                 50,1,0;
%                 1,50,0;
%                 50,100,0;
%                 100,50,0];
            N_agents = size(new_agent_locations,1);
            obj.est_map=zeros(spill.size_x,spill.size_x);   
            new_conc =spill.sample_concentration(new_agent_locations);
            
            n_in_range = 0;
            for i = 1:N_agents
                if new_conc(i)>0.25*spill.conc_thresh
                    n_in_range=n_in_range+1;
                end
            end
            
            % Able to generate plot
            if n_in_range>0.5*N_agents
          
                top = [linspace(1,100,pps)',ones(pps,1),zeros(pps,1)];
                bottom = [linspace(1,100,pps)',100*ones(pps,1),zeros(pps,1)];
                right = [ones(pps,1),linspace(1,100,pps)',zeros(pps,1)];
                left = [ones(pps,1),100*linspace(1,100,pps)',zeros(pps,1)];
                pin_corners = unique([top;bottom;right;left],'rows');

                if size(obj.data,1)>= obj.memory_size* (N_agents+1)
                    start = size(obj.data,1) - obj.memory_size*N_agents;
                    obj.data = obj.data(start:size(obj.data,1),:);
                end

                obj.data = unique([obj.data; [new_agent_locations,new_conc]],'rows');
                
                sf = fit([obj.data(:,1:2);pin_corners(:,1:2)],[obj.data(:,3);pin_corners(:,3)],'poly44');
    %             if size(obj.data,1)>=15
    %                 sf = fit(obj.data(:,1:2),obj.data(:,3),'poly44');
    %             else
    %                 sf = fit(obj.data(:,1:2),obj.data(:,3),'poly22');
    %             end

                for x=1:spill.size_x
                    for y=1:spill.size_y
                        val = feval(sf,[x,y]);
                        if val<=0
                            val=0;
                        elseif val>=spill.pt_amp
                            val=spill.pt_amp;
                        end
                        est_map(y,x)= val;
                        obj.est_map(y,x)= val;
                    end
                end
                %feval(sf,agent_locations(1,:));

                %plot(sf,agent_locations,conc)
            else
                for x=1:spill.size_x
                    for y=1:spill.size_y       
                        est_map(y,x)= 0;
                        obj.est_map(y,x)= 0;
                    end
                end
            end
            
        end

        
        %% Advance enviorment N timesteps
        function result = step(obj,n_steps)
            for it = 1:n_steps
                x_size = size(obj.env,1);
                y_size = size(obj.env,2);
                for x_loc = 1:x_size
                    for y_loc = 1:y_size
                        if obj.env(x_loc,y_loc) >0.0001
                            conc =  obj.env(x_loc,y_loc);

                            if ((1<x_loc)&&(x_loc<x_size)) && ((1<y_loc)&&(y_loc<y_size))
                                obj.env(x_loc-1,y_loc)= obj.env(x_loc-1,y_loc)+conc* obj.diff_rate;
                                obj.env(x_loc+1,y_loc)= obj.env(x_loc+1,y_loc)+conc* obj.diff_rate;
                                obj.env(x_loc,y_loc-1)= obj.env(x_loc,y_loc-1)+conc* obj.diff_rate;
                                obj.env(x_loc,y_loc+1)= obj.env(x_loc,y_loc+1)+conc* obj.diff_rate;
                                obj.env(x_loc,y_loc) = conc - 4*conc* obj.diff_rate;
                            else
                                obj.env(x_loc,y_loc) = conc - 4*conc* obj.diff_rate;
                            end
                            
                        end
                    end
                end
            end
            result = obj.env;
        end
        %% Return current map
        function result = current_map(obj)
            result = obj.env;
        end
        function concs = sample_concentration(obj,agent_loc)
            concs = []; % List of concentrations
            for i = 1:size(agent_loc,1)
                x_loc=int16(agent_loc(i,1));
                y_loc=int16(agent_loc(i,2));
                conc = obj.env(y_loc,x_loc);
                concs = [concs; conc];
            end
        end
    end
end