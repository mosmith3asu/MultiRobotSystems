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

classdef Enviorment < handle
    properties
        pt_amp = 10; % amplitude of point diffusion
        conc_thresh = 0.05; % perimeter definition
        diff_rate = 0.1;
        %pot = pt_amp;
        size_x = 100;
        size_y = 100;
        env = [];
        x_grid = [];
        y_grid = [];
        spill_centers = [];
        %Value {mustBeNumeric}
    end
    methods
        function result = initialize(obj,initial_steps)
            % Place oil amplitude at spill center location
            obj.env = zeros(obj.size_x,obj.size_y);
           
            obj.x_grid = linspace(0,1,obj.size_x);
            obj.y_grid = linspace(0,1,obj.size_y);
            obj.spill_centers = [
                50,50;
                45,45;
                55,55;
                50,55;
                55,50;
                50,45];
            for i = 1:size(obj.spill_centers,1)
                x = obj.spill_centers(i,1);
                y = obj.spill_centers(i,2);
                obj.env(x,y)=obj.pt_amp;
            end
            
            %% Initialize Enviorment at N_Steps
            for it = 1:initial_steps
                for x_loc = 1:size(obj.env,1)
                    for y_loc = 1:size( obj.env,2)
                        if obj.env(x_loc,y_loc) >0.0001
                            conc =  obj.env(x_loc,y_loc);
                             obj.env(x_loc-1,y_loc)= obj.env(x_loc-1,y_loc)+conc* obj.diff_rate;
                             obj.env(x_loc+1,y_loc)= obj.env(x_loc+1,y_loc)+conc* obj.diff_rate;
                             obj.env(x_loc,y_loc-1)= obj.env(x_loc,y_loc-1)+conc* obj.diff_rate;
                             obj.env(x_loc,y_loc+1)= obj.env(x_loc,y_loc+1)+conc* obj.diff_rate;
                             obj.env(x_loc,y_loc) = conc - 4*conc* obj.diff_rate;
                        end
                    end
                end
            end
            %disp(obj.env)
            result = [obj.env];
            
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
                x_loc=round(agent_loc(i,1));
                y_loc=round(agent_loc(i,2));
                conc = obj.env(y_loc,x_loc);
                concs = [concs; conc];
            end
        end
        
        function [value,index] = max_conc(obj)
            value = obj.env(1,1);
            index = [1,1];
            for y = 1:obj.size_y
                for x=1:obj.size_x
                    if obj.env(y,x)>value
                        value = obj.env(y,x);
                        index = [x,y];
                    end
                end
            end
            
        end
    end
end