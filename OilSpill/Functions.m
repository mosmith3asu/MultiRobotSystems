classdef Functions < handle
    properties
        conc_thresh = 0.05; % perimeter definition
    end
    methods
        %% Cartesian potentials
        function [xy,dxy,d2xy]=update_locations(obj,pot_forces,agent_loc,agent_vel,agent_acc)
            max_vel = 0.5;
            max_acc = 5;
            
            d2xy = pot_forces;
            
            dxy = agent_vel + d2xy;
            dxy = obj.hard_cap(dxy,max_vel);
            
            xy = agent_loc + dxy;
             
            xy= obj.check_index100(xy);
    
        end
    
        function  F = repulsive_pot(obj,all_agents_loc)
            %Mobile-Sensor-Network-Deployment-using-Potential-Fields-2002.pdf
            k=1;
            F=[];
            for i = 1:size(all_agents_loc,1)
                [agent_i,agent_j] = obj.seperate_agent_i(i,all_agents_loc);
                F0 = [];
                for j =1:size(agent_j,1)
                    ri_vec = [agent_j(j,1)-agent_i(1), agent_j(j,2)-agent_i(2)];
                    ri_norm = norm(ri_vec);
                    F0 = [F0;-k*1/ri_norm^2*(ri_vec/ri_norm)];
                end
                F = [F;sum(F0)];
            end
        end
        
        function agent_pot_vec = attractive_pot(obj,agent_loc,spill_model,conc_thresh)
            %conc_thresh = 0.1;
            scale = 1;
            size_x = size(spill_model, 2);
            size_y = size(spill_model, 1);
            x_grid = linspace(0,1,size_x);
            y_grid = linspace(0,1,size_y);
            
            pot_field = zeros(size_y,size_x);
            for x_loc = 1:size(spill_model,1)
                for y_loc = 1:size(spill_model,2)
                    pot_field(y_loc,x_loc) = abs(conc_thresh - spill_model(y_loc,x_loc));
                end
            end
            
            [px,py] = gradient(-1*pot_field);
            agent_pot_vec = [];
            for i = 1:size(agent_loc,1)
                x_loc = round(agent_loc(i,1));
                y_loc = round(agent_loc(i,2));
                agent_pot_vec = [agent_pot_vec; px(y_loc,x_loc),py(y_loc,x_loc)];
            end
            agent_pot_vec = scale*agent_pot_vec;
            %figure
            %zmin = 2*min(pot_field(:));
            %     Z=pot_field
            %     zmin = floor(min(Z(:)))
            %     zmax = ceil(max(Z(:)));
            %     zinc = (zmax - zmin) / 3;
            %     zlevs = zmin:zinc:zmax;
            %     level = 0.02;
            %     contour(x_grid,y_grid,pot_field,[level,level]);
            %     hold on
            %     quiver(x_grid,y_grid,px,py)
            %     hold off
            
        end
        
        function F=neighoring_conc_pot(obj,agents_loc,map)
            % Case for platue effect then flock to root with highest concentration
            thresh = 1e-4;
            scale= 1;
            F=[];
            for i = 1:size(agents_loc,1)
                [agent_i,agent_j] = obj.seperate_agent_i(i,agents_loc);
                xi = round(agent_i(1));
                yi=round(agent_i(2));
                
                % check if there is no significant attractive potential for agent i
                if map(yi,xi)<thresh
                    
                    % initialize maximum concentration variables
                    xj = round(agent_j(1,1));
                    yj =round(agent_j(1,2));
                    nearest_per_loc = [xj,yj];
                    nearest_per_conc =abs( obj.conc_thresh- map(yj,xj));
                    
                    % Check for other higher concentrations
                    for j =1:size(agent_j,1)
                        xj = round(agent_j(j,1));
                        yj =round(agent_j(j,2));
                        conc =abs(obj.conc_thresh- map(yj,xj));
                        if conc<nearest_per_conc
                            nearest_per_loc = [xj,yj];
                            nearest_per_conc = conc;
                        end
                    end
                    
                    ri_vec = [nearest_per_loc(1)-xi, nearest_per_loc(2)-yi];
                    ri_norm = norm(ri_vec);
                    
                    %d_conc = max_conc - spill_model(yi,xi);
                    %k=scale*ri_norm*d_conc;
                    k=scale*ri_norm;
                    
                    F = [F;k*1/ri_norm^2*(ri_vec/ri_norm)];
                    %end
                else
                    F = [F;[0,0]];
                end
            end
            
        end
        
        function F=agent_potentials(obj,agents_loc,map)
     
            a_scale=15;
            r_scale=5;
%             p_scale = 10;
%             a_scale=15;
%             r_scale=4;
             p_scale = 12;
             
            attractive_pot_vec =a_scale*obj.attractive_pot(agents_loc,map,obj.conc_thresh);
            cart_repulsive_pot_vec = r_scale*obj.repulsive_pot(agents_loc);
            rad_repulsive_pot_vec = r_scale*obj.radial_repulsive_pot(agents_loc);
            platue_pot_vec = p_scale*obj.neighoring_conc_pot(agents_loc,map);
            
            repulsive_pot_vec = [];
            for i = 1:size(platue_pot_vec,1)
                if norm(platue_pot_vec(i,:))>0
                    repulsive_pot_vec=[repulsive_pot_vec; rad_repulsive_pot_vec(i,:)];
                else
                    repulsive_pot_vec=[repulsive_pot_vec; cart_repulsive_pot_vec(i,:)];
                end
            end

            forces_arp = [attractive_pot_vec,repulsive_pot_vec,platue_pot_vec]
            
            F = (attractive_pot_vec + repulsive_pot_vec +platue_pot_vec);
        end
        %% Polar Potentials    
        function Fc=radial_repulsive_pot(obj,agent_loc)
            %https://search.lib.asu.edu/discovery/fulldisplay?context=PC&vid=01ASU_INST:01ASU&search_scope=MyInst_and_CI&tab=Everything&docid=cdi_crossref_primary_10_1016_j_oceaneng_2020_107238
            % agent_i is an index value
            % Generate repulsive potentials only in the radial direction
            % Equation 16
            global spill;
            [conc,Pc] = spill.max_conc();
            Pc=Pc/100;
            agent_loc=obj.polar_agent_loc(agent_loc,Pc);
            Fc=[];
            K =1; % constant positive gain
            N = size(agent_loc,1);%num agents
            
            for i=1:N
                % Remove agent_i from list of all agent locations
                [agent_i,agent_j] = obj.seperate_agent_i(i,agent_loc);
                
                pi = agent_i(1);
                theta_i = agent_i(2);
                
                pj = agent_j(:,1);
                theta_j = agent_j(:,2);
                
                dij =  obj.distance_to_other_agents(i,agent_loc);
                
                F_phi = [];
                for j = 1:N-1
                    F_phi = [F_phi; K*(pj(j)/dij(j)^3*sin(theta_j(j)-theta_i))];
                end
                F_phi = sum(F_phi); % GET NET FORCE IN RADIAL DIRECTION
                Fc=[Fc;[F_phi*sin(theta_i),F_phi*cos(theta_i)]];
            end
        end
               
        function polar_coord=polar_agent_loc(obj,agent_cartesien,Pc)
            % Polar coordinate of the agent with reference to the center Pc
            % Equation 6
            N = size(agent_cartesien,1);%num agents
            xi = agent_cartesien(:,1);
            yi = agent_cartesien(:,2);
            xc = Pc(1);
            yc = Pc(2);
            
            polar_coord = [];
            for i = 1:N
                [theta,rho] = cart2pol(xi(i)-xc,yi(i)-yc);
                polar_coord = [polar_coord; rho,theta];
                %    Pi = sqrt((xi(i)-xc)^2+(yi(i)-yc)^2);
                %    theta_i = atan((yi(i)-yc)/(xi(i)-yc));
                %
                %    polar_coord = [polar_coord;Pi,theta_i];
            end
            
            
        end 
        % Distance to neighboring boths
        function dij = distance_to_other_agents(obj,i,all_agents)
            N = size(all_agents,1);%num agents
            
            [agent_i,agent_j] = obj.seperate_agent_i(i,all_agents);
            
            pi= agent_i(1);
            theta_i= agent_i(2);
            pj= agent_j(:,1);
            theta_j= agent_j(:,2);
            
            dij = [];
            for j = 1:N-1
                dij = [dij; sqrt(pi^2+pj(j)^2-2*pi*pj(j)*cos(theta_j(j)-theta_i))];
            end
            
        end
        % Radial repulsive force to other boths
  
        %% Analysis tool distance to neighoring robots
        function [agent_i, agent_j] = seperate_agent_i(obj,i,all_agents)
            agent_i = all_agents(i,:);
            agent_j = all_agents;
            % Remove agent_i from list of all agent locations
            agent_j(i,:) = [];
        end
        
        %% Analysis tool distance to neighoring robots
        function nearest2 = distance_to_neighors(obj,i,all_agents)
            [agent_i,agent_j] = obj.seperate_agent_i(i,all_agents);
            dist = [];
            for j = 1:size(agent_j,1)
                xi = agent_i(1);
                yi=agent_i(2);
                xj = agent_j(j,1);
                yj =agent_j(j,2);
                dist = [dist norm([xj-xi,yj-yi])];
            end
            nearest2 = mink(dist,2);
        end
        
        %% Cocentrations of oil at each agent
        function concs = get_concs(obj,all_agents_loc,spill_model)
            concs =[];
            for i = 1:size(all_agents_loc,1)
                all_agents_loc(i,1)
                all_agents_loc(i,2)
                x=round(all_agents_loc(i,1));
                y=round(all_agents_loc(i,2));
                
                [x,y] = obj.check_index100(x,y);
                
                conc = spill_model(y,x);
                concs = [concs, conc];
            end
        end
        
        function loc=randrange_loc(obj,n,min_loc,max_loc)
            loc=round((max_loc-min_loc).*rand(n,2) + min_loc);
        end
        
        %% Plotting tools
        function plot_pot_field(obj)
        end
        
        function plot_agents_spill(obj,agents_loc,spill,tit)
            figure('Name',tit)
            pcolor(spill.x_grid,spill.y_grid,spill.env);
            hold on
            shading interp
            colorbar
            colormap(jet)
            
            
            %plot potential field vectors on agents
            scatter(agents_loc(:,1)/100,agents_loc(:,2)/100,'+','k')
            
            %quiver(agents_loc(:,1)/100,agents_loc(:,2)/100,attractive_pot_vec(:,1),attractive_pot_vec(:,2),'color','g')
            %quiver(agents_loc(:,1)/100,agents_loc(:,2)/100,repulsive_pot_vec(:,1),repulsive_pot_vec(:,2),'color','r')
            level = spill.conc_thresh;
            [C,h]= contour(spill.x_grid,spill.y_grid,spill.env,[level,level],'w');
            h.LineWidth = 2;
            hold off
            
        end
        
        function plot_mapcontour(obj,spill,it)
            map = spill.current_map();
            level = spill.conc_thresh;
            [C,h]= contour(spill.x_grid,spill.y_grid,map,[level,level],'k');
            clabel(C,h,'FontSize',7);
            %             if plot_est==1
            %                 est_map = diff_est.map(agents_loc,spill);
            %                 [Cest,hest]= contour(spill.x_grid,spill.y_grid,est_map,[level,level],est_map_color);
            %                 clabel(Cest,hest,'FontSize',7);
            %             end
            drawnow
            
            labels=h.TextPrims;
            
            for idx = 1 : numel(labels)
                h.TextPrims(idx).String = num2str(it);
            end
            %             if plot_est==1
            %                 est_labels=hest.TextPrims;
            %                 for idx = 1 : numel(est_labels)
            %                     hest.TextPrims(idx).String = num2str(it);
            %                 end
            %             end
            %
            title('Spill Boundry and Agent Location')
            xlabel('x')
            ylabel('y')
            %xlim([0 1])
            %ylim([0 1])
            global spill;
            [conc,Pc] = spill.max_conc()
            scatter(Pc(1)/100,Pc(2)/100,'k')
            drawnow
        end
        
        function plot_mapcontour_agents(obj,N_agents,it_tracker,location_tracker_x,location_tracker_y,location_tracker_x2,location_tracker_y2)
            dx = 0.005; dy = 0.005; % displacement so the text does not overlay the data points
            scatter_labels = cellstr(num2str(it_tracker));
            for i = 1:N_agents
                scatter(location_tracker_x2(i,:),location_tracker_y2(i,:),'+')
                
                %text(location_tracker_x2(i,:)+dx, location_tracker_y2(i,:)+dy,  scatter_labels, 'Fontsize', 5);
                
                %windowSize = 1; a=1.1;b=(1/windowSize)*ones(1,windowSize);
                %x = filter(b,a,location_tracker_x(i,:));
                %y = filter(b,a,location_tracker_y(i,:));
                %plot(x,y)
                plot(location_tracker_x(i,:),location_tracker_y(i,:),'LineWidth',0.25)
            end
            
        
            
        end
        %% Error Handling
        function xy = check_index100(obj,xy)
            check_NaN = 1;
            for i=1:size(xy,1)
                for j =1:size(xy,2)
                    if xy(i,j)<1
                        xy(i,j)=1;
                    elseif xy(i,j)>100
                        xy(i,j)=99;
                    elseif isnan(xy(i,j)) && check_NaN == 1
                        xy(i,j)=100;
                    end
                end
            end
%             % Cap x at boundry
%             if x<1
%                 x=1;
%             elseif x>100
%                 x=100;
%             elseif isnan(x) && check_NaN == 1
%                 x=100;
%             end
%             x= round(x);
%             % Cap y at boundry
%             if y<1
%                 y=1;
%             elseif y>100
%                 y=100;
%             elseif isnan(y) && check_NaN == 1
%                 y=100;
%             end
%             y=round(y);
         end
        
        function arr = hard_cap(obj,arr,cap)
            for i=1:size(arr,1)
                for j =1:size(arr,2)
                    if arr(i,j)>0
                        arr(i,j)=min(cap,arr(i,j));
                    elseif arr(i,j)<0
                        arr(i,j)=max(-cap,arr(i,j));
                    end
                    
                end
            end
            
        end
    end

end