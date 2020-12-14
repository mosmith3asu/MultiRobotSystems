clear all
clc
close all
hold off
%% Global Vars
global spill;
global f;
global goToX;
global goToY;
global N_agents;
%% Initialize Function Handler
f = Functions; % function handler
spill = Enviorment; % enviorment object

%% Enviorment Parameters
conc_thresh = 0.15; % perimeter definition
diffusion_rate = 0.05;

n_initial = 50; % number of spill initialization iteration
T=2000;          % number of live spill iterations

plot_est = 0; % enable estimation of map

attractive_amp = 50;
attractive_pot_coeff = 0.5;
repulsive_pot_coeff = 0.5;
repulsive_amp = 50;

N_spills = 3;

f.conc_thresh = conc_thresh;
spill.conc_thresh = conc_thresh;

map_color = 'k';
est_map_color = 'red';

spill_centers=f.randrange_loc(N_spills,50,53)%round((max_loc-min_loc).*rand(N_spills,2) + min_loc)

%% Agent Parameters
N_agents = 6;

% agents_loc=f.randrange_loc(N_agents,10,30)
% first_to_find = [40,40];
% agents_loc = [agents_loc; first_to_find];
% agents_vel = 0.1*ones(size(agents_loc));
%agents_acc = 0.01*ones(size(agents_loc));


%% Set up Enviorment
% Description of first code block
%diff_est = SimpleSpillEstimator;
% 
% N_agents = size(agents_loc,1);
% init_loc = agents_loc;
disp("Initializing Spill")

spill.pt_amp = attractive_amp;
spill.diff_rate = diffusion_rate;
spill.spill_centers = spill_centers;
      
init_env = spill.initialize(n_initial);
map = spill.current_map();

[val,loc]= spill.max_conc();
goToX = loc(1)/100;
goToY = loc(2)/100;
%% Begin Flocking
disp('Begin Flocking...')
[agents_loc,agents_vel]=Flock()
% agents_loc = [
%     30,30;
%     70,70;
%      50,60;
%      60,40;
%      38,47;
%      20,20]/100
disp('Begin Potential Field Control...')
Pot(agents_loc,agents_vel,T)


