function [t,x] = GillespiesDirectMethod(N_Species,Tf,init_conditions)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%Step 1
%Initialize the counter containing the number of robots in each “species”
%(e.g., task, role, site) at time t = 0.
%Chemical Master Equation
CME = [] 


t_span = [0 Tf]
Nx = N*x_0(1);
Ny1 = N*x_0(2);
Ny2 = N*x_0(3);
Nz1 = N*x_0(4);
Nz2 = N*x_0(5);
Nw1 = N*x_0(6);
Nw2 = N*x_0(7);

Xi = [Nx;Ny1;Ny2;Nz1,Nz2;Nw1;Nw2]

%Step 2
%Calculate the propensities ap ( p = 1,...,S ). 


%Step 3
%Choose the next reaction sp according to the distribution: 


%Step 4
%Choose T according to the distribution: 


%Step 5
%Advance time by t and effect reaction sp : Decrement the number of robots
%in the appropriate species and increment the counter at a deterministic
%time representing the end of the reaction (e.g., the robot’s 
%travel time between two sites).

%Step 6
%Whenever the counter is incremented, go to step 2.
end
function [P_react] = ReactionProbabilities(Xi,CME)





end