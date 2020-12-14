function [Ni,T] = GillespiesDirectMethod(V,N,x0,Tf)
%V is stoichiometric matrix
%N is population of simulation
%x0 is the initial population proportions
%Tf is the final time
%c is a 1xN array of reaction rate constants

clear Xs;
clear T;
clc;

% Define Reaction Rate Constants
a1 =2.0; a2 = 4.0;
b1=1.2; b2=0.7;
g1=0.5; g2=1.0;
k1=1.5; k2=0.4;
u1=0.7; u2=1.2;
e1=0.9; e2=0.4;
n1 =1.1; n2=0.6;


%V = [-1 1 0; -1 1 1; 1 -1 -1; 0 0 1];
%%%%%%%%%% Parameters andInitial Conditions %%%%%%%%%
%nA = 6.023e23; % Avagadro’s number
%vol = 1e-15; % volume of system
%X = zeros(4,1);
Nspecies = length(x0);
X = zeros(Nspecies,1);
Ni = zeros(Nspecies,1);;    %Set of integer populations in each state
T = [];     %Time intervals

% initialize starting populations
for i = 1:size(Nspecies)
    X(i) = x0(i)*N;
    

t = 0;
tfinal = Tf;
while t < tfinal
    a(1) = a1*X(1)*X(2);
    a(2) = b1*X(2)*X(4);
    a(3) = a2*X(1)*X(3);
    a(4) = b2*X(5)*X(3);
    a(5) = k1*X(2);
    a(6) = g1*X(4);
    a(7) = k2*X(3);
    a(8) = g2*X(5);
    a(9) = e1*X(7)*X(2);
    a(10) = e2*X(6)*X(3);
    a(11) = n1*X(7)*X(2);
    a(12) = u1*X(6)*X(7);
    a(13) = u2*X(6)*X(7);
    a(14) = n2*X(6)*X(3);

    asum = sum(a);
    j = min(find(rand<cumsum(a/asum)));
    tau = log(1/rand)/asum;
    %X=X+ V(:,j);
    X=X(:,1)+ V(:,j);
    t=t+ tau;
    T = [T t];
    Ni = [Ni X]
    % Recordor plot system state here if required
end
plot(Ni(1,:),T,Ni(2,:),T)
end