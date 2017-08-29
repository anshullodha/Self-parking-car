%-------------------------- Parking Problem -----------------------%
%--------------------------------------------------------------------%
clear all
close all
clc

xr_0 = 8.5;
xr_f = 7.5;
yr_0 = 41.5;
yr_f = 41.5;
th_0 = pi/2;
th_f = 0;
phi_0=0;
phi_f=0;
vel_0=0;
vel_f=0;

% Auxillary data:
%-------------------------------------------------------------------%
%-------------------- Data Required by Problem ---------------------%
%-------------------------------------------------------------------%
% Some model related constants
gamma = .0001;
auxdata.gamma = gamma ;

%-------------------------------------------------------------------%
%----------------------------- Bounds ------------------------------%
%-------------------------------------------------------------------%

%-------------------------------------------------------------------%
t0  = 0;
tf  = 8;

xMin = [-100 -100 -6.28 -0.785398 -12];       %minimum of coordinates
xMax = [100 100 2*3.14 0.785398 12];       %maximum of coordinates

uMin = [-12 -0.785398]; %minimum of torques
uMax = [12 0.785398]; %maximum of torques

% setting up bounds
bounds.phase.initialtime.lower  = 0;
bounds.phase.initialtime.upper  = 0;
bounds.phase.finaltime.lower    = 0.05;
bounds.phase.finaltime.upper    = tf;
bounds.phase.initialstate.lower = [xr_0 yr_0 th_0 phi_0 vel_0];
bounds.phase.initialstate.upper = [xr_0 yr_0 th_0 phi_0 vel_0];
bounds.phase.state.lower        = xMin;
bounds.phase.state.upper        = xMax;
bounds.phase.finalstate.lower   = [xr_f yr_f th_f phi_f vel_f];
bounds.phase.finalstate.upper   = [xr_f yr_f th_f phi_f vel_f];
bounds.phase.control.lower      = uMin; 
bounds.phase.control.upper      = uMax; 
bounds.phase.integral.lower     = 0;
bounds.phase.integral.upper     = 10000;


%-------------------------------------------------------------------%
%--------------------------- Initial Guess -------------------------%
%-------------------------------------------------------------------%
rng(0);

xrGuess = [xr_0;xr_f]; 
yrGuess = [yr_0;yr_f]; 
thGuess = [th_0;th_f];
phiGuess = [phi_0; phi_f];
velGuess = [vel_0; vel_f];
u1Guess = [1;0];
u2Guess = [0.15;0];
tGuess = [0;tf]; 

guess.phase.time  = tGuess;
guess.phase.state = [xrGuess,yrGuess,thGuess,phiGuess velGuess];
guess.phase.control        = [u1Guess, u2Guess] ;
guess.phase.integral         = 0;

%-------------------------------------------------------------------%
%--------------------------- Problem Setup -------------------------%
%-------------------------------------------------------------------%
setup.name                        = 'Parking-Problem';
setup.functions.continuous        = @CLMR_continuous;
setup.functions.endpoint          = @CLMREndpoint;
setup.bounds                      = bounds;
setup.auxdata                     = auxdata;
setup.functions.report            = @report;
setup.guess                       = guess;
setup.nlp.solver                  = 'ipopt';
setup.derivatives.supplier        = 'sparseCD';
setup.derivatives.derivativelevel = 'first';
setup.scales.method               = 'none';
setup.derivatives.dependencies    = 'full';
setup.mesh.method                 = 'hp-PattersonRao';
setup.mesh.tolerance              = 0.01;
setup.method                      = 'RPM-Integration';

%-------------------------------------------------------------------%
%------------------- Solve Problem Using GPOPS2 --------------------%
%-------------------------------------------------------------------%
output = gpops2(setup);
output.result.nlptime
solution = output.result.solution;

save solution.mat
plotGPOPS;