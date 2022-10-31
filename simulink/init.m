clear all
clear functions
clc

set(0, 'defaultfigurewindowstyle', 'docked'); 


%% Paths
addpath('initScripts')
addpath(fullfile('models'));

%% GLobal Values
g = 9.806;

%% Vehicle Ridgid body data

vehicle = initVehicle();

%% Atmosphere
atmosphere = initAtmos();

%% Propeller
propeller = initPropeller();

%% Propulsion
propulsion = initPropulsion();

%% Sensors
sensors = initSensors(vehicle.pos0(3));

%% Controller
controller = initController(vehicle, propeller);

%% Buses
load('bus_definition');
%initBusDef(size(propeller.positions,1));
rosshutdown
rosinit

