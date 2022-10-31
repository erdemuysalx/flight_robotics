function vehicle = initVehicle()

vehicle.sampletime = 0.0005;

% Total vehicle inertia
vehicle.J = ...
    [0.00297959015000000 1.01643953670516e-20 6.77626357803440e-21;...
    1.01643953670516e-20 0.00194640581666667 0;...
    6.77626357803440e-21 0 0.00462384373333334];
vehicle.m = 0.460;

%% Aircraft location and state init

h0 = 1;         % height above ground

locationStruct = struct();
locationStruct.name = 'Indoor PWR 27';
%locationStruct.groundHeight = 433;
locationStruct.groundHeight = 0;
locationStruct.Loc = [48.749905,9.105740];
locationStruct.Heading = 0;
locationStruct.landed = false;


vehicle.pos0(3) = locationStruct.groundHeight + h0;
vehicle.ori0(1:2) = [0,0]*pi/180;
vehicle.v0 = [0,0,0];


vehicle.omega0 = [0;0;0];
vehicle.pos0(1:2) = locationStruct.Loc;
vehicle.ori0(3) = locationStruct.Heading*pi/180;
vehicle.ground_height = locationStruct.groundHeight;
end
