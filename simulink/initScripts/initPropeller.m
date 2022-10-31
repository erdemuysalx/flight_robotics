function propeller = initPropeller()


% Graupner 5x3"
% propeller.radius    = 0.127/2;                                  % propeller radius (diameter = 5 " = 0.127 m)
% propeller.blades    = 2;                                        % number of propeller blades
% % #1 propeller.pitch     = 2*atan(0.0762/(2*pi*propeller.radius));     % pitch angle at propeller tip (-> 3 " = 0.0762 m)
% propeller.pitch     = atan(0.0762/(2*pi*propeller.radius));     % pitch angle at propeller tip (-> 3 " = 0.0762 m)
% propeller.chord     = 0.013;                                     % mean chord of the propeller blades
% mass                = 0.0028;

% DALprop 5x4.5"
propeller.radius    = 0.127/2;                                  % propeller radius (diameter = 5 " = 0.127 m)
propeller.blades    = 2;                                        % number of propeller blades
propeller.pitch     = atan(0.1143/(2*pi*propeller.radius));     % pitch angle at propeller tip (-> 4.5 " = 0.1143 m)
propeller.chord     = 0.017;                                     % mean chord of the propeller blades
mass                = 0.004;


propeller.J         = 2*(propeller.radius/2)^2*mass/2;          % Steiner's law: mass*lever^2
propeller.delta     = [0.0087, -0.012, 0.4];                    % coefficients proposed by Bailey (A simplified theoretical method..., p. 9) for NACA23012 airfoil for 2e6 < Re < 8e6
%cif        = 0;                                                % installation angle elevation
%caf        = 0;                                                % installation angle azimuth
%cpos       = [0.1;0;0.01];                                     % installation position
%cfoldSpeed = 50*pi/180;

% Propeller installation positions [m]
%xDistance = 0.163/2;
%yDistance = 0.232/2;
xDistance = 0.25
yDistance = 0.25
zDistance = 0.021;
propeller.positions = [ xDistance,  -yDistance,  -zDistance;                 % Prop 1            
                        xDistance,  yDistance,   -zDistance;                 % Prop 2
                        -xDistance, yDistance,   -zDistance;                 % Prop 3
                        -xDistance, -yDistance,  -zDistance];                % Prop 4
propeller.rotDirs   = [ -1; 1; -1; 1];                           % Rotation directions
propeller.count = length(propeller.rotDirs);

% ToDo: define indiviual propeller data 
% --> propeller(1).radius = ...
% --> propeller{1}.radius = ...