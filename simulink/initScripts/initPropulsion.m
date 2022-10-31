function propulsion = initPropulsion()

% combined 4 motor measurement
motorCommand        = [1000; 1050; 1100; 1150; 1200; 1250; 1300; 1350; 1400; 1450; 1500; 1550; 1600; 1650];
electricCurrent     = [0; 1.4; 1.9; 2.6; 3.3; 4.2; 5.1; 6.2; 7.3; 8.6; 9.8; 11.0; 12.3; 13.7];
voltage             = [12; 11.89; 11.86; 11.83; 11.80; 11.75; 11.72; 11.66; 11.60; 11.54; 11.48; 11.42; 11.35; 11.28];
mass                = [0; 173; 230; 290; 357; 420; 490; 550; 600; 660; 700; 740; 780; 825] * 1/1000;
revolutionPerMinute = [0; 5580; 6330; 6990; 7680; 8280; 8880; 9260; 9870; 10230; 10530; 10980; 11200; 11520];
n                   = 4;

g = 9.806;
lever = 170/228.5;

% 1 motor
propulsion.PWM    = motorCommand;
propulsion.Omega  = 2*pi/60* revolutionPerMinute;
propulsion.thrust = mass .* lever .* g ./ n;
propulsion.torque = propulsion.thrust ./ 73;  % approx alpha_q/alpha_t for small motors/props
propulsion.I      = electricCurrent ./ n;
propulsion.power  = voltage .* propulsion.I;
propulsion.U      = voltage;

propulsion.lowPassTimeConstant = 0.025;





end
