function sensors = initSensors(h0)
%% Sensor Parameters


%% Inertial Measurement Unit (MPU-9250)
sensors.IMU.timestep = 0.001;

% Gyroscope parameters
sensors.IMU.Gyro_Noise = 0.1 *pi/180;           % [rad/s-rms], 0.1 deg/s (Root Mean Square)
% sensors.IMU.Gyro_Bias = 5 *pi/180 *(2*rand-1);  % [rad/s], +/-5 deg/s, before calibration
sensors.IMU.Gyro_Bias = 0.5 *pi/180 *(2*rand-1);  % [rad/s], +/-5 deg/s
sensors.IMU.Gyro_Range = 2000;                  % 250/500/1000/2000 [deg/s]

% Accelerometer parameters 
sensors.IMU.Acc_Noise = 8 *10^-3;               % [g-rms], 8 mg (Root Mean Square)
sensors.IMU.Acc_BiasXY = 60 *10^-3 *(2*rand-1); % [g], +/-60 mg
sensors.IMU.Acc_BiasZ = 80 *10^-3 *(2*rand-1);  % [g], +/-80 mg
sensors.IMU.Acc_Range = 8;                      % 2/4/8/16 [g]

% Magnetometer parameters
% sensors.Mag.FSRange = 4800;               % [µT], +/-4800 µT Full-Scale Range
% sensors.Mag.SSFactor = 0.6;               % [µT/LSB], 0.6 µT/LSB Sensitivity Scale Factor
sensors.Mag.Bias = 500*0.6 *10^-6 *(2*rand-1);     % [LSB*T/LSB], +/-500 LSB
%Bias = 0.05/9.81 *21;
%Noise = 0.02/9.81 *21;
% --> Pseudo Bias (und Noise?)

%% Barometer (DPS310)
sensors.Baro.timestep = 0.01;
sensors.Baro.Noise = 0.5;                  % [Pa-rms], +/-0.5 Pa (RMS) (High precision mode)
sensors.Baro.Bias = 6;                     % [Pa], +/-6 Pa relative accurancy

sensors.Baro.p0 = 101325*(1-0.0065*h0/288.15)^5.255;

%% Radar 
sensors.Radar.timestep = 0.01;
sensors.Radar.Noise = 0;
sensors.Radar.Bias = 0;
sensors.Radar.lwrLimit = 0.5;                  % [m], 0.5 m lower detection limit
sensors.Radar.uprLimit = 30;                   % [m], 30 m upper detection limit

%% Attitude Estimation

sensors.Attitude.Noise = 0.1 * pi/180;

end
