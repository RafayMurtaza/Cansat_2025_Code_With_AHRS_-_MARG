global pkt_cntr ;
global g;
global calibrationSampleSize;
global T0
global N_forEllipsoidalCalibration
global collectingCalibrationData
global Fuse;
global q;
global filter;
global options;
global p;

filter = insfilterMARG('ReferenceFrame','NED',"ReferenceLocation",[31.5497,74.3436,217],"AccelerometerBiasNoise",1.5,"IMUSampleRate",25);
Fuse = ahrsfilter("SampleRate",25,"ReferenceFrame","NED","OrientationFormat","quaternion",ExpectedMagneticFieldStrength=48.5,GyroscopeNoise=0.00358898,GyroscopeDriftNoise=0.00358898,AccelerometerNoise=0.081613,MagnetometerNoise=0.09526);

g = 9.8;
pkt_cntr = 1;
calibrationSampleSize = 600;
N_forEllipsoidalCalibration = 10000;
collectingCalibrationData  = 1;
N_mag_calib = 000;
fs = 25;  % sample rate
flag_full_sensor_data_config = 1;

if flag_full_sensor_data_config == 1
    packet_size = 76;
else
    packet_size = 36;
end
 pkt_cntr = 1;
plotRate = 10;
plotSkip = round(fs / plotRate);

SENSITIVE   = 1;
NORMAL      = 2;
DAMPED      = 3;
VERYDAMPED  = 4;

delete(serialportfind);
s = serialport("COM8",115200);
configureTerminator(s,"CR/LF");
%pause (3);   % Wait for status messages from Arduino to Accumulate
numBytes = s.NumBytesAvailable;
if(numBytes > 0)
    rawData = read(s, numBytes, "uint8");   % Read status messages before configuring callback
end
fprintf('%d Bytes of status messages flushed \n', numBytes);
flush(s);


% delete(findall(0));   % Close All Previously opened GUIs


T0 = datetime('now');


% Setup track plot
% hLine = plot3(NaN,NaN,NaN,'b-','LineWidth',1.5);
% grid on; axis equal;
% xlabel('X'); ylabel('Y'); zlabel('Z');
% title('Cube Real-time Trajectory');

% --- Set UP GUIs
hGUI = initRawSensorGUI(s);


p_v_a_tolerances = [    0.00005 0.0005 0.001;   % SENSITIVE
                        0.0002 0.002 0.01;      % NORMAL
                        0.0001 0.001 0.005;     % DAMPED
                        0.0002 0.002 0.01;      % VERYDAMOED
                        ] ;

sel = NORMAL;
s.UserData = struct( "initialized" , false, "filt", imufilter('SampleRate', fs), "dt", 1/fs, "packet_size", packet_size, "pkt_cntr", 1,  ...
    "pos", [0 0 0], "vel", [0 0 0], "aPrev", [], "g", 9.8, ...
    "fUSeEllipsoidCalibrationForAccel", false, "ax_offset", 0.0, "ay_offset", 0.0, "az_offset", 0.0, ...                             % Empirically Determined Offset -- Applied at the source
    "samples_for_scale_and_bias_calibration", 210, "accMean", ones(1,3), "accBias", zeros(1,3), "gyroMean", zeros(1,3), "buffer", uint8([]),  ...
    "pDeadband", p_v_a_tolerances(sel, 1), "vDeadband", p_v_a_tolerances(sel, 2),  "zuptAccelTol", p_v_a_tolerances(sel, 3), "zuptGyroDeg", 3.0,  ...
    "hLine", hLine, "track", zeros(0,3), "plotCounter", 0, "plotSkip", 20, "GUI_RAW", hGUI, ...
    "nSampleMagnetometerCalibration", N_mag_calib, "mx", zeros(1,N_mag_calib), "my", zeros(1,N_mag_calib), "mz", zeros(1,N_mag_calib), "k",1, "magx_offset", -6.5, "magy_offset", -17.5, "magz_offset", -12.0, ...
    "fApplyAccCorrection", 0, "fApplyGyroCorrection", 1, "fApplyMagCorrection",0, "tStart" ,0 ...
    );




% Calibration of Acceleration - Related 
s.UserData.calib = struct();
s.UserData.calib.Araw     = zeros(12000,3);   % preallocate
s.UserData.calib.count    = 0;                % number of samples stored
s.UserData.calib.maxCount = 12000;            % target dataset size
s.UserData.calib.collect  = false;            % flag: are we collecting?



% Set Up Gyro Monitor
 initGyroGUI(s); 
 % initPosVelAccelGUI(s);


configureCallback(s,  "byte", s.UserData.packet_size, @esp32_packet_callback)