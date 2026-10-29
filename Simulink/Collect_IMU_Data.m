clear s;
clc;

% Define serial port parameters
portName = '/dev/ttyACM0';
baudRate = 115200;

% Open serial connection
fprintf('Connecting to %s ... \n', portName);
s = serialport(portName, baudRate);
flush(s);

% Verify the connection
if isvalid(s)
    fprintf('Connection established successfully.\n');
else
    error('Failed to establish connection.');
end

numSamples = 1000;
fprintf('Keep board STILL! Collecting %d static samples ... \n', numSamples);

% Pre-allocate matrix [ax, ay, az, gx, gy, gz]
imuData = zeros(numSamples, 6);

% Struct definition length (6 floats * 4 bytes = 24 bytes)
bytesPerPacket = 24;

i = 1;
timeoutTimer = tic;
maxWaitTime = 5; % 5-second timeout if no data flows

while i <= numSamples
    % Check if data is arriving at all
    avail = s.NumBytesAvailable;
    
    if avail >= bytesPerPacket
        % Read exact packet
        rawBytes = read(s, bytesPerPacket, "uint8");
        floatVals = typecast(uint8(rawBytes), 'single');
        
        if length(floatVals) == 6
            imuData(i, :) = double(floatVals);
            i = i + 1;
            timeoutTimer = tic; % Reset timeout on successful read
        end
    else
        % Check for hang condition
        if toc(timeoutTimer) > maxWaitTime
            clear s;
            error('TIMED OUT: Received only %d bytes in buffer. Verify RP2040 is running and sending data.', avail);
        end
        pause(0.001); % Yield slightly to allow buffer to fill
    end
end

% Clear port to free Linux handle
clear s;
fprintf('Closed the serial connection.\n');

% Extract IMU channels
ax = imuData(:, 1);
ay = imuData(:, 2);
az = imuData(:, 3);
gx = imuData(:, 4);
gy = imuData(:, 5); % Gyro Y
gz = imuData(:, 6);

% FIXED BUG 1: Changed atan to atan2
theta_acc_static = atan2(ax, sqrt(ay.^2 + az.^2));
R_val = var(theta_acc_static);

% Compute process noise Q
dt = 0.01;

% FIXED BUG 2: Changed gyro_y to gy
sigma_gyro = std(gy); 
Q_mat = [(dt^2)*(sigma_gyro^2), 0;
         0                    , 1e-6];

fprintf("\nCalculated R = %.8f \n", R_val);
disp("Calculated Q ="); 
disp(Q_mat);