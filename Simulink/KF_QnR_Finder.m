% accel is Nx3 matrix, gyro is Nx3 matrix
ax = imuData(:, 1);
ay = imuData(:, 2);
az = imuData(:, 3);

% Calculate static pitch angle vector (in radians)
theta_acc_static = atan2(ax, sqrt(ay.^2 + az.^2));

% Extract gyro Y-axis vector (in rad/s or deg/s)
gyro_y_static = imuData(:, 5);

R_val = var(theta_acc_static);
fprintf('Measurement Noise R = %.8f\n', R_val);