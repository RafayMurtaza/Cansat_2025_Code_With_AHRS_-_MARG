function analyzeAccelNoiseFromExcel(filename)
% analyzeAccelNoiseFromExcel(filename)
% ---------------------------------------------
% Analyzes stationary accelerometer data (in m/s^2)
% Computes bias, noise std, and drift trend.
%
% Input:
%   filename : Excel file (e.g., 'stationary_accel.xlsx')
%
% Example:
%   analyzeAccelNoiseFromExcel('stationary_accel.xlsx');

    fprintf('\n=== Accelerometer Noise and Drift Analysis ===\n');
    fprintf('Step 1: Reading Excel file: "%s"\n', filename);

    % --- STEP 1: Read data ---
    T = readtable(filename);
    fprintf('       → File successfully loaded.\n');

    % --- STEP 2: Extract accelerometer columns ---
    fprintf('Step 2: Extracting accelerometer columns (ax, ay, az)...\n');
    accelData = [T.ax, T.ay, T.az];
    N = size(accelData, 1);
    t = (0:N-1)'; % sample index for plotting
    fprintf('       → %d samples detected.\n', N);

    % --- STEP 3: Compute Bias (Mean) ---
    fprintf('Step 3: Calculating accelerometer bias (mean value)...\n');
    bias = mean(accelData, 1);
    fprintf('       → Bias = [%.6f, %.6f, %.6f] m/s²\n', bias);

    % --- STEP 4: Compute Noise (Standard Deviation & Variance) ---
    fprintf('Step 4: Calculating noise (standard deviation and variance)...\n');
    noise_std = std(accelData, 0, 1);
    noise_var = var(accelData, 0, 1);
    fprintf('       → Noise STD  = [%.6f, %.6f, %.6f] m/s²\n', noise_std);
    fprintf('       → Noise VAR  = [%.6f, %.6f, %.6f] (m/s²)²\n', noise_var);

    % --- STEP 5: Compute Drift Trend ---
    fprintf('Step 5: Estimating drift by integrating (accel - bias)...\n');
    drift = cumsum(accelData - bias);
    fprintf('       → Drift trend computed (cumulative integration).\n');

    % --- STEP 6: Display Summary ---
    fprintf('\n===== Final Accelerometer Analysis Results =====\n');
    fprintf('Number of samples: %d\n', N);
    fprintf('---------------------------------------------\n');
    fprintf('Bias (m/s²):     ax=%.6f, ay=%.6f, az=%.6f\n', bias);
    fprintf('Noise STD:        ax=%.6f, ay=%.6f, az=%.6f\n', noise_std);
    fprintf('Noise VAR:        ax=%.6f, ay=%.6f, az=%.6f\n', noise_var);
    fprintf('---------------------------------------------\n');
    fprintf('Interpretation:\n');
    fprintf('  - Bias: constant offset due to sensor or gravity misalignment\n');
    fprintf('  - Noise STD/VAR: random jitter; used to set measurement noise R matrix\n');
    fprintf('  - Drift: shows bias instability or temperature effects\n');
    fprintf('=============================================\n\n');

    % --- STEP 7: Plot raw accelerometer data ---
    fprintf('Step 7: Plotting raw accelerometer data...\n');
    figure('Name','Accel Raw Data','NumberTitle','off');
    plot(t, accelData);
    xlabel('Sample Index');
    ylabel('Acceleration (m/s²)');
    legend('ax','ay','az');
    title('Raw Accelerometer Readings (Stationary Test)');
    grid on;

    % --- STEP 8: Plot drift trend ---
    fprintf('Step 8: Plotting integrated drift trend...\n');
    figure('Name','Accel Drift Trend','NumberTitle','off');
    plot(t, drift);
    xlabel('Sample Index');
    ylabel('Integrated Drift (arbitrary units)');
    legend('ax','ay','az');
    title('Cumulative Accelerometer Drift (Stationary Test)');
    grid on;

    fprintf('All plots generated successfully.\n');
    fprintf('=== Analysis Complete ===\n\n');
end
