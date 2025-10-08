function analyzeGyroDriftFromExcel(filename)
% analyzeGyroDriftFromExcel(filename)
% ---------------------------------------------
% Reads stationary gyroscope data from an Excel file,
% computes its bias, short-term noise (std), and bias-wander (drift) std.
% It also visualizes the raw data, integrated drift, and windowed bias.
%
% Input:
%   filename : Excel file name (e.g., 'stationary_gyro.xlsx')
%
% Notes:
% - Assumes gyro columns are named gx, gy, gz
% - Tries to detect a timestamp column to estimate dt; otherwise assumes 100 Hz

    fprintf('\n=== Gyroscope Drift and Noise Analysis ===\n');
    fprintf('Step 1: Reading Excel file: "%s"\n', filename);

    % --- STEP 1: Read data ---
    T = readtable(filename);
    fprintf('       → File successfully loaded.\n');

    % --- TRY TO FIND A TIMESTAMP COLUMN TO ESTIMATE dt ---
    vnames = T.Properties.VariableNames;
    lowernames = lower(vnames);
    tsCandidates = {'pythontimestamp','hardwaretimestamp','timestamp','time','t','timesec'};
    tsIdx = find(ismember(lowernames, tsCandidates), 1);

    if ~isempty(tsIdx)
        rawTs = T.(vnames{tsIdx});
        if ~isnumeric(rawTs)
            rawTs = str2double(rawTs);
        end
        dts = diff(double(rawTs));
        dt_raw = median(dts(~isnan(dts) & ~isinf(dts)));
        % heuristic for units: if huge, assume micro/milli seconds
        if isempty(dt_raw) || dt_raw <= 0 || isnan(dt_raw)
            useDefaultDt = true;
        else
            useDefaultDt = false;
            if dt_raw > 1e6
                dt = dt_raw / 1e6;
            elseif dt_raw > 1e3
                dt = dt_raw / 1e3;
            else
                dt = dt_raw;
            end
        end
    else
        useDefaultDt = true;
    end

    if exist('useDefaultDt','var') && useDefaultDt
        fs = 100; % default sampling rate (Hz) if no timestamp detected
        dt = 1 / fs;
        fprintf('No usable timestamp detected. Assuming fs = %g Hz (dt = %.4g s).\n', fs, dt);
    else
        fprintf('Detected timestamp column "%s". Estimated sample interval dt = %.6g s\n', vnames{tsIdx}, dt);
    end

    % --- STEP 2: Extract gyroscope columns ---
    fprintf('Step 2: Extracting gyroscope columns (gx, gy, gz)...\n');
    expected = {'gx','gy','gz'};
    foundIdx = find(ismember(lowernames, expected));
    if numel(foundIdx) < 3
        error('Could not find gx, gy, gz columns in the file. Column names: %s', strjoin(vnames, ', '));
    end
    % ensure they are in order gx, gy, gz
    % map exact indices for gx, gy, gz
    idx_gx = find(strcmpi(vnames, 'gx'),1);
    idx_gy = find(strcmpi(vnames, 'gy'),1);
    idx_gz = find(strcmpi(vnames, 'gz'),1);
    gyroData = [T.(vnames{idx_gx}), T.(vnames{idx_gy}), T.(vnames{idx_gz})];
    gyroData = double(gyroData); % ensure numeric
    N = size(gyroData, 1);
    total_time = (N-1) * dt;
    t = (0:N-1)' * dt; % time vector in seconds
    fprintf('       → %d samples (total duration ~ %.2f s)\n', N, total_time);

    % --- STEP 3: Compute Bias (Mean) ---
    fprintf('Step 3: Calculating gyro bias (mean angular velocity)...\n');
    bias = mean(gyroData, 1);
    fprintf('       → Bias = [%.6f, %.6f, %.6f] rad/s\n', bias);

    % --- STEP 4: Compute Short-term Noise (Standard Deviation) ---
    fprintf('Step 4: Calculating short-term noise (standard deviation of raw samples)...\n');
    noise_std = std(gyroData, 0, 1);
    fprintf('       → Short-term Noise STD (rad/s) = [gx=%.6f, gy=%.6f, gz=%.6f]\n', noise_std);
    gyroNoise_scalar = mean(noise_std);
    fprintf('       → Representative GyroNoise (scalar) = %.6g rad/s (mean of axes)\n', gyroNoise_scalar);

    % --- STEP 5: Estimate Bias-wander (Drift) via windowing ---
    % choose window length (seconds). Default 5s or smaller if recording is short
    if total_time < 10
        window_sec = max(1, floor(total_time / 4)); % at least 1 second, split into ~4 windows
    else
        window_sec = 5;
    end
    w = max(1, round(window_sec / dt)); % window size in samples
    numWindows = floor(N / w);

    fprintf('Step 5: Estimating bias-wander (drift) using windowing:\n');
    fprintf('       → Window length = %g s (%d samples). Number of full windows = %d\n', window_sec, w, numWindows);

    if numWindows < 2
        fprintf('       WARNING: Not enough data windows to estimate bias-wander reliably (need >=2 windows).\n');
        gyroDrift_std_per_axis = [NaN NaN NaN];
        gyroDrift_scalar = NaN;
    else
        windowMeans = zeros(numWindows, 3);
        for k = 1:numWindows
            idx = (k-1)*w + (1:w);
            windowMeans(k,:) = mean(gyroData(idx,:), 1);
        end
        % std of window means -> how the bias moves between windows
        gyroDrift_std_per_axis = std(windowMeans, 0, 1);
        gyroDrift_scalar = mean(gyroDrift_std_per_axis);
        fprintf('       → Drift (bias-wander) STD (rad/s) = [gx=%.6g, gy=%.6g, gz=%.6g]\n', gyroDrift_std_per_axis);
        fprintf('       → Representative GyroDriftNoise (scalar) = %.6g rad/s (mean of axes)\n', gyroDrift_scalar);
    end

    % --- STEP 6: Compute integrated drift (angle) using dt ---
    fprintf('Step 6: Integrating (gyro - bias) to visualize accumulated angle drift over time...\n');
    ang = cumsum((gyroData - bias) * dt, 1); % integrated angle in radians
    fprintf('       → Integration done (angles in radians).\n');

    % --- STEP 7: Display Summary ---
    fprintf('\n===== Final Gyro Drift Analysis Results =====\n');
    fprintf('Number of samples: %d (duration %.2f s)\n', N, total_time);
    fprintf('---------------------------------------------\n');
    fprintf('Bias (rad/s):        gx=%.6f, gy=%.6f, gz=%.6f\n', bias);
    fprintf('Short-term Noise STD (rad/s): gx=%.6f, gy=%.6f, gz=%.6f\n', noise_std);
    if ~any(isnan(gyroDrift_std_per_axis))
        fprintf('Bias-wander STD (rad/s):       gx=%.6g, gy=%.6g, gz=%.6g\n', gyroDrift_std_per_axis);
    else
        fprintf('Bias-wander STD: not enough data to estimate reliably.\n');
    end
    fprintf('Representative scalar values (use these as starting points for AHRS):\n');
    fprintf('  GyroscopeNoise       = %.6g rad/s\n', gyroNoise_scalar);
    fprintf('  GyroscopeDriftNoise  = %.6g rad/s\n', gyroDrift_scalar);
    fprintf('---------------------------------------------\n');
    fprintf('Interpretation:\n');
    fprintf('  - Short-term noise (std) describes high-frequency jitter around the mean.\n');
    fprintf('  - Bias-wander/std of window means describes slow change of the bias over seconds.\n');
    fprintf('  - For AHRS: set GyroscopeNoise to the short-term noise (or 0.5-2x of it)\n');
    fprintf('    and GyroscopeDriftNoise to the bias-wander estimate (or slightly larger to allow bias adaptation).\n');
    fprintf('=============================================\n\n');

    % --- STEP 8: Plot raw data ---
    fprintf('Step 8: Plotting raw gyroscope data...\n');
    figure('Name','Gyro Raw Data','NumberTitle','off');
    plot(t, gyroData);
    xlabel('Time (s)');
    ylabel('Angular Velocity (rad/s)');
    legend('gx','gy','gz');
    title('Raw Gyroscope Readings (Stationary Test)');
    grid on;

    % --- STEP 9: Plot integrated angle drift ---
    fprintf('Step 9: Plotting integrated (bias-corrected) angle drift...\n');
    figure('Name','Gyro Integrated Drift','NumberTitle','off');
    plot(t, ang);
    xlabel('Time (s)');
    ylabel('Integrated Angle (rad)');
    legend('gx','gy','gz');
    title('Integrated Angle Drift (gyro - bias integrated)');
    grid on;

    % --- STEP 10: Plot window means (bias over time) if available ---
    if numWindows >= 1 && exist('windowMeans','var')
        tw = ((0:numWindows-1) * w + w/2) * dt; % center time of windows
        figure('Name','Window Means (Bias over time)','NumberTitle','off');
        plot(tw, windowMeans);
        xlabel('Time (s)');
        ylabel('Mean Angular Rate (rad/s)');
        legend('gx','gy','gz');
        title(sprintf('Window Means (window = %.2f s) — shows bias wander', window_sec));
        grid on;
    end

    fprintf('All plots generated successfully.\n');
    fprintf('=== Analysis Complete ===\n\n');
end
