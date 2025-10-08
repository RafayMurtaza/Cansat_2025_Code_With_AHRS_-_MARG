function analyzeMagDriftFromExcel(filename)
% analyzeMagDriftFromExcel(filename)
% -------------------------------------------------------
% Reads stationary magnetometer data (in microtesla) from an Excel file,
% computes bias, noise variance, and magnetic disturbance parameters
% useful for configuring AHRS or INS filters.
%
% Inputs:
%   filename : Excel file (e.g., 'mag_data.xlsx')
%
% Example:
%   analyzeMagDriftFromExcel('stationary_mag.xlsx');

    fprintf('\n=== Magnetometer Drift and Noise Analysis ===\n');
    fprintf('Step 1: Reading Excel file: "%s"\n', filename);

    % --- STEP 1: Read data ---
    T = readtable(filename);
    fprintf('       → File successfully loaded.\n');

    % --- STEP 2: Extract magnetometer columns ---
    fprintf('Step 2: Extracting magnetometer columns (mx, my, mz)...\n');
    vnames = T.Properties.VariableNames;
    lowernames = lower(vnames);
    expected = {'mx','my','mz'};
    foundIdx = find(ismember(lowernames, expected));
    if numel(foundIdx) < 3
        error('Could not find mx, my, mz columns in the file. Column names: %s', strjoin(vnames, ', '));
    end
    idx_mx = find(strcmpi(vnames, 'mx'),1);
    idx_my = find(strcmpi(vnames, 'my'),1);
    idx_mz = find(strcmpi(vnames, 'mz'),1);
    magData = [T.(vnames{idx_mx}), T.(vnames{idx_my}), T.(vnames{idx_mz})];
    magData = double(magData);
    N = size(magData, 1);
    t = (0:N-1)';

    fprintf('       → %d samples detected.\n', N);

    % --- STEP 3: Compute Bias (Mean Magnetic Field) ---
    fprintf('Step 3: Calculating magnetometer bias (mean field)...\n');
    bias = mean(magData, 1);
    fprintf('       → Mean Magnetic Field = [%.3f, %.3f, %.3f] µT\n', bias);
    meanMagStrength = norm(bias);
    fprintf('       → Average Magnetic Field Strength = %.3f µT\n', meanMagStrength);

    % --- STEP 4: Compute Instantaneous Noise (Standard Deviation) ---
    fprintf('Step 4: Calculating signal noise (standard deviation)...\n');
    noise_std = std(magData, 0, 1);
    noise_var = noise_std.^2; % variance (µT²)
    fprintf('       → Noise STD = [%.3f, %.3f, %.3f] µT\n', noise_std);
    fprintf('       → Noise VAR = [%.3f, %.3f, %.3f] µT²\n', noise_var);
    magNoise_scalar = mean(noise_var);
    fprintf('       → Representative MagnetometerNoise = %.3f µT²\n', magNoise_scalar);

    % --- STEP 5: Estimate Magnetic Disturbance Dynamics ---
    % Approximate slow field changes as a first-order Markov process
    fprintf('Step 5: Estimating magnetic disturbance dynamics...\n');
    window_sec = 5;  % assume ~5 s slow drift window
    w = max(1, round(N / 20)); % ~20 windows
    numWindows = floor(N / w);

    if numWindows < 2
        fprintf('       WARNING: Not enough samples for disturbance analysis.\n');
        magDrift_var = NaN;
        decayFactor = NaN;
    else
        windowMeans = zeros(numWindows, 3);
        for k = 1:numWindows
            idx = (k-1)*w + (1:w);
            windowMeans(k,:) = mean(magData(idx,:), 1);
        end

        % Compute variance of slow bias wandering
        magDrift_var = var(windowMeans, 0, 1);
        magDrift_scalar = mean(magDrift_var);
        fprintf('       → Slow-drift variance (MagDisturbanceNoise) = %.3f µT²\n', magDrift_scalar);

        % Compute approximate decay factor using first-order autocorrelation
        deltaMean = diff(windowMeans);
        corrFactor = 1 - var(deltaMean(:)) / (2 * var(windowMeans(:)));
        decayFactor = max(0, min(1, corrFactor)); % clamp to [0,1]
        fprintf('       → MagneticDisturbanceDecayFactor ≈ %.3f\n', decayFactor);
    end

    % --- STEP 6: Display Summary ---
    fprintf('\n===== Final Magnetometer Analysis Results =====\n');
    fprintf('Number of samples: %d\n', N);
    fprintf('---------------------------------------------\n');
    fprintf('Mean Field (µT):      [%.3f, %.3f, %.3f]\n', bias);
    fprintf('Signal Noise Var (µT²): [%.3f, %.3f, %.3f]\n', noise_var);
    fprintf('Avg MagnetometerNoise (µT²): %.3f\n', magNoise_scalar);
    fprintf('MagneticDisturbanceNoise (µT²): %.3f\n', magDrift_scalar);
    fprintf('MagneticDisturbanceDecayFactor: %.3f\n', decayFactor);
    fprintf('---------------------------------------------\n');
    fprintf('Interpretation:\n');
    fprintf('  - MagnetometerNoise: random sample-to-sample noise (µT²)\n');
    fprintf('  - MagneticDisturbanceNoise: slow change of magnetic bias (µT²)\n');
    fprintf('  - MagneticDisturbanceDecayFactor: 0→fast decay, 1→persistent bias\n');
    fprintf('=============================================\n\n');

    % --- STEP 7: Visualization ---
    fprintf('Step 7: Plotting magnetometer raw data and slow bias drift...\n');
    figure('Name','Magnetometer Raw Data','NumberTitle','off');
    plot(t, magData);
    xlabel('Sample Index');
    ylabel('Magnetic Field (µT)');
    legend('mx','my','mz');
    title('Raw Magnetometer Readings (Stationary Test)');
    grid on;

    if numWindows >= 2
        tw = ((0:numWindows-1) * w + w/2);
        figure('Name','Magnetometer Bias Drift','NumberTitle','off');
        plot(tw, windowMeans);
        xlabel('Sample Index');
        ylabel('Mean Magnetic Field (µT)');
        legend('mx','my','mz');
        title('Windowed Mean Magnetic Field (Slow Bias Drift)');
        grid on;
    end

    fprintf('All plots generated successfully.\n');
    fprintf('=== Analysis Complete ===\n\n');
end
