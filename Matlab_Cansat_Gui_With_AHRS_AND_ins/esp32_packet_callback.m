function esp32_packet_callback(src, ~)
    % Callback whenever ud.packet_size bytes are received.
    global timeSincePowerUp
    global T0
    global q
    global p
    global Fuse
    global filter
    global g
    
    g = src.UserData.g; 

    sensor_data = struct( Header = 0, Temperature= 0.0, Altitude= 0.1, Pressure= 0.3, Heading = 0.4 , Xacc = 0.5 ,   Yacc  =  0.7  ,  Zacc  = 0.8  ,  Angaccx  =  0.9  ,  Angaccy  =  0.10  ,  Angaccz  =  0.11  ,  Magx  =  0.12  ,  Magy  =  0.13  ,  Magz  =  0.14  ,  Sat  =  0.15  ,  Lat  =  0.16  ,  Long  =  0.17  ,  GPSAlt = 0.18, Pkt_num = 0, TimeStamp = 0, Recv_time = ' ' );
    %[sensor_data] = read_complete_sensor_data_from_esp32(src, sensor_data);
    %update_raw_sensor_gui(src);

    packets = read_complete_sensor_data_from_esp32(src);

    for k = 1:numel(packets)
        sensor_data = packets{k};
        ud = src.UserData;
        %% --- Your custom code to process the packet goes here ---
      
        % Displaying every 100th Packet
        if( rem(sensor_data.Pkt_num, 100) == 0)
            fprintf('No = %d,   Accl =[%0.2f, %0.2f, %0.2f] - Ang =[%0.2f, %0.2f, %0.2f] -- Mag = [%0.2f, %0.2f, %0.2f], TimeStamp = %d, Temp = %0.2f, Press = %0.2f, Alti = %0.2f, Heading = %0.2f', ...
               sensor_data.Pkt_num , sensor_data.Xacc, sensor_data.Yacc, sensor_data.Zacc, ...
               sensor_data.Angaccx, sensor_data.Angaccy, sensor_data.Angaccz, ...
               sensor_data.Magx, sensor_data.Magy, sensor_data.Magz, sensor_data.TimeStamp, ...
               sensor_data.Temperature,sensor_data.Pressure,sensor_data.Altitude,sensor_data.Heading );
    
            fprintf('Reading Receive Time = %s \n', sensor_data.Recv_time);
        end

        %Start-Up Bias Measurement for Accelerometer and Gyroscope
        if(sensor_data.Pkt_num < ud.samples_for_scale_and_bias_calibration)  % waiting for the calibration data to gather
            ud.accMean  = ( ud.accMean *(sensor_data.Pkt_num-1) + [sensor_data.Xacc sensor_data.Yacc sensor_data.Zacc])/sensor_data.Pkt_num;
            ud.gyroMean = ( ud.gyroMean*(sensor_data.Pkt_num-1) + [sensor_data.Angaccx sensor_data.Angaccy sensor_data.Angaccz])/sensor_data.Pkt_num;
            src.UserData = ud;
            continue;
        end

        if(ud.fApplyMagCorrection)
            sensor_data.Magx = sensor_data.Magx - ud.magx_offset;
            sensor_data.Magy = sensor_data.Magy - ud.magy_offset;
            sensor_data.Magz = sensor_data.Magz - ud.magz_offset;
        end
    
        if(ud.fApplyAccCorrection)
            sensor_data.Xacc = sensor_data.Xacc - ud.ax_offset;
            sensor_data.Yacc = sensor_data.Yacc - ud.ay_offset;
            sensor_data.Zacc = sensor_data.Zacc - ud.az_offset;
        end
    
        if(ud.fApplyGyroCorrection)           % Apply Gyro Means Correction if enabled
            sensor_data.Angaccx = sensor_data.Angaccx - ud.gyroMean(1);
            sensor_data.Angaccy = sensor_data.Angaccy - ud.gyroMean(2);
            sensor_data.Angaccz = sensor_data.Angaccz - ud.gyroMean(3);
        end

        guiUpdateRawSensorData(src, sensor_data);         
        guiUpdateGyroData;    

        timeSincePowerUp = datetime('now') - T0;
        src.UserData = ud;
    end
        % --- Accelerometer Calibration ---
     sensor_data.Xacc = (sensor_data.Xacc - 0.83) * 0.994);
     sensor_data.Yacc = (sensor_data.Yacc - 0.21) * 0.997;
     sensor_data.Zacc = (sensor_data.Zacc + 1.45) * 0.972;
    % 
    % % --- Magnetometer Calibration ---
     bias = [-1.821, -0.628, 11.873]; % Hard-iron offset
    M = [ 0.9783,  0.0056,  0.0070;
           0.0056,  0.9354, -0.0592;
           0.0070, -0.0592,  1.1088 ]; % Soft-iron correction
     magVec = [sensor_data.Magx; sensor_data.Magy; sensor_data.Magz];
    magCorrected = M * (magVec - bias');
    sensor_data.Magx = magCorrected(1);
    sensor_data.Magy = magCorrected(2);
    sensor_data.Magz = magCorrected(3);
    % 
    % 
    acc_data = [sensor_data.Xacc,sensor_data.Yacc,sensor_data.Zacc];
    mag_data = [sensor_data.Magx,sensor_data.Magy,sensor_data.Magz];
    gyro_data = [sensor_data.Angaccx,sensor_data.Angaccy,sensor_data.Angaccz];

    [orientation,angularVelocity,interdata] = Fuse(acc_data,gyro_data,mag_data);
    q = orientation;
    plotRealtimeQuaternion(q);
    %bias = [-1.821, -0.628, 11.873]; % Hard-iron offset
    
    acc_data  = [sensor_data.Xacc, sensor_data.Yacc, sensor_data.Zacc];
    gyro_data = [sensor_data.Angaccx, sensor_data.Angaccy, sensor_data.Angaccz];
    mag_data  = [sensor_data.Magx, sensor_data.Magy, sensor_data.Magz];

    % Step 1: Always predict using IMU
    filter.predict(acc_data, gyro_data);

    % Step 2: Check GPS availability
    gpsValid = (sensor_data.Sat >= 4) && ...     % at least 4 satellites
               (sensor_data.Lat ~= 0) && ...
               (sensor_data.Long ~= 0);

    if gpsValid
        % --- Fuse GPS position only ---
        lla = [sensor_data.GPSAlt, sensor_data.Long, sensor_data.Lat];  % degrees, meters
        Rpos = 2.5;     % GPS position covariance (m^2)
        filter.fusegps(lla, Rpos);
    else
        % --- Fuse Magnetometer only ---
        Rmag = eye(3) * 0.5;   % example covariance, tune as needed
        filter.fusemag(mag_data, Rmag);
    end

    % Step 3: Get current estimated pose
    [p, q] = pose(filter,'quaternion');
% --- Limit plotting rate to avoid GUI freeze ---
    persistent lastPlotTime
    if isempty(lastPlotTime)
        lastPlotTime = tic;
    end

    if toc(lastPlotTime) > 0.1   % update every 0.1s (10 Hz)
        plotRealtimePositionAndOrientation(p,q);
        lastPlotTime = tic;
    end
end
