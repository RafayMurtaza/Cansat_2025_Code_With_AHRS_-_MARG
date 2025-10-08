% function [sensor_data] = read_complete_sensor_data_from_esp32(src, sensor_data )
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Gets bytes from serial buffer, deserializes them and stores them in sensor_data structure. 
% % Inputs and Outputs
% % src: serial port object
% % sensor_data: sensor_data structure with a header
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% new_pkt_available = 0;   % flag
% HEADER = uint32(0xAABBCCDD);  % <-- your sync word
% ud = src.UserData;
% buffer = ud.buffer;
% 
% 
% % Check for the availability of new bytes in serial buffer
% if( src.NumBytesAvailable > 0)
%     newData = read(src, src.NumBytesAvailable, "uint8")';
% else
%     return;
% end
% 
% % ------ Extract data from buffer and store it in sensor_data structure -- 
% buffer = [buffer; newData];                           
% while length(buffer) >= ud.packet_size                        % Search for Header while more buffer size is greater than ud.packet_size
%     headerCandidate = typecast(buffer(1:4), 'uint32');
%     if headerCandidate == HEADER                              % Header found, extract packet
%         packet = buffer(1:ud.packet_size);
%         buffer(1:ud.packet_size) = [];
% 
%         offset = 1;
%         sensor_data.Header = typecast(uint8(packet(offset:offset+3)), 'uint32'); 
%         offset = offset+4;
%         sensor_data.Temperature = typecast(uint8(packet(offset:offset+3)), 'single'); 
%         offset = offset+4;
%         sensor_data.Altitude = typecast(uint8(packet(offset:offset+3)), 'single'); 
%         offset = offset+4;
%         sensor_data.Pressure = typecast(uint8(packet(offset:offset+3)), 'single'); 
%         offset = offset+4;
%         sensor_data.Heading  = typecast(uint8(packet(offset:offset+3)), 'single'); 
%         offset = offset+4;
%         sensor_data.Xacc     = typecast(uint8(packet(offset:offset+3)), 'single') - 0*ud.ax_offset; 
%         offset = offset+4;
%         sensor_data.Yacc     = typecast(uint8(packet(offset:offset+3)), 'single') - 0*ud.ay_offset; 
%         offset = offset+4;
%         sensor_data.Zacc     = typecast(uint8(packet(offset:offset+3)) , 'single') -0*ud.az_offset; 
%         offset = offset+4;
%         sensor_data.Angaccx  = typecast(uint8(packet(offset:offset+3)), 'single'); 
%         offset = offset+4;
%         sensor_data.Angaccy  = typecast(uint8(packet(offset:offset+3)), 'single');
%         offset = offset+4;
%         sensor_data.Angaccz  = typecast(uint8(packet(offset:offset+3)), 'single');
%         offset = offset+4;
%         sensor_data.Magx     = typecast(uint8(packet(offset:offset+3)), 'single'); 
%         offset = offset+4;
%         sensor_data.Magy     = typecast(uint8(packet(offset:offset+3)), 'single');
%         offset = offset+4;
%         sensor_data.Magz     = typecast(uint8(packet(offset:offset+3)), 'single');
%         offset = offset+4;
%         sensor_data.Sat      = typecast(uint8(packet(offset:offset+3)), 'single');
%         offset = offset+4;
%         sensor_data.Lat      = typecast(uint8(packet(offset:offset+3)), 'single'); 
%         offset = offset+4;
%         sensor_data.Long     = typecast(uint8(packet(offset:offset+3)), 'single');
%         offset = offset+4;
%         sensor_data.GPSAlt   = typecast(uint8(packet(offset:offset+3)), 'single');
%         offset = offset+4;
%         sensor_data.TimeStamp= typecast(uint8(packet(offset:offset+3)), 'uint32');
%         sensor_data.Pkt_num  = ud.pkt_cntr;
%         sensor_data.Recv_time= datetime('now');
% 
%         new_pkt_available = 1;
%         break;                     % sensor_data is received, break out of the loop to process it in the caller routine
%     else
%         buffer(1) = []; % shift until aligned
%         new_pkt_available = 0;
%     end
% end
% 
% if(new_pkt_available == 1)              %sensor_data contains new packet
% 
%     if( ud.pkt_cntr < ud.samples_for_scale_and_bias_calibration)
%         ud.accMean  = ( ud.accMean *(ud.pkt_cntr-1) + [sensor_data.Xacc sensor_data.Yacc sensor_data.Zacc])/ud.pkt_cntr;
%         ud.gyroMean = ( ud.gyroMean*(ud.pkt_cntr-1) + [sensor_data.Angaccx sensor_data.Angaccy sensor_data.Angaccz])/ud.pkt_cntr;
%     end
% 
%     if( rem(ud.pkt_cntr, 100) == 0)
%         fprintf('No = %d, Buff = %d,  Accl =[%0.2f, %0.2f, %0.2f] - Ang =[%0.2f, %0.2f, %0.2f] -- Mag = [%0.2f, %0.2f, %0.2f], TimeStamp = %d, Temp = %0.2f, Press = %0.2f, Alti = %0.2f, Heading = %0.2f', ...
%            sensor_data.Pkt_num, length(buffer) , sensor_data.Xacc, sensor_data.Yacc, sensor_data.Zacc, ...
%            sensor_data.Angaccx, sensor_data.Angaccy, sensor_data.Angaccz, ...
%            sensor_data.Magx, sensor_data.Magy, sensor_data.Magz, sensor_data.TimeStamp, ...
%            sensor_data.Temperature,sensor_data.Pressure,sensor_data.Altitude,sensor_data.Heading );
% 
% 
%         fprintf('Reading Receive Time = %s \n', sensor_data.Recv_time);
%         %fprintf('GPS numSat = %0.2f, Lat = %0.6f , Longi = %0.6f, GPS Alt = %0.2f ', sensor_data.Sat, sensor_data.Lat, sensor_data.Long, sensor_data.GPSAlt);
%     end
% end
% 
% ud.pkt_cntr = ud.pkt_cntr + 1;
% src.UserData = ud;
% 
% end

function sensor_datas = read_complete_sensor_data_from_esp32(src)
    % Returns a cell array of decoded packets (0, 1, or many)

    HEADER = uint32(0xAABBCCDD);
    ud = src.UserData;
    buffer = ud.buffer;
    sensor_datas = {};   % <-- will hold all decoded packets

    % Read all available bytes
    if src.NumBytesAvailable > 0
        newData = read(src, src.NumBytesAvailable, "uint8")';
        buffer = [buffer; newData];
    end

    % Try to extract as many packets as possible
    while length(buffer) >= ud.packet_size
        headerCandidate = typecast(buffer(1:4), 'uint32');
        if headerCandidate == HEADER
            % Extract one packet
            packet = buffer(1:ud.packet_size);
            buffer(1:ud.packet_size) = [];

            % Deserialize into struct
            sensor_data = parse_packet(packet, ud);
            ud.pkt_cntr = ud.pkt_cntr + 1;

            % Append to list
            sensor_datas{end+1} = sensor_data;
        else
            % Misaligned, discard one byte and keep searching
            buffer(1) = [];
        end
    end

    % Save leftover buffer back
    ud.buffer = buffer;
    src.UserData = ud;
end




function sensor_data = parse_packet(packet, ud)
    offset = 1;
    sensor_data.Header = typecast(uint8(packet(offset:offset+3)), 'uint32'); 
    offset = offset+4;
    sensor_data.Temperature = typecast(uint8(packet(offset:offset+3)), 'single'); 
    offset = offset+4;
    sensor_data.Altitude = typecast(uint8(packet(offset:offset+3)), 'single'); 
    offset = offset+4;
    sensor_data.Pressure = typecast(uint8(packet(offset:offset+3)), 'single'); 
    offset = offset+4;
    sensor_data.Heading  = typecast(uint8(packet(offset:offset+3)), 'single'); 
    offset = offset+4;
    sensor_data.Xacc     = typecast(uint8(packet(offset:offset+3)), 'single') - 0*ud.ax_offset; 
    offset = offset+4;
    sensor_data.Yacc     = typecast(uint8(packet(offset:offset+3)), 'single') - 0*ud.ay_offset; 
    offset = offset+4;
    sensor_data.Zacc     = typecast(uint8(packet(offset:offset+3)) , 'single') -0*ud.az_offset; 
    offset = offset+4;
    sensor_data.Angaccx  = typecast(uint8(packet(offset:offset+3)), 'single'); 
    offset = offset+4;
    sensor_data.Angaccy  = typecast(uint8(packet(offset:offset+3)), 'single');
    offset = offset+4;
    sensor_data.Angaccz  = typecast(uint8(packet(offset:offset+3)), 'single');
    offset = offset+4;
    sensor_data.Magx     = typecast(uint8(packet(offset:offset+3)), 'single'); 
    offset = offset+4;
    sensor_data.Magy     = typecast(uint8(packet(offset:offset+3)), 'single');
    offset = offset+4;
    sensor_data.Magz     = typecast(uint8(packet(offset:offset+3)), 'single');
    offset = offset+4;
    sensor_data.Sat      = typecast(uint8(packet(offset:offset+3)), 'single');
    offset = offset+4;
    sensor_data.Lat      = typecast(uint8(packet(offset:offset+3)), 'single'); 
    offset = offset+4;
    sensor_data.Long     = typecast(uint8(packet(offset:offset+3)), 'single');
    offset = offset+4;
    sensor_data.GPSAlt   = typecast(uint8(packet(offset:offset+3)), 'single');
    offset = offset+4;
    sensor_data.TimeStamp= typecast(uint8(packet(offset:offset+3)), 'uint32');
    sensor_data.Pkt_num  = ud.pkt_cntr;
    sensor_data.Recv_time= datetime('now');

end