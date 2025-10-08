function plotRealtimeQuaternion(q)
% Fast real-time 3D visualization of orientation using quaternion input.
% Keeps updating a single 3D figure efficiently.

persistent fig hAxes xAxis yAxis zAxis firstRun

if isempty(fig) || ~isvalid(fig)
    % Create figure and axes once
    fig = figure('Name','3D Orientation Viewer','NumberTitle','off');
    hAxes = axes('Parent',fig);
    axis(hAxes,[-1 1 -1 1 -1 1]*1.5);
    axis(hAxes,'equal');
    grid(hAxes,'on');
    xlabel(hAxes,'East (X)');
    ylabel(hAxes,'North (Y)');
    zlabel(hAxes,'Up (Z)');
    view(hAxes,[45 30]);
    title(hAxes,'Real-Time Orientation (AHRS)');
    hold(hAxes,'on');

    % Draw fixed origin
    plot3(hAxes,0,0,0,'ko','MarkerFaceColor','k');

    % Initialize axes for orientation
    xAxis = line(hAxes,[0 1],[0 0],[0 0],'Color','r','LineWidth',2);
    yAxis = line(hAxes,[0 0],[0 1],[0 0],'Color','g','LineWidth',2);
    zAxis = line(hAxes,[0 0],[0 0],[0 1],'Color','b','LineWidth',2);

    firstRun = true;
end

if ~isa(q,'quaternion')
    warning('Input must be a quaternion object');
    return;
end

% Convert quaternion to rotation matrix
R = rotmat(q,'frame');

% Define local body axes (unit vectors)
origin = [0;0;0];
axesBody = R * eye(3);

% Update axis lines (each column is a rotated axis)
set(xAxis,'XData',[origin(1) axesBody(1,1)], ...
           'YData',[origin(2) axesBody(2,1)], ...
           'ZData',[origin(3) axesBody(3,1)]);
set(yAxis,'XData',[origin(1) axesBody(1,2)], ...
           'YData',[origin(2) axesBody(2,2)], ...
           'ZData',[origin(3) axesBody(3,2)]);
set(zAxis,'XData',[origin(1) axesBody(1,3)], ...
           'YData',[origin(2) axesBody(2,3)], ...
           'ZData',[origin(3) axesBody(3,3)]);

if firstRun
    drawnow;
    firstRun = false;
else
    drawnow limitrate nocallbacks;
end
end
