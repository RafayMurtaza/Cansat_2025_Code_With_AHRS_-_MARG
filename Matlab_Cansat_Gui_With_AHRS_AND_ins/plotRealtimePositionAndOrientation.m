function plotRealtimePositionAndOrientation(p, q)
% Real-time 3D visualization using position and quaternion
% p = [x y z] position (1x3)
% q = quaternion (1x1 or 1x4) orientation

persistent fig ax hTraj hDir firstRun

if isempty(fig) || ~isvalid(fig)
    fig = figure('Name','3D Position & Orientation Viewer','NumberTitle','off');
    ax = axes('Parent', fig);
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
    view(45,30);
    title(ax, 'Position & Orientation');

    hTraj = plot3(ax, NaN, NaN, NaN, 'b-', 'LineWidth', 1.5);
    hDir  = quiver3(ax, 0,0,0,0,0,0, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
    firstRun = true;
end

% --- Convert quaternion to rotation matrix ---
if isa(q, 'quaternion')
    R = rotmat(q, 'frame');
else
    R = quat2rotm(q); % for numeric [w x y z] format
end

% Update trajectory
set(hTraj, 'XData', [get(hTraj,'XData') p(1)], ...
           'YData', [get(hTraj,'YData') p(2)], ...
           'ZData', [get(hTraj,'ZData') p(3)]);

% Draw orientation (red arrow shows forward direction, X-axis)
dirVec = R(:,1)' * 0.5; % scale arrow
set(hDir, 'XData', p(1), 'YData', p(2), 'ZData', p(3), ...
          'UData', dirVec(1), 'VData', dirVec(2), 'WData', dirVec(3));

drawnow limitrate nocallbacks;
