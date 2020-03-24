function [xhat, meas] = filterTemplate(calAcc, calGyr, calMag)
% FILTERTEMPLATE  Filter template
%
% This is a template function for how to collect and filter data
% sent from a smartphone live.  Calibration data for the
% accelerometer, gyroscope and magnetometer assumed available as
% structs with fields m (mean) and R (variance).
%
% The function returns xhat as an array of structs comprising t
% (timestamp), x (state), and P (state covariance) for each
% timestamp, and meas an array of structs comprising t (timestamp),
% acc (accelerometer measurements), gyr (gyroscope measurements),
% mag (magnetometer measurements), and orint (orientation quaternions
% from the phone).  Measurements not availabe are marked with NaNs.
%
% As you implement your own orientation estimate, it will be
% visualized in a simple illustration.  If the orientation estimate
% is checked in the Sensor Fusion app, it will be displayed in a
% separate view.

%% Setup necessary infrastructure
import('se.hendeby.sensordata.*');  % Used to receive data.

%% Filter settings
t0 = [];  % Initial time (initialize on first data received)
nx = 4;
% Add your filter settings here.
window = 100;
orientation_del_history = zeros(window,1);
orientation_old = [0 0 0 0]';
activity = ["Stationary" "Walking" "Running"];
thershold = [5e-3       20e-3];
initialized = 0;
% true = Phone calculated Q, provided by android
% false = Developed algo calculated Q
orientation_type = true; 

% Current filter state.
x = [1; 0; 0 ;0];
P = eye(nx, nx);

% Saved filter states.
xhat = struct('t', zeros(1, 0),...
    'x', zeros(nx, 0),...
    'P', zeros(nx, nx, 0));

meas = struct('t', zeros(1, 0),...
    'acc', zeros(3, 0),...
    'gyr', zeros(3, 0),...
    'mag', zeros(3, 0),...
    'orient', zeros(4, 0));
try
    %% Create data link
    server = StreamSensorDataReader(3400);
    % Makes sure to resources are returned.
    sentinel = onCleanup(@() server.stop());
    
    server.start();  % Start data reception.
catch e
    fprintf(['Unsuccessful connecting to client!\n' ...
        'Make sure to start streaming from the phone *after* '...
        'running this function!']);
    return;
end

% Used for visualization.
figure(1);
subplot(2, 2, 1);
ownView = OrientationView('Own filter', gca);  % Used for visualization.
googleView = [];
counter = 0;  % Used to throttle the displayed frame rate.

%% Filter loop
while server.status()  % Repeat while data is available
    % Get the next measurement set, assume all measurements
    % within the next 5 ms are concurrent (suitable for sampling
    % in 100Hz).
    data = server.getNext(5);
    
    if isnan(data(1))  % No new data received
        continue;
    end
    t = data(1)/1000;  % Extract current time
    
    if isempty(t0)  % Initialize t0
        t0 = t;
    end
    
    gyr = data(1, 5:7)';
    if ~any(isnan(gyr))  % Gyro measurements are available.
        % Do something
    end
    
    acc = data(1, 2:4)';
    if ~any(isnan(acc))  % Acc measurements are available.
        % Do something
    end
    
    mag = data(1, 8:10)';
    if ~any(isnan(mag))  % Mag measurements are available.
        % Do something
    end
    
    orientation = data(1, 18:21)';  % Google's orientation estimate.
    
    % Visualize result
    if rem(counter, 10) == 0
        setOrientation(ownView, x(1:4));
        title(ownView, 'OWN', 'FontSize', 16);
        if ~any(isnan(orientation))
            if isempty(googleView)
                subplot(2, 2, 2);
                % Used for visualization.
                googleView = OrientationView('Google filter', gca);
            end
            setOrientation(googleView, orientation);
            title(googleView, 'GOOGLE', 'FontSize', 16);
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%% Visualize quartenion average delta %%%%%%%%%%%%%%%%%%%%
    if orientation_type
        val = orientation;
    else
        val = x(1:4)';
    end
    if ~any(isnan(val))
        subplot(2, 2, [3,4]);
        if initialized == 0
            plot(t, 0);
            initialized = 1;
        end
            if counter > 0 && rem(counter, window) == 0     
                del_avg = (1.0/window)*sum(orientation_del_history);
                ax = gca(); % get handle of current axes;
                line = get(ax, 'Children'); % get handle to line object
                line.XData = [line.XData t];
                line.YData = [line.YData del_avg];
                
                % Majority voting
                stationary = sum(orientation_del_history <= thershold(1), 'all');
                walk = sum(orientation_del_history > thershold(1) & orientation_del_history < thershold(2), 'all');
                run = window - (stationary + walk);
                category = [stationary walk run];
                [~, argmax] = max(category);
                set(get(gca, 'title'), 'string', activity(argmax))

            end
            del = 2*acos(abs(dot(orientation_old,val)));
            orientation_old = val;
            orientation_del_history(1+rem(counter, window)) = real(del);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    counter = counter + 1;
    % Save estimates
    xhat.x(:, end+1) = x;
    xhat.P(:, :, end+1) = P;
    xhat.t(end+1) = t - t0;
    
    meas.t(end+1) = t - t0;
    meas.acc(:, end+1) = acc;
    meas.gyr(:, end+1) = gyr;
    meas.mag(:, end+1) = mag;
    meas.orient(:, end+1) = orientation;
end
end
