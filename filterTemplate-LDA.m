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
load dataset.mat
MdlLinear = fitcdiscr(meas,tag);
clear meas tag
measvec_prev = [];
%% Filter settings
t0 = [];  % Initial time (initialize on first data received)
nx = 4;
% Add your filter settings here.
window = 100; % No. of historical points for activity calulation
orientation_del_history = zeros(window,1);
orientation_old = [0 0 0 0]';
activity = ["Stationary" "Walking" "Running"];
thershold = [5e-3       20e-3];
initialized = 0;

% true = Phone calculated Q, provided by android
% false = Developed algo calculated Q
orientation_type = true;

% Offline settings without camera recorded data
offline = false;
data_size = 0;

correct_cnt = 0;
total_cnt = 0;
%% Camera for VINS
try
    % Get the plugin connector from here Home->Env->Add-Ons
    % https://www.mathworks.com/matlabcentral/fileexchange/63319-android-mobile-camera-connector
    
    % true|false = Use|Stop camera
    camera = false;
    
    % Show one frame per 'cam_window' frame streamed
    cam_window = 100;
        
    % 1 = Default view
    % 2 = Activity view
    % 3 = Camera View in the main window
    plot_size = 2;
    
    % integrated = Display in one window
    % float = Display as a seperate window
    % None = No display but use camera features for calculations
    camera_view = 'integrated';
    
    % Change the IP based on IP display of the Webcam app
    % https://ip-webcam.appspot.com/
    camera_url = ('http://192.168.1.215:8080/shot.jpg?rnd=350264');
    if camera
        if strcmp(camera_view,'float')
            video=vision.VideoPlayer();  
        elseif strcmp(camera_view,'integrated')
            plot_size = 3;
            cam_handle = imshow(imread(camera_url), 'XData',[1 12], 'YData',[1 3], 'Parent', subplot(plot_size, 2, [5,6]));
        else
            fprintf('Activated camera stream in background!\n');
        end
    else
        fprintf('Not using camera!\n');
    end
catch e
    fprintf(['Unsuccessful connecting to IP Cam!\n' ...
        'Make sure to start streaming from the phone *before*\n'...
        'running this function and activate stream in background.\n'...
        'Or, simply turn off camera usage using camera = flase flag!\n']);
    return;
end
%% Current filter state.
x = [1; 0; 0 ;0];
P = eye(nx, nx);

%% Saved filter states.
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
    if offline
        record = load('data.mat');
        data_size = size(record.sample,1);
    else
        server.start();  % Start data reception.
    end
catch e
    fprintf(['Unsuccessful connecting to client!\n' ...
        'Make sure to start streaming from the phone *after* '...
        'running this function!']);
    return;
end

% Used for visualization.
figure(1);
subplot(plot_size, 2, 1);
ownView = OrientationView('Own filter', gca);  % Used for visualization.
googleView = [];
counter = 0;  % Used to throttle the displayed frame rate.

meas_struct = [];

%% Filter loop
while server.status() || (counter < data_size) % Repeat while data is available
    % Get the next measurement set, assume all measurements
    % within the next 5 ms are concurrent (suitable for sampling
    % in 100Hz).
    if offline
        data = record.sample(counter+1,:);
    else
        data = server.getNext(5);
    end
    
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
    

    
    if camera && (counter > 0 && rem(counter, cam_window) == 0) 
        img = imread(camera_url);
        if ~any(isnan(img))  % Camera image are available.
            % Do something
        end
    end
    
    orientation = data(1, 18:21)';  % Google's orientation estimate.
    if isempty(measvec_prev)
        measvec = [t, orientation', acc', gyr'];;
    end
    % Store data in .mat
    measvec_prev = measvec;
    measvec = [t, orientation', acc', gyr'];
    
    % Visualize result
    if rem(counter, 10) == 0
        setOrientation(ownView, x(1:4));
        title(ownView, 'OWN', 'FontSize', 16);
        if ~any(isnan(orientation))
            if isempty(googleView)
                subplot(plot_size, 2, 2);
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
        subplot(plot_size, 2, [3,4]);
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
%             stationary = sum(orientation_del_history <= thershold(1));
%             walk = sum(orientation_del_history > thershold(1) & orientation_del_history < thershold(2));
%             run = window - (stationary + walk);
%             category = [stationary walk run];
%             [~, argmax] = max(category);
            
            if isnan(measvec)*ones(11,1) == 0
                % Get time delta
                t_before = measvec_prev(1);
                t_curr = measvec(1);
                dt = t_curr - t_before;

                % Get lin. velocity 
                accel_before = measvec_prev(6:8);
                accel_curr = measvec(6:8);
                lin_vel = (accel_curr - accel_before)/dt;

                % Get current data 
                q = measvec(2:5);
                gyro = measvec(9:11);

                % Build proc data
                proc_data_vec = [q, lin_vel, gyro, accel_curr];
                currAct = predict(MdlLinear, proc_data_vec);
                if strcmp(currAct{1},'walking')
                    correct_cnt = correct_cnt+1;
                end
                set(get(gca, 'title'), 'string', currAct{1})
                total_cnt = total_cnt + 1;
                fprintf('Total count: %d  - Correct count: %d \n',total_cnt, correct_cnt);
            end
            
        end
        del = 2*acos(abs(dot(orientation_old,val)));
        orientation_old = val;
        orientation_del_history(1+rem(counter, window)) = real(del);
        
        if camera && (counter > 0 && rem(counter, cam_window) == 0)                       
            if strcmp(camera_view,'float')
                step(video,img);
            elseif strcmp(camera_view,'integrated')
                subplot(plot_size, 2, [5,6]);
                set(cam_handle,'CData', img);
                drawnow;
            else
                fprintf('Activated camera stream in background!\n');
            end
        end
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
