function [xhat, meas] = filterTemplate(calAcc, calGyr, calMag)

startup()
showIP()

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
% Samsung Note 10+ Södertälje
Pos = [0 0 0]';

acc_base = [-0.1102 0.1214 9.7468]';
cov_acc = diag([.08438e-03 .08831e-03 2.50e-3]); % [m/s^2]

cov_gyro = diag([1.669e-06 1.402e-06 1.419e-06]); % [rad/s]

% mag_base = [0 sqrt(6.372^2+11.725^2) -42.427]';
mag_base = [-11.725 6.372 -42.427]';
mag_norm = norm(mag_base);
cov_mag = diag([0.1389 0.10001 0.1852]);
alpha = 0.01;

window = 100; % No. of historical points for activity calulation
orientation_old = [0 0 0 0]';
activity = ["Stationary" "Walking" "Running"];
thershold = [0.15          1.0];
activity_initialized = 0;
img_initialized = 0;

% true = Phone calculated Q, provided by android
% false = Developed algo calculated Q
orientation_type = false;

% Offline settings without camera recorded data
offline = true;
data_size = 0;

%% Camera for feature tracking to incorporate VINS
try
    % Get the plugin connector from here Home->Env->Add-Ons
    % https://www.mathworks.com/matlabcentral/fileexchange/63319-android-mobile-camera-connector
    
    % true|false = Use|Stop camera
    camera = false;
    
    % 2 = Default view
    % 4 = Activity view
    % 6 = 3D Position view
    % 8 = Camera View in the main window
    plot_size = 6;
    
    % Show one frame per 'cam_window' frame streamed
    cam_window = 100;
    
    % integrated = Display in one window
    % float = Display as a seperate window
    % None = No display but use camera features for calculations
    camera_view = 'None';
    
    
    
    % Change the IP based on IP display of the Webcam app
    % https://ip-webcam.appspot.com/
    camera_url = ('http://192.168.1.179:8080/shot.jpg?rnd=350264');
    if camera
        if strcmp(camera_view,'float')
            video=vision.VideoPlayer();
        elseif strcmp(camera_view,'integrated')
            plot_size = 8;
            % 'XData',[1 12], 'YData',[1 3] Sets the size of the image
            cam_handle = imshow(imread(camera_url), 'XData',[1 12], 'YData',[1 3], 'Parent', subplot(plot_size, 2, [13 14 15 16]));
        else
            fprintf('Activated camera stream in background!\n');
        end
    else
        fprintf('Not using camera!\n');
    end
catch e
    fprintf(['Unsuccessful connecting to IP Cam!\n' ...
        'Make sure to start streaming from the phone *before* running this function. \n'...
        'If needed please activate stream in background and also check for the correct *IP*.\n'...
        'Or, simply turn off camera usage using *camera = false* flag!\n']);
    return;
end
%% Current filter state.
x = [1; 0; 0 ;0];
P = eye(nx, nx);

%% Saved filter states.
xhat = struct('t', zeros(nx, 0),...
    'x', zeros(nx, 0),...
    'P', zeros(nx, nx, 0));

meas = struct('t', zeros(1, 0),...
    'acc', zeros(3, 1),...
    'gyr', zeros(3, 1),...
    'mag', zeros(3, 1),...
    'orient', zeros(4, 1),...
    'orient_del', zeros(1, 0),...
    'Pos', zeros(3, 1),...
    'img', zeros(1080, 1920, 3));

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
subplot(plot_size, 2, [9 10 11 12]);
postion_graph = plot3(Pos(1), Pos(2), Pos(3),'-b', Pos(1), Pos(2), Pos(3), '-or','MarkerSize',10,'MarkerFaceColor','r');
% xlim([-2 2])
% ylim([-2 2])
% zlim([-2 2])
xlabel('Position X')
ylabel('Position Y')
zlabel('Position Z')
grid on

subplot(plot_size, 2, [1 3]);
ownView = OrientationView('Own filter', gca);  % Used for visualization.
googleView = [];
counter = 0;  % Used to throttle the displayed frame rate.

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
    if ~size(meas.t,2)
        prv_t = 0;
    else
        prv_t = meas.t(:,end);
    end
    gyr = data(1, 5:7)';
    if ~any(isnan(gyr))  % Gyro measurements are available.
        [x, P] = update_gyro(x, P, t-t0-prv_t, cov_gyro^2*eye(3), gyr);
    else
        [x, P] = update_gyro(x, P, t-t0-prv_t, cov_gyro^2*eye(3));
    end
    
    acc = data(1, 2:4)';
    if ~any(isnan(acc))  % Acc measurements are available.
        if  norm(acc)<9.81*1.6 && norm(acc)>9.81*0.8
            [x, P] = update_acc(x, P, acc, cov_acc, acc_base); % Update state estimate
        end
    end
    
    mag = data(1, 8:10)';
    if ~any(isnan(mag))  % Mag measurements are available.
        % Low pass-filter to account for that the magnitude of m0 might drift
        mag_norm = (1-alpha)*mag_norm + alpha*norm(mag);
        
        % If magnitude of measurement is too large, skip update step
        if 32<mag_norm && mag_norm<56 % Thresholds for magnetic field
            [x, P] = update_mag(x, P, mag, mag_base, cov_mag); % Update state estimate
        end
    end
    
    if camera  && (counter > 0 && rem(counter, cam_window) == 0)
        img = imread(camera_url);
        if ~any(isnan(img))  % Camera image are available.
            % Do something
        end
    end
    
    orientation = data(1, 18:21)';  % Google's orientation estimate.
    
    % Visualize result
    if rem(counter, 10) == 0
        setOrientation(ownView, x(1:4));
        title(ownView, 'OWN', 'FontSize', 16);
        if ~any(isnan(orientation))
            if isempty(googleView)
                subplot(plot_size, 2, [2 4]);
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
        val = x(1:4)'; % Self-calculated orientation
    end
    
    if ~any(isnan(val))
        subplot(plot_size, 2, [5 6 7 8]);
        if activity_initialized == 0
            plot(t, 0);
            xlabel('Time')
            ylabel('Attitude delta')
            grid on
            activity_initialized = 1;
        end
        if counter > 0 && rem(counter, window) == 0
            history_orient_del = meas.orient_del(end-window+1:end);
            orient_del_avg = (1.0/window)*sum(history_orient_del);
            ax = gca(); % get handle of current axes;
            line = get(ax, 'Children'); % get handle to line object
            line.XData = [line.XData t];
            line.YData = [line.YData orient_del_avg];
            
            % Majority voting
            stationary = sum(history_orient_del <= thershold(1));
            walk = sum(history_orient_del > thershold(1) & history_orient_del < thershold(2));
            run = window - (stationary + walk);
            category = [stationary walk run];
            [~, argmax] = max(category);
            set(get(gca, 'title'), 'string', activity(argmax))
            
        end
        % Difference between quartenions
        % http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf
        if ~any(orientation_old(:))
            del = real(2*acos(val(1)));
        else
            Q_del = quatmultiply(quatconj(val), orientation_old);
            del = real(2*acosd(Q_del(1)));
        end
        
        if counter > 0 && rem(counter, window/10) == 0
            history_position = meas.Pos(:,end-1:end);
            position_avg = mean(history_position, 2);
            
            set(postion_graph, {'XData'}, {[postion_graph.XData position_avg(1)]; position_avg(1)});
            set(postion_graph, {'YData'}, {[postion_graph.YData position_avg(2)]; position_avg(2)});
            set(postion_graph, {'ZData'}, {[postion_graph.ZData position_avg(3)]; position_avg(3)});
        end
        
        orientation_old = val;
        
        if camera && (counter > 0 && rem(counter, cam_window) == 0)
            if img_initialized
                
                try
                    tic
                    corners = detectFASTFeatures(rgb2gray(meas.img));
                    img_corners = insertMarker(meas.img, corners.selectStrongest(100).Location, '+','color','green','size',10);
                    
                    tracker = vision.PointTracker('MaxBidirectionalError', 3, 'NumPyramidLevels', 5);
                    initialize(tracker, corners.Location, meas.img);
                    
                    [new_corners, validIdx] = step(tracker, img);
                    matchedPoints1 = corners.Location(validIdx, :);
                    matchedPoints2 = new_corners(validIdx, :);
                    
                    % tform =  estimateGeometricTransform(matchedPoints2,matchedPoints1,'similarity')
                    tform = fitgeotrans(matchedPoints2,matchedPoints1,'nonreflectivesimilarity')
                    
                    toc
                    if strcmp(camera_view,'float')
                        step(video,img_corners);
                    elseif strcmp(camera_view,'integrated')
                        subplot(plot_size, 2, [13 14 15 16]);
                        set(cam_handle,'CData', img_corners);
                        drawnow;
                    else
                        fprintf('Activated camera stream in background!\n');
                    end
                catch e
                    fprintf(['No transformations\n']);
                    meas.img = img;
                end
                
                
            else
                meas.img = img;
                img_initialized = 1;
            end
        end
    else
        del = 0;
    end
    
    % Calculate position
    if ~any(isnan(acc))
        acc_del = acc-Qq(x)'*acc_base;
        if del > thershold(1)
            vel = 1.4;
            Pos = Pos(:,end) + 0.5*acc_del*(t-t0-prv_t)^2;
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    counter = counter + 1;
    % Save estimates
    xhat.x = [xhat.x, x];
    xhat.P = cat(3,xhat.P, P);
    xhat.t = [xhat.t, (t-t0)];
    
    meas.t = [meas.t, (t-t0)];
    meas.acc = [meas.acc, acc];
    meas.gyr = [meas.gyr, gyr];
    meas.mag = [meas.mag, mag];
    meas.orient = [meas.orient, orientation];
    meas.orient_del = [meas.orient_del, del];
    meas.Pos = [meas.Pos, Pos];
end
end
