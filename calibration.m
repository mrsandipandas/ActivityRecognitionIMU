clear, clc, close all

startup()
showIP()

[xhat, meas] = filterTemplate();

mean_acc = mean(meas.acc(:,~any(isnan(meas.acc),1)),2);
mean_gyr = mean(meas.gyr(:,~any(isnan(meas.gyr),1)),2);
mean_mag = mean(meas.mag(:,~any(isnan(meas.mag),1)),2);

set(groot, 'DefaultTextInterpreter', 'latex')
set(groot, 'DefaultLegendInterpreter', 'latex')
set(groot,'DefaultAxesFontSize',12)

for i=1:3
    cov_acc(i) = cov(meas.acc(i,~any(isnan(meas.acc),1)));
    cov_gyr(i) = cov(meas.gyr(i,~any(isnan(meas.gyr),1)));
    cov_mag(i) = cov(meas.mag(i,~any(isnan(meas.mag),1)));
end


cov_vec = {cov_acc,cov_gyr,cov_mag};
to_plot = {meas.acc-mean_acc, meas.gyr-mean_gyr, meas.mag-mean_mag};
sensor = {'Accelorometer','Gyroscope','Magnetometer'};
ax = {'X','Y','Z'};

bins = [150,150,150];

xlimits = [0.04, 6e-3, 2];
ylimits = [600 1600 1000];
cm = colormap(lines);


for i=1:3
    figure(i)
    % For each axis
    for j=1:3
        subplot(3,1,j)
        h = histogram(to_plot{i}(j,:), bins(i),'Normalization','count','FaceAlpha',1,'FaceColor',cm(j,:));
        
        legend(['$' ax{j} '$'])
        xlim([-xlimits(i) xlimits(i)]),ylim([0 ylimits(i)])
        
        if j==1
            title(['$Histogram \ bins \ of \ ' sensor{i} '$'])
        end
        if j==2
            ylabel('$Frequency$')
        end
        if j==3
            xlabel('$Data \ spread$')
        end
    end
end