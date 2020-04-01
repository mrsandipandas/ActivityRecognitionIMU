filename = 'sitted';
ld_file = [filename,'.mat'];
load(filename)
proc_data = [];

for i=2:size(meas_struct,1)
   
   % there are no NANs
   if isnan(meas_struct(i,:))*ones(11,1) == 0
       % Get time delta
       t_before = meas_struct(i-1,1);
       t_curr = meas_struct(i,1);
       dt = t_curr - t_before;
       
       % Get lin. velocity 
       accel_before = meas_struct(i-1,6:8);
       accel_curr = meas_struct(i,6:8);
       lin_vel = (accel_curr - accel_before)*dt;
       
       % Get current data 
       q = meas_struct(i,2:5);
       gyro = meas_struct(i,9:11);
       
       % Build proc data
       proc_data_vec = [q, lin_vel, gyro, accel_curr];
       proc_data = [proc_data; proc_data_vec];
   end
end
label = cell(size(proc_data,1),1);
label(:) = {filename};

save([filename,'_proc.mat'],'proc_data','label');