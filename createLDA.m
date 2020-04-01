meas = [];
tag = [];

load walking_proc
meas = [meas; proc_data];
tag = [tag; label];
load sitted_proc
meas = [meas; proc_data];
tag = [tag; label];
load standing_proc
meas = [meas; proc_data];
tag = [tag; label];

save('dataset.mat','meas','tag');

MdlLinear = fitcdiscr(meas,tag);
meanmeas = mean(meas);
meanclass = predict(MdlLinear,meas(4000,:))