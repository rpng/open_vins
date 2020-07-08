
% clear and close old
close all;
clear all;

% our data file
path_main = '../';
filename_in = 'raw/urban39_global_pose.csv';
filename_out = 'urban39';

%% Vehicle2IMU.txt
% IMU extrinsic calibration parameter from vehicle
% The groundtruth poses are in the vehicle frame while we need the imu
p_IinV = [-0.07; 0; 1.7];
R_ItoV = [1 0 0; 0 1 0; 0 0 1];
T_VtoI = [...
    R_ItoV' -R_ItoV'*p_IinV;
    0 0 0 1
];



%% Load the groundtruth file
disp('Loading groundtruth...')
data_e = {};
data_e{length(data_e)+1} = importdata([path_main,filename_in],',',1);
temp_data = data_e{end}.data;
data_e{length(data_e)}.data = zeros(size(temp_data,1),8);
for jj=1:size(temp_data,1)
    % timestamp is in nanoseconds so convert to our seconds
    data_e{length(data_e)}.data(jj,1) = 1e-9*temp_data(jj,1);
    % next get our transform
    T_VtoUTM = [temp_data(jj,2:5); temp_data(jj,6:9); temp_data(jj,10:13); 0 0 0 1];
    T_ItoUTM = T_VtoUTM*inv(T_VtoI);
    % store in our vector
    data_e{length(data_e)}.data(jj,2:4) = T_ItoUTM(1:3,4)';
    data_e{length(data_e)}.data(jj,5:8) = rot2quat(T_ItoUTM(1:3,1:3)');
end



% finally write groundtruth data
fprintf('Outputing groundtruth (RPE format)...\n')
filename = [path_main,filename_out,'.txt'];
file = fopen([filename], 'w');
for t=1:size(data_e{end}.data(:,1:8),1)
    fprintf(file, '%0.9f %0.6f %0.6f %0.6f %0.8f %0.8f %0.8f %0.8f\n',data_e{end}.data(t,1:8));
end
fprintf('   + saved %s\n',filename)


% finally write groundtruth data
fprintf('Outputing groundtruth (ETH format)...\n')
filename = [path_main,filename_out,'.csv'];
file = fopen([filename], 'w');
for t=1:size(data_e{end}.data(:,1:8),1)
     fprintf(file, '%0.9f,%0.6f,%0.6f,%0.6f,%0.8f,%0.8f,%0.8f,%0.8f\n',1e9*data_e{end}.data(t,1),data_e{end}.data(t,2:4),data_e{end}.data(t,8),data_e{end}.data(t,5:7));
end
fprintf('   + saved %s\n',filename)

