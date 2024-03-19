clear all; clc; close all;
% Determine where your m-file's folder is.
folder = fileparts(which(mfilename)); 
% Add that folder plus all subfolders to the path.
addpath(genpath(folder));

data_names = dir("iphoneXR_gforce/*.csv");

for i=1:length(data_names)
    
    imu.(data_names(i).name(1:end-4)) = readmatrix("iphoneXR_gforce/" + data_names(i).name);

end

ori = fieldnames(imu);


for i = 1:length(ori)
    figure(1)
    subplot(length(ori),1,i)

    current_data = imu.(ori{i});
    
    for axis = 2:4
        hold on
        plot(current_data(:,1),current_data(:,axis))
        grid on
        xlim([5,8])
    end
    title(ori{i})
    legend("X","Y","Z")
    hold off

    [r,p] = roll_and_pitch(current_data(:,2:4));

    figure(2)

    subplot(length(ori),1,i)

    hold on
    plot(current_data(:,1),r)
    plot(current_data(:,1),p);
    xlim([5,8])
    grid on
    title(ori{i})
    legend("Roll","Pitch")
    hold off


    yaw = (atan2(-current_data(:,2),current_data(:,3)))*180/pi();

end
%% adding label elements to the resultant figures
gforce_plot = axes(figure(1),'Visible','off');
gforce_plot.XLabel.Visible='on';
gforce_plot.YLabel.Visible='on';
xlabel(gforce_plot,"Time (s)")
ylabel(gforce_plot,"Gforce")

rot_plot = axes(figure(2),'Visible','off');
rot_plot.XLabel.Visible='on';
rot_plot.YLabel.Visible='on';
xlabel(rot_plot,"Time (s)")
ylabel(rot_plot,"Degrees ^0")

%% Get data for Q5 for this part!


%% Part 2 Pedestrian data
load('pedestrian.mat')
% row data of the matrix
% timestamp
% 2-4 specific forces x,y,z respectively
% 5-7 angular rates about x,y,z in deg/s
% 8-10: magnetic field (x,y,z) in uT

%first we can write a plotting function to ingest the data and visualize it
%for us for analysis

pedestrian_plotter(marg4A)

% Initial visualization of the data shows a clear trend for steps
% particularly in the Z and Y axis of the phone.

% since the known stride length if .75m, to walk 4.5m, we know
% approximately 6 steps must be taken, and the data clearly suggests 6
% distinct peaks that occur in the data.

% since the z and y axis are the ones that are moving more up and down
% relative to the ground, it makes sense that the acceleration in the
% perpinduclar axis to them ei the X axis.

%the value we are looking for is the moment the step peaks in acceleration
%in around the time we expect a step to occur, as during the step the
%acceleration will drop back downas its in a psuedo free fall state, our
%acceleration will be based on the initial jump.

% For ease of analysis we will simply find this peak visually with the
% graphs to find our approximate x acceleration and see if it gets us close
% to the stride length we expect. from marg4A, the first step peaks at
% about time 16.021, and goes until about 16.729, but its initial peak is
% about 2.851

t = marg4A(1,:)
step_idx = find(t==16.021)
x = marg4A(2,:);
accel = x(step_idx)
%time bettttween steps was about .7s
%using constant accel
% x = x0 + vot + .5 * at^2 where x0 and v0 are 0
% so x = .5 at^2
displacement = .55^2*accel
%%
% to chart out the motion through the air, we need to fix our frame of
% reference. This can be done simply by using the initial rotation values
% as our "forward" direction, and calculate any deviations in rotation and
% direction from that position to keep our position straight.
pos = dead_pedestrian(marg4A);
figure(3)
scatter3(pos.disp(1,:),pos.disp(2,:),pos.disp(3,:))

function pos = dead_pedestrian(data)
    t = data(1,:);
    accel = data(2:4,:);
    gyro = data(5:7,:);
    mag = data(8:10,:);

    fixed_orientation = gyro(:,1);
    % since what we really care about is the difference between each data
    % point we can calculate the remaining arrays as such
    diff_data = diff(data,1,2);

    t_diff = diff_data(1,:);
    accel_diff = diff_data(2:4,:);
    gyro_diff = diff_data(5:7,:);
    mag_diff = diff_data(8:10,:);

    vo = [0;0;0]; % velocity vector
    do = [0;0;0]; % displacement vector

    v=[];
    d=[0;0;0];

    for t = 1:length(t_diff)
        % since time is directly related to the error as well, we will
        % check to see the difference in acceleration, and not do any
        % computations until we know we have started moving, we can do this
        % by simplye threshold if statement
        if all(accel_diff(:,t) < 0.05)
            v = [v, [0;0;0]];
            d = [d,do];
            continue
        end

        % since the orientation of the gyro is going to be changing from
        % our initial fixed position, we need to find the change in
        % rotation to correct our velocity directions back to the fixed
        % frame of reference. this can be done by taking the velocity
        % vector and correcting it with the gyro data
     

        gyro_correction = fixed_orientation-gyro(:,t+1);
        %gyro_correction = gyro_diff(:,t)
        R = rz(gyro_correction(3,:)) * ry(gyro_correction(2,:)) * rx(gyro_correction(1,:)) ;
        % with this rotation matrix we can correct our acceleration vectors
        % to be in the orientation of our initial fixed orientation, and
        % calculate the resulting velocity and position from that fixed
        % position, first we need to get the cross product of the
        % acceleration
        corr_accel = R * accel_diff(:,t);

        % the time we integrate over will simply by t_diff(:,t) for each
        % iteration
        % we can calulate the velocity
        v = [v, vo + corr_accel .*t_diff(:,t)];

        %then the position
        d = [d, do + vo .* t_diff(:,t) + .5 .* corr_accel .* t_diff(:,t).^2];

        %setting our new initial values for the next iteration
        vo = v(:,t);
        do = d(:,t);

    end
    pos.vel = v;
    pos.disp = d;



end

function fig = pedestrian_plotter(data)
    t = data(1,:);
    accel = data(2:4,:);
    gyro = data(5:7,:);
    mag = data(8:10,:);
    
    fig = figure;

    subplot(3,1,1)
    plot(t,accel)
    title("Specific Force m/s")
    legend("X","Y","Z")

    subplot(3,1,2)
    plot(t,gyro)
    title("Rotation")
    legend("X","Y","Z")

    subplot(3,1,3)
    plot(t,mag)
    title("Magnetic")
    xlabel("Time (s)")
    legend("X","Y","Z")

end 
