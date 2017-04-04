% For Linux systems, the included java.opts file must be in the same 
% folder when Matlab starts up. This file allows for the correct serial
% port to be recognized in Matlab.
%% Clean Up:
priorPorts=instrfind;   % find previous serial ports 
delete(priorPorts);     % delete the previous connection
% rosshutdown;          % shuts down ros <debugging>       
clc;                    % clear command terminal
clear;                  % clear workspace
%% Serial Setup:
s=serial('/dev/ttyACM0');           % create serial object (may need to alter port address)
fopen(s);                           % open serial object
s.ReadAsyncMode='continuous';       % set asynchronous reading to continuous
s.BaudRate=115200;                  % set baud rate to 115200
%% ROS Setup:
rosinit;                            % initialize ROS
imu_pub=rospublisher('imu','sensor_msgs/Imu');  % create publisher
msg=rosmessage('sensor_msgs/Imu');              % create Imu message instance
%% Data Initialization:
time=0;                                 % initialize time [ms]
eul=zeros(1,3);                         % prealocate euler array [degrees]
ind=0;                                  % index for covariance sampling
sample_size=10;                         % number of samples to store for covariance calc
QUAT=ones(sample_size,3);               % quaternion covariance sample set
ANG_VEL=ones(sample_size,3);            % angular velocity covariance sample set
ACCEL=ones(sample_size,3);              % linear acceleration covariance sample set
msg.OrientationCovariance=[1;0;0;0;1;0;0;0;1];          % prealocate matrix
msg.AngularVelocityCovariance=[1;0;0;0;1;0;0;0;1];      % prealocate matrix
msg.LinearAccelerationCovariance=[1;0;0;0;1;0;0;0;1];   % prealocate matrix
%% Main Loop:
while 1                     % continuous loop
    data=fscanf(s);         % get serial information
    data=str2num(data);     % convert from string to number
    %disp(data);            % <debugging>
    if length(data)==11                 % redundant check to make sure serial info is correct
        ind=ind+1;                      % increment sample index
        dt=(data(1)-time)/1000;         % change in time for discrete derivative
        time=data(1);                   % update time variable
        % set Orientation values
        msg.Orientation.W=data(5);      
        msg.Orientation.X=data(6);
        msg.Orientation.Y=data(7);
        msg.Orientation.Z=data(8);
        % take sample for orientation covariance calculation
        QUAT(ind,1)=msg.Orientation.X;
        QUAT(ind,2)=msg.Orientation.Y;
        QUAT(ind,3)=msg.Orientation.Z;
        std_quat=std(QUAT);             % calculate quaternion standard variation
        var_quat=std_quat'*std_quat;    % Orientation Covariance calculation
        % set Orientation Covariance values
        msg.OrientationCovariance=var_quat(:);
        % set Angular Velocity values
        msg.AngularVelocity.X=(data(9)-eul(1))*pi./(180*dt);
        msg.AngularVelocity.Y=(data(10)-eul(2))*pi./(180*dt);
        msg.AngularVelocity.Z=(data(11)-eul(3))*pi./(180*dt);
        % take sample for Angular Velocity Covariance calculation
        ANG_VEL(ind,1)=msg.AngularVelocity.X;
        ANG_VEL(ind,2)=msg.AngularVelocity.Y;
        ANG_VEL(ind,3)=msg.AngularVelocity.Z;
        std_ang_vel=std(ANG_VEL);       % calculate Angular Velocity standard deviation
        var_ang_vel=std_ang_vel'*std_ang_vel;   % Angular Velocity Covariance calculation
        % set Angular Velocity Covariance values
        msg.AngularVelocityCovariance=var_ang_vel(:);  
        eul=data(9:11);                 % update euler angle variable
        % set Linear Acceleration values
        msg.LinearAcceleration.X=data(2);
        msg.LinearAcceleration.Y=data(3);
        msg.LinearAcceleration.Z=data(4);
        % take sample for Linear Acceleration Covariance calculation
        ACCEL(ind,1)=msg.LinearAcceleration.X;
        ACCEL(ind,2)=msg.LinearAcceleration.Y;
        ACCEL(ind,3)=msg.LinearAcceleration.Z;
        std_accel=std(ACCEL);           % calculate Linear Acceleration standard deviation
        var_accel=std_accel'*std_accel;     % Linear Acceleration Covariance calculation
        % set Linear Acceleration Covariance values
        msg.LinearAccelerationCovariance=var_accel(:);
        send(imu_pub,msg);          % publish message to topic
        % reset index back to zero cycle
        if ind==sample_size         
            ind=0;
        end
    end
end
%% END
