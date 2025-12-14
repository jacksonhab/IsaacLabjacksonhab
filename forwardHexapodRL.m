%Updated 10/21/2025
%jrh6552@psu.edu
%This file loaded contacts from Di's research to then run on hexapod

%Make a copy if changing anything

%% Initialize and Connect using Dynamixel SDK
lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_bulk_read.h', 'addheader', 'group_bulk_write.h');
end

% Control table address
ADDR.TORQUE_ENABLE              = 64;          % Control table address is different in Dynamixel model
ADDR.LED                        = 65;
ADDR.GOAL_POSITION              = 116;
ADDR.PRESENT_POSITION           = 132;
ADDR.PROFILE_VELOCITY           = 112;

% Data Byte Length
LEN.LED                         = 1;
LEN.GOAL_POSITION               = 4;
LEN.PRESENT_POSITION            = 4;

% Protocol version
PROTOCOL_VERSION                = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
% DOF order used by the robot (index, name):
% 0 BackLink
% 1 FrontLink
% 2 MiddleLeft
% 3 MiddleRight
% 4 BackLeft
% 5 BackRight
% 6 FrontLeft
% 7 FrontRight

DXL_ID_links                    = [6 3];
DXL_ID_legs                     = [5 4 8 7 1 2]; %follows the order from above
%DXL_ID_legs                     = [5 4 1 2 8 7];
%DXL_ID_legs_init                = [1 2 3 4 5 6 7 8]; %follows the order from above

DXL_ID                          = [DXL_ID_links, DXL_ID_legs];


BAUDRATE                        = 57600;
DEVICENAME                      = 'COM9'; %com7 for laptop - com3 for computer  % Check which port is being used on your controller ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'

TORQUE_ENABLE                   = 1;            % Value for enabling the torque
TORQUE_DISABLE                  = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE      = 0;            % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE      = 4095;         % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD     = 20;           % Dynamixel moving status threshold
PROFILE_VELOCITY_links          = 0;           % Profile Velocity
PROFILE_VELOCITY_legs           = 0;

ESC_CHARACTER                   = 'e';          % Key for escaping loop

COMM_SUCCESS                    = 0;            % Communication Success result value
COMM_TX_FAIL                    = -1001;        % Communication Tx Failed

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

% Initialize groupBulkWrite Struct
groupwrite_num = groupBulkWrite(port_num, PROTOCOL_VERSION);

% Initialize Groupbulkread Structs
groupread_num = groupBulkRead(port_num, PROTOCOL_VERSION);

dxl_comm_result = COMM_TX_FAIL;                 % Communication result
dxl_addparam_result = false;                    % AddParam result
dxl_getdata_result = false;                     % GetParam result

dxl_error = 0;                                  % Dynamixel error
dxl_led_value = 0;                              % Dynamixel LED value for write
dxl_present_position = zeros(1,length(DXL_ID)); % Present position
dxl_led_value_read = 0;                         % Dynamixel moving status

% Open port
if (openPort(port_num))
    fprintf('Port Opened Successfully. \n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('clc to terminate...\n');
    return;
end

% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set Successfully. \n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('clc to terminate...\n');
    return;
end

% Enable Dynamixel Torque
for i=1:numel(DXL_ID_links)
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_links(i), ADDR.PROFILE_VELOCITY, PROFILE_VELOCITY_links);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_links(i), ADDR.TORQUE_ENABLE, TORQUE_ENABLE);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    else
        fprintf('Dynamixel [ID:%03d] Torque Enabled. Velocity Set To: %d \n', DXL_ID_links(i),PROFILE_VELOCITY_links);
    end
end

for i=1:numel(DXL_ID_legs)
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_legs(i), ADDR.PROFILE_VELOCITY, PROFILE_VELOCITY_legs);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_legs(i), ADDR.TORQUE_ENABLE, TORQUE_ENABLE);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    else
        fprintf('Dynamixel [ID:%03d] Torque Enabled. Velocity Set To: %d \n', DXL_ID_legs(i),PROFILE_VELOCITY_legs);
    end
end



%% Position Planning

%Calls the .mat file and returns the body angles in encoder values and leg
%contact/no contact in encoder values

%filename = "C:\Users\jrh6552\Hexapod\IsaacLab\Position Files\HexapodRL_Actions_12-8-2025_Obs5_2sec.csv";
%filename = "C:\Users\jrh6552\Hexapod\IsaacLab\Position
%Files\HexapodRL_Rad_12-9-25_O5_10sec.csv"; -- %okay
filename = "C:\Users\habal\OneDrive - The Pennsylvania State University\Fifth year\LiBR Lab\Hexapod\Code\RL Gaits\HexapodRL_Rad_12-13-25_O5_10sec_real.csv"; %-- Best, used for RL project implementation

impData = importdata(filename);
imported_position = impData.data(:,:); %100 is 2 seconds

imported_position_deg = imported_position*(360/(2*pi));
imported_position_enc = imported_position*(4095/(2*pi));

position_legs_enc = imported_position_enc(:,3:end);

position_front_link_enc = imported_position_enc(:,2)*-1;
position_back_link_enc = imported_position_enc(:,1);

position_links_enc = [position_back_link_enc,position_front_link_enc];

position_legs_enc = position_legs_enc*-1 + (4095/2);
position_links_enc = position_links_enc + (4095/2);

position = [position_links_enc, position_legs_enc];
position_deg = position*(360/4095);

trialName = 'Side2Pause016_LLeg30_shift-3';
iterationNum = 1;
%% Send Position
%Number of cycles (each one is 2*pi)
num_cycles = 1;
pause(4);



% Add parameter storage for present position value
for i=1:numel(DXL_ID)
    dxl_addparam_result = groupBulkReadAddParam(groupread_num, DXL_ID(i), ADDR.PRESENT_POSITION, LEN.PRESENT_POSITION);
    if dxl_addparam_result ~= true
        fprintf('[ID:%03d] groupBulkRead addparam failed', DXL_ID(i));
        return;
    end
end

%Start Stopwatch
startTime = tic;

%Loop through the number of cycles
for n = 1:num_cycles

    for j = 1:length(position)
        % Packet 1: All goal positions for Legs
        groupBulkWriteClearParam(groupwrite_num);
        for i=1:numel(DXL_ID)
            %fprintf(typecast(int32(sinpos(t,1,(i-1)*pi,'legs')), 'uint32'))
            dxl_addparam_result = groupBulkWriteAddParam(groupwrite_num, DXL_ID(i), ADDR.GOAL_POSITION, LEN.GOAL_POSITION, typecast(int32(position(j,i)), 'uint32'), LEN.GOAL_POSITION);
            if dxl_addparam_result ~= true
              fprintf('[ID:%03d] groupBulkWrite addparam goal position failed', DXL_ID(i));
              return;
            end
        end
        
        groupBulkWriteTxPacket(groupwrite_num);
    
        groupBulkWriteClearParam(groupwrite_num);
        %{
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        end
    
        % Bulkread present position and moving status
        groupBulkReadTxRxPacket(groupread_num);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        end
        %}
        %Implemented Pause, Keeps the Curve Evenly Spaced so there is no weird
        %angle measurements
        pause(0.01565) %RL steps are in 0.02sec/step -- this is 50 steps/sec - pause of 0.01565 to get as close as possible to the 2sec period
    end
    periodTime = toc;
end

%Track Time
period_s = toc(startTime)/num_cycles;
display(period_s)

% Save it
%logPeriod(trialName, iterationNum, period_s, "DiGaits/period_log.mat");
%iterationNum = iterationNum + 1;

%S = load('DiGaits/period_log.mat','periodLog');
%disp(S.periodLog)          % view in MATLAB

% Post the Hexapod Up
pause(3)

dxl_goal_position = [2048 2048 2048+300 2048+300 2048+300 2048+300 2048+300 2048+300];
for i=1:numel(DXL_ID)
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID(i), ADDR.GOAL_POSITION, typecast(int32(dxl_goal_position(i)), 'uint32'));
end


%% Post the Hexapod Up
dxl_goal_position = [2048 2048 2048+300 2048+300 2048+300 2048+300 2048+300 2048+300];
for i=1:numel(DXL_ID)
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID(i), ADDR.GOAL_POSITION, typecast(int32(dxl_goal_position(i)), 'uint32'));
end

%% Close Port and Disable Torque
for i=1:numel(DXL_ID)
    % Disable Dynamixel Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID(i), ADDR.TORQUE_ENABLE, TORQUE_DISABLE);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
end

% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);
fprintf('Successfully Unloaded and Closed')
close all;
clear;