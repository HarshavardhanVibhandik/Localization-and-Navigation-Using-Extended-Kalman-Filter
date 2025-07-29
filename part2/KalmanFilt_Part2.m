clear; % Clear variables
datasetNum = 9; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM % It specifies the dataset number to be used
[sampledData, sampledVicon, sampledTime] = init(datasetNum); % sampledData: resampled sensor data collected from IMU %sampledVicon: resampled Vicon data which includes info about Velocity, position, orientation % sampledTime: timestamps corresponding to each sample in sampledData and sampledVicon  
Z = sampledVicon(7:9,:);%all the measurements that you need for the update % It selects the first 6 rows of sampledVicon
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state % Copy the Vicon Initial state % Vector that will contain the irst 9 rows of sampledVicon and the zero vector with length 6
covarPrev = eye(15); % Covariance constant %It initilizes the covariance matrix to be an Identity matrix of size 15x15
savedStates = zeros(15, length(sampledTime)); % Just for saving state his. %Matrix with 15 rows and number of columns = lenght of sampledTime
prevTime = 0; %last time step in real time %Variable to store the previous time step which will be used to track the time progression 

%write your code here calling the pred_step.m and upd_step.m functions
for i = 1:length(sampledTime) % This is For loop that iterates over each time step in the sampledTime
    dt  = sampledTime(i) - prevTime; % To calculate time step %Substract previous time from the current time
    angVel= sampledData(i).omg; %To retrive the angular velocity from IMU data
    acc = sampledData(i).acc; %To retrive linear acceleration from IMU data
    currTime = sampledTime(i); %To extract the current timestamp from the sampleddata
    prevTime = currTime; %To update the previous time with the current
    z_t = Z(:,i); %Retrive measurement data at current time step

    % The Prediction step
    [covarEst,uEst] =  pred_step(uPrev,covarPrev,angVel,acc,dt);

    % The Update step
    [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst);

    uPrev = uCurr; % Update the preious state for the next iteration
    covarPrev = covar_curr; % Update the previous covariance for the next iteration
    
    savedStates(:,i) = uCurr; %Save current state at time step i
    

end
plotData(savedStates, sampledTime, sampledVicon, 2, datasetNum);