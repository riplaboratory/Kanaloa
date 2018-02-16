% Send motor commands to Arduino Mega via ROS.
% Based from Stationkeeping/stationkeeping_April19.m

%{
    Setup Notes:
    - Run the Master launch file before
    - Make sure that the Arduino Mega from the low current box is
      recognized as "ttyACM0" in Linux.
%}

ThrustOutput_Q1 = 0;
ThrustOutput_Q2 = 100;
ThrustOutput_Q3 = 0;
ThrustOutput_Q4 = 0;
    
while true
    % Constrain motor commands to 0-100.
    Motor_Controller_Q1 = ThrustOutput_Q1;
    Motor_Controller_Q2 = ThrustOutput_Q2;
    Motor_Controller_Q3 = ThrustOutput_Q3;
    Motor_Controller_Q4 = ThrustOutput_Q4;
    
    if(50 < Motor_Controller_Q1)
        Motor_Controller_Q1 = 50;
    end
    
    if(50 < Motor_Controller_Q2)
        Motor_Controller_Q2 = 50;
    end
    
    if(50 < Motor_Controller_Q3)
        Motor_Controller_Q3 = 50;
    end
    
    if(50 < Motor_Controller_Q4)
        Motor_Controller_Q4 = 50;
    end
    
    Motor_Controller_Q1 = Motor_Controller_Q1 + 50.0;
    Motor_Controller_Q2 = Motor_Controller_Q2 + 50.0;
    Motor_Controller_Q3 = Motor_Controller_Q3 + 50.0;
    Motor_Controller_Q4 = Motor_Controller_Q4 + 50.0;
    
    if Motor_Controller_Q1 < 0 
        Motor_Controller_Q1 = 0;
    end
    
    if Motor_Controller_Q2 < 0
        Motor_Controller_Q2 = 0;
    end
    
    if Motor_Controller_Q3 < 0
        Motor_Controller_Q3 = 0;
    end
    
    if Motor_Controller_Q4 < 0
        Motor_Controller_Q4 = 0;
    end
    
    % Keep inside this loop to update the motor commands right away.
    mQ1_pub = rospublisher('/motor_q1', 'std_msgs/UInt16');   % create Matlab publisher to Q1 Arduino
    mQ2_pub = rospublisher('/motor_q2', 'std_msgs/UInt16');   % create Matlab publisher to Q2 Arduino
     mQ3_pub = rospublisher('/motor_q3', 'std_msgs/UInt16');   % create Matlab publisher to Q3 Arduino
     mQ4_pub = rospublisher('/motor_q4', 'std_msgs/UInt16');   % create Matlab publisher to Q4 Arduino

    % Converting ROS topic to MATLAB variable
     m_msg1 = rosmessage(mQ1_pub);
    m_msg2 = rosmessage(mQ2_pub);
     m_msg3 = rosmessage(mQ3_pub);
     m_msg4 = rosmessage(mQ4_pub);
    
    % Inputting our controller outputs to ROS Publisher variable
     m_msg1.Data = round(Motor_Controller_Q1, 0);
    m_msg2.Data = round(Motor_Controller_Q2, 0);
     m_msg3.Data = round(Motor_Controller_Q3, 0);
     m_msg4.Data = round(Motor_Controller_Q4, 0);
    
    %debug
    fprintf('m_msg 1-4: %f,%f,%f,%f \n',Motor_Controller_Q1,Motor_Controller_Q2,Motor_Controller_Q3,Motor_Controller_Q4)
    fprintf('\n')
    
    % Publish to ROS TOPIC 
     send(mQ1_pub,m_msg1);
    send(mQ2_pub,m_msg2);
     send(mQ3_pub,m_msg3);
     send(mQ4_pub,m_msg4);
end