

/****** Motor Functions ******/

void Turn_Left() # Makes the robot turn to the left until Motor_Stop() function is called.
{
     Motor_Direction(MOTOR_LEFT, MOTOR_BACKWARD);
     Motor_Direction(MOTOR_RIGHT, MOTOR_FORWARD);
}

void Turn_Right() # Makes the robot turn to the right until Motor_Stop() function is called.
{
     Motor_Direction(MOTOR_LEFT, MOTOR_FORWARD);
     Motor_Direction(MOTOR_RIGHT, MOTOR_BACKWARD);
}

void Go_Ahead()  # Makes the robot go ahead until Motor_Stop() function is called.
{
    Motor_Direction(MOTOR_LEFT, MOTOR_FORWARD);
    Motor_Direction(MOTOR_RIGHT, MOTOR_FORWARD);
}

void Go_Back()  # Makes the robot go back until Motor_Stop() function is called.
{
    Motor_Direction(MOTOR_LEFT, MOTOR_BACKWARD);
    Motor_Direction(MOTOR_RIGHT, MOTOR_BACKWARD);
}

void Motor_Stop() # Stops the robot
{
    Motor_Direction(MOTOR_LEFT, MOTOR_STOP);
    Motor_Direction(MOTOR_RIGHT, MOTOR_STOP);
}

bool Turn_Left_TD(uint16_t degree) # Makes the robot to turn left to the value given in the parameter.
{   
    if(yaw_flag2)
    {
        expected_gyro_degree = degree;
        CONTROL_BITS2.exp_gyr_degree_done = false;
        yaw_tmp = yaw;
    }    


    if(CONTROL_BITS2.exp_gyr_degree_done)
    {
        expected_gyro_degree = 0;
        yaw_flag2 = 1;
        return true;
    }    
    else
    {
        yaw_flag2 = 0;
        Turn_Left();
        return false;
    }    
}

bool Turn_Right_TD(uint16_t degree) # Makes the robot to turn right to the value given in the parameter.
{   
    if(yaw_flag2)
    {
        expected_gyro_degree = degree;
        CONTROL_BITS2.exp_gyr_degree_done = false;
        yaw_tmp = yaw;
    }    


    if(CONTROL_BITS2.exp_gyr_degree_done)
    {
        expected_gyro_degree = 0;
        yaw_flag2 = 1;
        return true;
    }    
    else
    {
        yaw_flag2 = 0;
        Turn_Right();
        return false;
    }      
}

/***** Proxy & Color Functions *****/

bool IsAnyObject_fo(uint8_t threshold) # Returns true if there is any object in the front of the robot. It decides according to the threshold proxy value.
{
    Sensor_Proxy_Open();
    
    return SDAT.dataProxy > threshold; 
}

COLOR Color_Pick() # Determines the color of the object in front of the robot
{
    Sensor_Proxy_Open();
    
    if(IsAnyObject_fo(25) && PRSENSOR_ENABLE.aen)
    {   
        return  Read_Color(Read_Red_Light(), Read_Green_Light(), Read_Blue_Light()).color; 
    }
    return COLOR_READ_ERROR;
}

/***** 


