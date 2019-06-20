/* This file contains Inertial Measurement Unit functions of this embedded system/robotic platform.
   
   AccGyro_Offset_Calculation() : Calculates the offset of data received from Acc & Gyro sensor. The offset is 
                                  the average of first 500 acc & gyro data. The robot should be stable/no motion 
                                  mode during this calculation

   AccGyro_Calibration()        : After calculating offset, this offset value is subtracted from all newly received 
                                  acc & gyro data. Then it's divided by the sensitivity value. The sensitivity values 
                                  are determined by the datasheet of related sensor. (Bosch BMI160) Acc data is mulptiplied 
                                  with 1G to transform the unit from 1G to the m/s2. Through this optimizations, the result 
                                  data is more correct than raw acc & gyro data and ready to use.
   
   dt_Calculation()             : Calculates the impact duration of incoming data.
   
   getYaw()                     : Calculates the degree equvalent of the received Gyro data in this system tour. 
                                  For example, in this system tour,  if the robot turns to left one degree,
                                  it gives +1 or if it turns to right 5 degree it gives -5 as the result.

   getPitch()                   : Calculates Pitch value made during this system tour.
   
   getRoll()                    : Calculates Roll value  made during this system tour.
   
   getAccX()                    : Returns the Acc value of X axe.
   
   getAccY()                    : Returns the Acc value of Y axe.
   
   getAccZ()                    : Returns the Acc value of Z axe.
   
   Drift_Correction()           : By calculating the amount of deviation to the left or to the 
                                  right when robot is going straight, the power of the left or right motor is 
                                  increased in line with this amount and the balance between the two motors is 
                                  tried to be achieved.
    
   yaw_calibration()            : Since this is a complex embedded system, and the data from the Gyro & Acc Sensor 
                                  is made ready for use via various functions such as AccGyro_Calibration (), dt_Calculation (), 
                                  GetTotalYPR (), the calculation of the total degree values may delay the action to be taken 
                                  in accordance with the calculated value. Suppose that the Turn_Left_TD (30) function is called 
                                  by the user. In this case, robot is expected to stop after turning 30 degrees to the left.
                                  When the data coming from the gyro sensor pass through all these stages and the degree 
                                  information is calculated, it is already delayed to start the stop operation. 
                                  So, preparing "30 degree left turned" information, causes delays for stopping after exactly 
                                  30 degree left turned. To avoid this error, while running each degree turning function 
                                  (Turn_Left_TD or Turn_Right_TD) calls the Yaw_Calibration () function in the Run_System_Components.c 
                                  file in the background. This function determines a YAW_CALIBRATION_COEFFICIENT value, 
                                  depending on the size of the expected range (expected_gyro_degree), and 
                                  execute the motors stop command without waiting until the expected_gyro_degree 
                                  is reached when expected_gyro_degree - YAW_CALIBRATION_COEFFICIENT is reached.
                                   
   Interrupt_Control()         :  This function is used to examine interrupts which are another characteristic of this sensor.
                                  With the interrupt feature and this function, the robot can detect the direction of an impact, 
                                  whether it is actually in motion, or if it is stuck in place, even when the motors are running, 
                                  and vibrations on the surface (for example, when you tap the table). 
                                  
   GetTotalYPR()               : When this function is called with argument dt calculated before, getYaw (), getPitch (), getRoll () 
                                 functions are used to obtain the degree information by multiplying the Gyro data of each axis by dt 
                                 and dividing it by 1000. For example, when robot is rotated 1 degree to the left, +1 return value is 
                                 obtained by using getYaw function. Considering that Acc_Gyro_Tasks () function has been completed 3 
                                 times after the offset calculation has been completed and the calibration is done by getYaw function, 
                                 passing through the dt stages. getYaw function returns to +1 degrees (1 degree left), the second round 
                                 to +2 degrees (2 degrees left), the third round to -8 degrees (8 degrees right). So the global yaw variable 
                                 gets -5. In other words, the yaw value works like a compass by informing the user how many degrees the robot
                                 has returned from its starting position.                                
*/



bool AccGyro_Offset_Calculation()
{
    static uint16_t counter = 0;

    /***** double offset calculation for acc  *****/
    if(counter < 250) 
    {    
        sumAX += accXdata.accX;
        sumAY += accYdata.accY;
        sumAZ += accZdata.accZ;
    }
    else if(counter == 250)
    {
        offsetAX = (float)sumAX / (float)counter;
        offsetAY = (float)sumAY / (float)counter;     
        offsetAZ = (float)sumAZ / (float)counter;
        sumAX = 0;
        sumAY = 0;
        sumAZ = 0;
    }    
    else if(counter <= 500)
    {
        sumAX += accXdata.accX - offsetAX;
        sumAY += accYdata.accY - offsetAY;
        sumAZ += accZdata.accZ - offsetAZ;
    }    


    /***** single offset calculation for gyro *****/
    if (counter < 500)
    {
        sumGX += gyroXdata.gyroX;
        sumGY += gyroYdata.gyroY;
        sumGZ += gyroZdata.gyroZ;
        counter++;
        return false;
    }
    else if (counter == 500) 
    {   
        offsetGX = (float)sumGX / (float)counter;
        offsetGY = (float)sumGY / (float)counter;     
        offsetGZ = (float)sumGZ / (float)counter;
        offsetAX += (float)sumAX / (float)(counter / 2);
        offsetAY += (float)sumAY / (float)(counter / 2);     
        offsetAZ += (float)sumAZ / (float)(counter / 2);

        counter++;
        return true;
    }
    else if(counter > 500)
    {
        CONTROL_BITS.acc_gyr_offset_calculated = 1;
        return true;  
    }     
}

void AccGyro_Calibration()
{
    /*******  calibration gyro values through the offset value and converting to degree/second *******/
    FGX = (float) gyroXdata.gyroX - offsetGX;
    FGX = FGX / GYRO_SENSITIVITY;
    FGY = (float) gyroYdata.gyroY - offsetGY;
    FGY = FGY / GYRO_SENSITIVITY;
    FGZ = (float) gyroZdata.gyroZ - offsetGZ;
    FGZ = FGZ / GYRO_SENSITIVITY;


    /*******  calibration acc values through the offset value *******/
    FAX = (float) accXdata.accX - offsetAX;
    FAX = (FAX / ACC_SENSITIVITY) * 9.80655;
    FAY = (float) accYdata.accY - offsetAY;
    FAY = (FAY / ACC_SENSITIVITY) * 9.80655;
    FAZ = (float) accZdata.accZ - offsetAZ;
    FAZ = (FAZ / ACC_SENSITIVITY) * 9.80655;
    
    CONTROL_BITS.auto_calibrate = 1;
}        

float dt_Calculation()
{
    float dt = 0;
    static uint32_t recent_time_stamp = 0;
    uint32_t change_of_time_stamp_bits = 0; 
    
    if(acg_time_stamp.acgtimestamp >= recent_time_stamp)
        change_of_time_stamp_bits = (uint32_t) acg_time_stamp.acgtimestamp - recent_time_stamp; 
    else
        change_of_time_stamp_bits = recent_time_stamp - (uint32_t) acg_time_stamp.acgtimestamp;
    
    
    recent_time_stamp = acg_time_stamp.acgtimestamp;
    
    /***** dt calculation in ms *****/
    
    dt = (float)change_of_time_stamp_bits * 0.039;
    return dt;
}

float getYaw(float dt)
{
    return  FGZ * (float)(dt / (float)1000); 
}

float getPitch(float dt)
{
    return  FGY * (float)(dt / (float)1000); 
}

float getRoll(float dt)
{
    return  FGX * (float)(dt / (float)1000); 
}

float getAccX()
{
    return  FAX; 
}

float getAccY()
{
    return  FAY; 
}

float getAccZ()
{
    return  FAZ; 
}



void Drift_Correction(uint8_t mode)
{
    static float yaw_begin = 0.0;
    static bool drift_r = false;
    static bool drift_l = false;
    static IMU_STATES dc_states = INITIALIZE;
   
    switch(mode)
    {
        case (NO_DRIFT_CORRECTION) :
        {    
            break;
        }    
        case (DRIFT_CORRECTION_ENABLE) :
        {
            if(!CONTROL_BITS.abs_control_flag)
            {
                if(CONTROL_BITS3.drift_correction_restart)
                {
                    CONTROL_BITS3.drift_correction_restart = false;
                    dc_states = INITIALIZE;
                    drift_l = false;
                    drift_r = false;
                    RGBbottom.bottomRGB = 0;
                }    
                
                switch(dc_states)
                {
                    case INITIALIZE: 
                    {
                        if(Wait_MilliSec(wait_drift, 100))
                        {
                            yaw_begin = yaw;
                            dc_states = CONTROL;
                        }    
                        break;
                    }
                    case CONTROL: 
                    {
                        if (yaw >= yaw_begin + 3)
                        {
                            drift_l = true;
                            dc_states = EXECUTE;
                        }
                        else if (yaw <= yaw_begin - 3)
                        {
                            drift_r = true;
                            dc_states = EXECUTE;
                        } 
                        break;
                    }
                    case EXECUTE: 
                    {
                        CONTROL_BITS3.drift_correction = true;
                        if(drift_l)
                        {
                            if(motorspeedL <= motorspeed + 3)
                            {    
                                RGBbottom.green = 100;
                                motorspeedL += 1;
                                Motor_SpeedL(motorspeedL);
                            }
                            else if (motorspeedR >= motorspeed - 3)
                            {
                                RGBbottom.blue = 100;
                                motorspeedR -= 1;
                                Motor_SpeedR(motorspeedR);
                            }    
                            dc_states = DONE;
                        }
                        else if (drift_r)
                        {
                            if(motorspeedR <= motorspeed + 3)
                            {    
                                RGBbottom.blue = 100;
                                RGBbottom.green = 100;
                                motorspeedR += 1;
                                Motor_SpeedR(motorspeedR);
                            }
                            else if (motorspeedL >= motorspeed - 3)
                            {
                                RGBbottom.red = 100;                                
                                motorspeedL -= 1;
                                Motor_SpeedL(motorspeedL);
                            }    
                            dc_states = DONE;
                        }    
                        break;
                    }
                    case DONE: 
                    {
                        drift_r = false;
                        drift_l = false;
                        RGBbottom.bottomRGB = 0;
                        CONTROL_BITS3.drift_correction = false;
                        dc_states = INITIALIZE;
                        break;
                    }
                    default:
                        break;
                } 
            } 
            break;
        }    
        default:
            break;
    }        
}


void yaw_calibration()
{
    static IMU_STATES ywc = INITIALIZE;
    
    if(expected_gyro_degree && !CONTROL_BITS.tap_interrupt_enable && !CONTROL_BITS.acc_gyr_constraint)
    {
        switch(ywc)
        {
            case INITIALIZE:
            {
                if(expected_gyro_degree >= YAW_CALIBRATION_COEFFICIENT)
                    ywc = EXECUTE;
                else
                    ywc = DONE;
                break;
            }    
            case EXECUTE:
            {
                if ((uint16_t)fabs(yaw - yaw_tmp) >= (expected_gyro_degree - YAW_CALIBRATION_COEFFICIENT))
                {       
                    Motor_Direction(MOTOR_LEFT, MOTOR_STOP);
                    Motor_Direction(MOTOR_RIGHT, MOTOR_STOP);
                    if(jump_to_ioextask())
                    {

                        if (current_move == 3)
                            yaw += YAW_CALIBRATION_COEFFICIENT;
                        if (current_move == 4)
                            yaw -= YAW_CALIBRATION_COEFFICIENT;
                        ywc = DONE;
                    }    
                } 
                break;
            }   
            case DONE:
            {
                expected_gyro_degree = 0;
                CONTROL_BITS2.exp_gyr_degree_done = true;
                ywc = INITIALIZE;
                break;
            }       
        }  
    }
}

void Interrupt_Control()
{
    // flat detection
    if(ACG_INTERRUPT_STATUS3.flat)
    {
        CONTROL_BITS2.flat = true;
    }    
    else
        CONTROL_BITS2.flat = false;

    // orientation
    if(ACG_INTERRUPT_STATUS3.orient_z)
    {
        CONTROL_BITS2.upside_down = true;
    }
    else
        CONTROL_BITS2.upside_down = false;

   
    // Hit detection

    if(ACG_INTERRUPT_STATUS2.tap_x)
    {
        if(ACG_INTERRUPT_STATUS2.tap_sign)
            hit_from = FF;
        else
            hit_from = FB;
    }
    else if(ACG_INTERRUPT_STATUS2.tap_y)
    {
        if(ACG_INTERRUPT_STATUS2.tap_sign)
            hit_from = FL;
        else
            hit_from = FR;
    }
    else
        hit_from = NH;
    
    // slow/no motion
    if(ACG_INTERRUPT_STATUS1.no_mo)
        CONTROL_BITS2.no_motion = true;
    else
        CONTROL_BITS2.no_motion = false;
        
}

void GetTotalYPR(float dt)
{
    roll += getRoll(dt);
    pitch += getPitch(dt);
    if((!CONTROL_BITS.yaw_calcul_constraint || (Current_Move() == LEFT_ROTATION || Current_Move() == RIGHT_ROTATION)))
    {
        yaw += getYaw(dt);
    }        
}
