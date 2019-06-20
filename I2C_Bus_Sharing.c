/* It's the source code of I2C_Bus_Sharing of these embedded system. 
   
   In these purpose, I2C_Done() function prevents that another sensor from who having the bus, uses the bus, 
   until it finishes its work in this system tour (transmits the necessary information to the microprocessor and 
   receives the necessary answers). This prevents the sensors from interrupting each other's communication. 
   If the bus is idle, the function returns true so that the next sensor receives the bus. So all sensors use the bus in sequence.

    In this platform there is a IOEX Driver (who controls motors), a LED Driver(who controls leds) an Accelerometer & Gyro Sensor and a 
    proxy sensor who needs to use I2C Bus for data transfering with microprocessor. Therefore the Run_System_Components() function
    make these sensors use I2C Bus sequentially with the help of I2C_Done() function. When a component have the bus, it start to use its
    own task function to program itself in the right way. The task functions is decribed in Components_Task_Function.c file
*/

bool I2C_Done(COM_TYPE commtype)
{
    bool result = false;
    if(i2cStates == I2C_DONE || i2cStates == I2C_START)
    {
        switch(commtype)
        {
            case IOEX:
            {                       
                if(STATUS_FLAGS.TASKFLAGREG == 0 || STATUS_FLAGS.ioextasks == 1)
                {
                    result = true;
                }
                break;
            }
            case LEDDRIVER:
            {
                if(STATUS_FLAGS.TASKFLAGREG == 0 || STATUS_FLAGS.leddrivertasks == 1)
                {
                    result = true;
                }
                break;
            }
            case PC_SENSOR:
            {
                if(STATUS_FLAGS.TASKFLAGREG == 0 || STATUS_FLAGS.pcsensortasks ==1)
                {
                    result = true;
                }
                break;
            }
            case ACC_GYRO:
            {
                if(STATUS_FLAGS.TASKFLAGREG == 0 || STATUS_FLAGS.accgyrotasks == 1)
                {
                    result = true;
                }
                break;
            }                
        }
    }
    return result;
}

void Run_System_Components
{
    if(I2C_Done(PC_SENSOR))
    {       
        PC_Sensor_Tasks();         
    }
            
    if(I2C_Done(ACC_GYRO))
    {
        Acc_Gyro_Tasks(); 
            
        if(IsAccGyro_Data_Ready())
        {            
            if(IsAccGyroPermission())
            {    
                AccGyro_Calibration();
                      
                /******* calculating dt  ******/
                       
                float dt = 0;
                dt = dt_Calculation();

                /******* calculating total yaw pitch roll degree *****/

                GetTotalYPR(dt);
                /******** get Acc X-Y-Z ****/
                        
                acc_x = getAccX();
                        
                /******** interrupt control *********************/

                Interrupt_Control();
                            
                /*****  drift correction *****/

                if (Current_Move() == FORWARD_TRANSLATION)
                {
                    Drift_Correction(drift_correction_mode);
                }   
                else
                    CONTROL_BITS3.drift_correction_restart = true;
                              
          }
      }      
  }                       
            
  yaw_calibration();
  Abs_Control();
           
  if(yaw > 360)
      yaw -= 360;
  else if (yaw < -360)
      yaw += 360;
            
  if(Current_Move() == FORWARD_TRANSLATION && motorspeed)
  {
      if(Wait_MilliSec(wait_distx, 250))
      {
          tmpdeg = yaw * DEGTORAD;
          length_x = ((motorspeed - unit_speed) * coefficient) + unit_length;
          tempx = (length_x * cos(tmpdeg));
          tempy = (length_x * sin(tmpdeg));
              
          distance_x += tempy * -1;
          distance_y += tempx;                   
      }
  }    
  else if(Current_Move() == BACKWARD_TRANSLATION && motorspeed)   
  {
      if(Wait_MilliSec(wait_distx, 250))
      {
          tmpdeg = yaw * DEGTORAD;
          length_x = (((motorspeed - unit_speed) * coefficient) + unit_length) * -1;
          tempx = (length_x * cos(tmpdeg));
          tempy = (length_x * sin(tmpdeg));
                    
          distance_x += tempy * -1;
          distance_y += tempx;    
      }
   }
   else
   {    
       Reset_Timer(wait_distx);
   }
            
   if(I2C_Done(IOEX))
   {  
       if(CONTROL_BITS.auto_calibrate  && !CONTROL_BITS2.ioex_interrupt)
       {
           if(IsQueuePermission(MOTOR))   
               Dequeue(MOTOR);
            IOEX_Tasks(); 
        }     
    }        
                        
    if(I2C_Done(LEDDRIVER))
    {     
        if(IsQueuePermission(LEDFRONT))  
            Dequeue(LEDFRONT);
        if(IsQueuePermission(LEDTOP))  
            Dequeue(LEDTOP);   
        if(IsQueuePermission(LEDBOTTOM))  
            Dequeue(LEDBOTTOM);
        if(IsQueuePermission(LEDUSER))  
            Dequeue(LEDUSER);   
         LedDriver_Tasks(); 
    } 
                
}

