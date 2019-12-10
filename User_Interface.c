

/****** Motor Functions ******/

void Turn_Left() // Makes the robot turn to the left until Motor_Stop() function is called.
{
     Motor_Direction(MOTOR_LEFT, MOTOR_BACKWARD);
     Motor_Direction(MOTOR_RIGHT, MOTOR_FORWARD);
}

void Turn_Right() // Makes the robot turn to the right until Motor_Stop() function is called.
{
     Motor_Direction(MOTOR_LEFT, MOTOR_FORWARD);
     Motor_Direction(MOTOR_RIGHT, MOTOR_BACKWARD);
}

void Go_Ahead()  // Makes the robot go ahead until Motor_Stop() function is called.
{
    Motor_Direction(MOTOR_LEFT, MOTOR_FORWARD);
    Motor_Direction(MOTOR_RIGHT, MOTOR_FORWARD);
}

void Go_Back()  // Makes the robot go back until Motor_Stop() function is called.
{
    Motor_Direction(MOTOR_LEFT, MOTOR_BACKWARD);
    Motor_Direction(MOTOR_RIGHT, MOTOR_BACKWARD);
}

void Motor_Stop() // Stops the robot
{
    Motor_Direction(MOTOR_LEFT, MOTOR_STOP);
    Motor_Direction(MOTOR_RIGHT, MOTOR_STOP);
}

bool Turn_Left_TD(uint16_t degree) // Makes the robot to turn left to the value given in the parameter.
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

bool Turn_Right_TD(uint16_t degree) // Makes the robot to turn right to the value given in the parameter.
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

bool IsAnyObject_fo(uint8_t threshold) // Returns true if there is any object in the front of the robot. It decides according to the threshold proxy value.
{
    Sensor_Proxy_Open();
    
    return SDAT.dataProxy > threshold; 
}

COLOR Color_Pick() // Determines the color of the object in front of the robot
{
    Sensor_Proxy_Open();
    
    if(IsAnyObject_fo(25) && PRSENSOR_ENABLE.aen)
    {   
        return  Read_Color(Read_Red_Light(), Read_Green_Light(), Read_Blue_Light()).color; 
    }
    return COLOR_READ_ERROR;
}

/***** Timing Functions ****/

bool Wait_MilliSec(WAITTYPE wtype,uint32_t msec)
{
    bool completed = false;
   
    if(wait_untill[wtype] == 0)
    {
         wait_untill[wtype] = TimeMS.millisecond + msec; 
    }
    
    if(wait_untill[wtype]  <= TimeMS.millisecond) 
    {
       wait_untill[wtype] =0;
       completed = true;
    }
       return completed;
}

void Reset_Timer(WAITTYPE wtype)
{
    wait_untill[wtype] = 0;
}

uint32_t Remain_Time(WAITTYPE wtype)
{
    return wait_untill[wtype] - TimeMS.millisecond; 
}

/************ LED Functions ********************/

void Led_Configuration(MODE led_engine)
{
    if(led_engine)
        leddrvStates = CHIP_ENABLE;
    else
        leddrvStates = CHIP_DISABLE;
}

uint8_t Get_LTred()
{
    return RGBtop.red;
}

uint8_t Get_LTgreen()
{
    return RGBtop.green;
}

uint8_t Get_LTblue()
{
    return RGBtop.blue;
}

uint8_t Get_LBred()
{
    return RGBbottom.red;
}

uint8_t Get_LBgreen()
{
    return RGBbottom.green;
}

uint8_t Get_LBblue()
{
    return RGBbottom.blue;
}

uint8_t Get_LH()
{
    return Led_Headlight;
}

uint8_t Get_LU()
{
    return Led_User;
}

void Set_LT(uint8_t red, uint8_t green, uint8_t blue)
{
    RGBtop.red = red;
    RGBtop.green = green;
    RGBtop.blue = blue;
}

void Set_LB(uint8_t red, uint8_t green, uint8_t blue)
{
    RGBbottom.red = red;
    RGBbottom.green = green;
    RGBbottom.blue = blue;
}

void Set_LH(uint8_t headlight)
{
    Led_Headlight = headlight;
}

void Set_LU(uint8_t leduser)
{
    Led_User = leduser;
}

void Set_LTB_Color(COLOR color)
{
    switch(color)
    {
        case(RED):
        {    
            Set_LT(100,0,0);
            Set_LB(100,0,0);
            break;
        }    
        case(GREEN):
        {    
            Set_LT(0,100,0);
            Set_LB(0,100,0);
            break;
        }    
        case(BLUE):
        {    
            Set_LT(0,0,100);
            Set_LB(0,0,100);
            break;
        }   
        case(PINK):
        {    
            Set_LT(100,0,100);
            Set_LB(100,0,100);
            break;
        }    
        case(TURQUOISE):
        {    
            Set_LT(0,100,100);
            Set_LB(0,100,100);
            break;
        }    
        case(ORANGE):
        {    
            Set_LT(150,25,0);
            Set_LB(150,25,0);
            break;
        }   
        case(WHITE):
        {
            Set_LT(100,100,100);
            Set_LB(100,100,100);
            break;
        }   
        case (YELLOW):
        {
            Set_LT(100,100,0);
            Set_LB(100,100,0);
            break;
        }     
    }       

}

void Turn_Off_LT(void)
{
    RGBtop.red = 0;
    RGBtop.green = 0;
    RGBtop.blue = 0;
}

void Turn_Off_LB(void)
{
    RGBbottom.red = 0;
    RGBbottom.green = 0;
    RGBbottom.blue = 0;
}

void Turn_Off_LH()
{
    Led_Headlight = 0;
}

void Turn_Off_LU()
{
    Led_User = 0;
}

void LT_Increase(char color)
{
    switch(color)
    {
        case ('r') :
        {    
            RGBtop.red++;
            break;
        }    
        case ('g') :
        {    
            RGBtop.green++;
            break;
        }    
        case ('b') :
        {    
            RGBtop.blue++;
            break;
        }    
        case ('a') :   
        {    
            RGBtop.red++;
            RGBtop.green++;
            RGBtop.blue++;
            break;
        }    
    }        
}


void LB_Increase(char color)
{
    switch(color)
    {
        case ('r') :
        {    
            RGBbottom.red++;
            break;
        }    
        case ('g') :
        {    
            RGBbottom.green++;
            break;
        }    
        case ('b') :
        {    
            RGBbottom.blue++;
            break;
        }    
        case ('a') :   
        {    
            RGBbottom.red++;
            RGBbottom.green++;
            RGBbottom.blue++;
            break;
        }    
        default :
            break;
    }        
}


uint8_t Set_LT_Queue(uint8_t red, uint8_t green, uint8_t blue, uint16_t time)
{
    return Enqueue(LEDTOP,"Led_Top", 4, red, green, blue, time);
}

uint8_t Turn_Off_LT_Queue(uint16_t time)
{
    return Enqueue(LEDTOP,"Turn_Off_Top", 1, time);
}

uint8_t Set_LB_Queue(uint8_t red, uint8_t green, uint8_t blue, uint16_t time)
{
    return Enqueue(LEDBOTTOM,"Led_Bottom", 4, red, green, blue, time);
}

uint8_t Turn_Off_LB_Queue(uint16_t time)
{
    return Enqueue(LEDBOTTOM,"Turn_Off_Bottom", 1, time);
}


uint8_t Set_LH_Queue(uint8_t headlight, uint16_t time)
{
    return Enqueue(LEDFRONT,"HeadLight", 2, headlight, time);
}

uint8_t Turn_Off_LH_Queue(uint16_t time)
{
    return Enqueue(LEDFRONT,"Turn_Off_HeadLight", 1, time);
}

uint8_t Set_LU_Queue(uint8_t leduser, uint16_t time)
{
    return Enqueue(LEDUSER,"Led_User", 2, leduser, time);
}

uint8_t Turn_Off_LU_Queue(uint16_t time)
{
    return Enqueue(LEDUSER,"Led_User", 1, time);
}

/******* GYRO & ACC Functions *****/

/* The functions adding some movement, impact detection features to the embedded system using BMI160 Acc & Gyro sensor */

bool Isonflat()
{
    return CONTROL_BITS2.flat;
}

bool Isupsidedown()
{
    return CONTROL_BITS2.upside_down;
}

int8_t Hit_Detection()
{
    return hit_from;
}


bool Isanytap(MODE mode)
{
    if(CONTROL_BITS.acc_gyr_offset_calculated)
    {    
        switch(mode)
        {
            case 1:
            {    
                CONTROL_BITS.tap_interrupt_enable = 1;
                if(ACG_INTERRUPT_STATUS2.tap_z)
                    return true;
                return false;
            }    
            case 0:
            {    
                CONTROL_BITS.tap_interrupt_enable = 0;
                return false;
            }       
        }      
    }

    return false;    
}

bool Ismovementchange()
{
    static uint8_t mov_type_tmp;
    
    static IMU_STATES ismvmntchange = INITIALIZE;

    switch(ismvmntchange)
    {
        case INITIALIZE :
        {
            mov_type_tmp = Current_Move();
            ismvmntchange = CONTROL;
            break;
        }
        case CONTROL :
        {
            if(Current_Move() != mov_type_tmp)
            {    
                ismvmntchange = INITIALIZE;
                return true;
            }     
            break;   
        }  
    }
    return false;
}

void Back_To_Move(uint8_t move)
{
    switch(move)
    {
        case (FORWARD_TRANSLATION):
            Go_Ahead();
            break;
        case (BACKWARD_TRANSLATION):
            Go_Back();
            break;
        case (LEFT_ROTATION):
            Turn_Left();
            break;
        case (RIGHT_ROTATION):
            Turn_Right();
            break;
        default:    
            break;
    }        
}

bool Issuddenacc()
{
    static IMU_STATES ismvng = PREINITIALIZE;
    static float speed;
    
    if(Current_Move() == NO_MOVE)
    {
        Reset_Timer(wait_ut1);
        ismvng = PREINITIALIZE;
    }
    switch(ismvng)
    {
        case PREINITIALIZE :
        {
            if(Current_Move() == FORWARD_TRANSLATION || Current_Move() == BACKWARD_TRANSLATION)
                if(Wait_MilliSec(wait_ut1, 500))
                    ismvng = INITIALIZE;
            break;
        }    
        case INITIALIZE :
        {
            speed = speed_x;
            ismvng = CONTROL;
            break;
        }    
        case CONTROL :
        {
            if(Wait_MilliSec(wait_ut1, 100))
            {
                if(speed_x >= speed + 30 || speed_x <= speed - 30)
                    ismvng = EXECUTE;
                else
                    ismvng = INITIALIZE;
            }    
            break;
        }    
        case EXECUTE :
        {
            if(speed_x < 20 && speed_x > - 20)
            {
                ismvng = INITIALIZE;
                break;
            }    
            else
            {
                return true;
            }    
        }    
    }    
    
    return false;
}

bool Ismotorstuck()
{
    static IMU_STATES ismtrstck = PREINITIALIZE;
    static uint8_t cur_mov;
    static uint8_t id = EFID_ERROR;
    
    switch(ismtrstck)
    {
        case PREINITIALIZE :
        {
            if(Issuddenacc())
            {    
                cur_mov = Current_Move();
                ismtrstck = INITIALIZE;
                Set_LT_Queue(100,0,0,200);
                id = Turn_Left_TD_Queue(90);
            }
            break;
        }    
        case INITIALIZE :
        {
            if(efi_table[id])
            {
                Reset_Timer(wait_ut3);
                ismtrstck = CONTROL;
                Set_LT_Queue(100,100,0,200);
            }  
            else if(Wait_MilliSec(wait_ut3, 2000))
            {
                Queue_Leftshift(MOTOR);
                id = Turn_Right_TD_Queue(90);
                ismtrstck = EXECUTE;
            }    
            break;
        }
        case EXECUTE :
        {
            if(efi_table[id])
            {
                Reset_Timer(wait_ut3);
                Set_LT_Queue(0,0,100,200);
                ismtrstck = CONTROL;
            }  
            else if(Wait_MilliSec(wait_ut3, 2000))
            {
                Queue_Leftshift(MOTOR);
                ismtrstck = DONE;
                Motor_Stop();   
                return true;
            }
            break;
        }   
        case CONTROL :
        {
            if(!CONTROL_BITS.abs_control_flag)
            {    
                Back_To_Move(cur_mov);
                ismtrstck = DONE;
            }
            break;
        }    
        case DONE :
        {
            if(Wait_MilliSec(wait_ut3, 5000))
                ismtrstck = PREINITIALIZE;
        }    
    } 
    return false;
}

void Yaw_Control(MODE mode)
{
    CONTROL_BITS.yaw_calcul_constraint = mode;
}

int8_t Get_Temperature()
{
    DATAS temp;
    temp.dataL = temperature.tempL;
    temp.dataH = temperature.tempH;

    return (temp.dataS * (1 / pow(2,9)) + 23);
}

bool IsCalibrationDone()
{
    return CONTROL_BITS.auto_calibrate; 
}
/******** IRDA FUNCTIONS *********/

/* User functions of IRDA mechanism. Turning on/off the IRDA ports, Data transformation through IRDA ports(special packages or broadcast),
buffer controling etc */

void IRDA_Port_Select(uint8_t irdaport) 
{
    switch (irdaport) 
    { 

        case 0:   // All IRDA DISABLED    (1=> Disable) ,  (0=> Enable) 
        { 
            IOX_P0.IREN1 = 1; 
            IOX_P0.IREN2 = 1; 
            IOX_P1.IREN3 = 1;
            break; 
        } 
        case 1:  //  FRONT 
        { 
            IOX_P0.IREN1 = 0; 
            IOX_P0.IREN2 = 1; 
            IOX_P1.IREN3 = 1; 
            break; 
        } 
        case 2:  // LEFT 
        { 
            IOX_P0.IREN1 = 1; 
            IOX_P0.IREN2 = 0; 
            IOX_P1.IREN3 = 1; 
            break; 
        } 
        case 3: //RIGHT 
        { 
            IOX_P0.IREN1 = 1; 
            IOX_P0.IREN2 = 1; 
            IOX_P1.IREN3 = 0; 
            break; 
        } 
    }
} 

void IRDA_Port_Select_Auto(uint8_t irdaports, uint16_t time_period)
{
    CONTROL_BITS.autoports = irdaports;
    autoport_period = time_period;
}

void IRDA_Port_Select_Auto_Stop()
{
    CONTROL_BITS.autoports = 0;
}

uint8_t Which_IRDA_Port() 
{
    switch(IRDA_PORTS.PORTSIRDA)
    {
        case (0):
            return 0;
        case (1):
            return IRDA_FRONT;
        case (2):
            return IRDA_LEFT;
        case (4): 
            return IRDA_RIGHT;    
    }        
}

bool Close_IRDA_Ports()
{
    IOX_P0.IREN1 = 1; 
    IOX_P0.IREN2 = 1; 
    IOX_P1.IREN3 = 1;

    if(!Which_IRDA_Port())
        return true;
    return false;
}

uint8_t Transmit(uint8_t dest, char datatype1, char datatype2, uint16_t irda_data, uint16_t time, uint8_t pac_imp, uint8_t br_ttl)
{
    if(TRANSMISSION_ID == TRANSMISSION_ERROR - 1)
    {
       TRANSMISSION_ID = 1; 
    }
    
    if(IsTransmission_Complete(TRANSMISSION_ID))
    {    
        transmission_table[TRANSMISSION_ID] = false;
        successful_transmissions_table[TRANSMISSION_ID] = false;
    }
    uint8_t size = Transmit_Buffer_Size();
    DATAU IRDA_DATA;
    IRDA_DATA.dataU = irda_data;
    
    if(size < TRANSMIT_BUFFER_SIZE) // transmit buffer? dolu de?ilse transmit verisi ekle s?raya.
    {
        Transmit_Buffer[size].IsAny = true; 
        Transmit_Buffer[size].datatype1 = datatype1; 
        Transmit_Buffer[size].datatype2 = datatype2;
        Transmit_Buffer[size].destination = dest;
        Transmit_Buffer[size].high_byte = IRDA_DATA.dataH;
        Transmit_Buffer[size].low_byte = IRDA_DATA.dataL;
        Transmit_Buffer[size].time = time;
        Transmit_Buffer[size].package_importance = pac_imp;  
        Transmit_Buffer[size].transmission_id = TRANSMISSION_ID;
        Transmit_Buffer[size].broadcast_ttl = br_ttl;
        
        TRANSMISSION_ID++;
        if(CONTROL_BITS.broadcast_packet)
        {    
            Transmit_Buffer[size].IsBroadcast = true;
            CONTROL_BITS.broadcast_packet = false;
        }    
        else
        {    
            Transmit_Buffer[size].IsBroadcast = false;
        } 
        return Transmit_Buffer[size].transmission_id;
    }
    return TRANSMISSION_ERROR;
}

uint8_t Transmit_Buffer_Size()
{

    int i; 
    uint8_t size = 0;
    for(i = 0; i < TRANSMIT_BUFFER_SIZE; i++) 
    {
        if (Transmit_Buffer[i].IsAny == false)
            break;
        else 
            size++;
    }
    
    return size;
}

void Transmit_Buffer_Empty()
{
    int i;
    uint8_t size = Transmit_Buffer_Size();
    
    for(i = 0; i < size; i++)
    {
        Transmit_Buffer[i].IsAny = false;
    }   
}

bool IsTransmitBufferFull()
{
     if (Transmit_Buffer_Size() == TRANSMIT_BUFFER_SIZE)
        return true;  
    return false;

}

bool IsTransmission_Complete(uint8_t id)
{
    return transmission_table[id];
}

bool IsTransmission_Successful(uint8_t id)
{
    return successful_transmissions_table[id];
}

struct RECEIVED_DATA Data_From_Register_Buffer() 
{ 
    uint8_t size = Register_Buffer_Size();
    if(!size)
    {
        struct RECEIVED_DATA tmp = Register_Buffer[0];
        uint8_t i;
        
        for(i = 0; i < size - 1; i++)
            Register_Buffer[i] = Register_Buffer[i +1];
        
        Register_Buffer[size - 1].data_source = '0';   
        Register_Buffer[size - 1].datatype = 0;
        Register_Buffer[size - 1].data.dataU = 0;
        
        return tmp;
    }    
}

uint8_t Signal_Call(uint8_t destination)
{
    return Transmit(destination, 'S', 'C', 0, 0, 3, 0);
}

uint8_t Broadcast_DC(uint8_t pac_imp, uint8_t br_ttl)
{
    return Transmit(NULL, 'B', 'C', 0, 0, pac_imp, br_ttl);
}


uint8_t Data_Start(char dest, uint8_t pac_imp)
{
    return Transmit(dest, 'D', 'S', 0, 0, pac_imp, 0);
}


uint8_t Data_End(char dest, uint8_t pac_imp)
{
    return Transmit(dest, 'D', 'E', 0, 0, pac_imp, 0);
}

void RID_reset()
{
    received_irda_data.destination = 0;
    received_irda_data.source = 0;
    received_irda_data.datatype1 = 0;
    received_irda_data.datatype2 = 0;
    received_irda_data.data.dataU = 0;
    received_irda_data.time.dataU = 0;
    received_irda_data.pac_imp = 0;
    received_irda_data.br_ttl = 0;
}


/********************** Buzzer Functions *************/

void Buzzer_Volume(uint16_t volume)
{
    STL_volume = volume;
    DRV_OC2_PulseWidthSet(volume);
}

void Buzzer_Note(uint16_t note)
{
    DRV_TMR1_Stop();
    DRV_TMR1_PeriodValueSet(note);
    DRV_TMR1_Start();
}
void Buzzer_Stop()
{
    DRV_TMR1_Stop();
}
uint8_t Buzzer_Note_Queue(uint16_t note, uint16_t timer)
{
    return Enqueue(BUZZER,"Buzzer_Note", 2, note, timer);
}

uint8_t Buzzer_Stop_Queue(uint16_t timer)
{
    return Enqueue(BUZZER,"Buzzer_Stop", 1, timer);
}


/********* User Button Functions *******/

BUTTON_STATE IsButtonPressed()
{
    return User_ButtonStateGet();
}

/*************** Bottom IRLEDS Functions **********/

void IRLEDS_Brightness(uint16_t power)
{
    DRV_OC3_PulseWidthSet(power);
}

bool Get_IRLED_Left()
{
    return Sense_LStateGet();
}

bool Get_IRLED_Right()
{
    return Sense_RStateGet();
}


/********** Terminal Functions **********/

/* The functions providing data transfer between embedded system and Pc, via a usb cable and a terminal program */

void write_to_terminal_unsigned8(uint8_t character)
{
    writeBuffer[terminal_size] = character;
    usb_tx_demand = true;
    terminal_size++;
    if(terminal_size == MIKROP_USB_CDC_COM_PORT_SINGLE_WRITE_BUFFER_SIZE)
        terminal_size = 0;
}

void write_to_terminal_signed8(int8_t character)
{
    writeBuffer[terminal_size] = character;
    
    usb_tx_demand = true;
    terminal_size ++;
}

void write_to_terminal_unsigned16(uint16_t character)
{
    DATAU character_temp;
    character_temp.dataU = character;
    
    writeBuffer[terminal_size] = character_temp.dataL;
    terminal_size++;
    writeBuffer[terminal_size] = character_temp.dataH;    
    
    usb_tx_demand = true;
    terminal_size ++;
}

void write_to_terminal_signed16(int16_t character)
{
    DATAS character_temp;
    character_temp.dataS = character;
    
    writeBuffer[terminal_size] = character_temp.dataH;
    terminal_size++;
    writeBuffer[terminal_size] = character_temp.dataL;    
    
    usb_tx_demand = true;
    terminal_size ++;
}



/***************** WIFI FUNCTIONS *****************/

/* General usage functions of Esp8266 module. The functions are created by using standart AT commands*/

void Wifi_Configuration(MODE wifi_engine)
{
    IOX_P0.WEN = wifi_engine;
}

void Wifi_Read_Reset()
{
    uint8_t i;
    for(i = 0; i <= wifi_read_index; i++)
        wifi_read[i] = 0;
    wifi_read_index = 0;
}

bool Wifi_Power_Interrogate()
{
    static WIFI_STATES pwriStates = Pending;
    uint8_t i;
    
    switch(pwriStates)
    {        
        case Pending :
        {
            if(CONTROL_BITS3.wifi_engine_enabled)
                pwriStates = Executing;
            break;
        }
        case Executing :
        {
            if(Wait_MilliSec(wait_wifi,500))
            {
                CONTROL_BITS2.wifi_read_demand = true;
                Wifi_Pwr_Interrogate();
                pwriStates = Waiting;
            }
            break;
        }
        case Waiting :
        {
            if(!strcmp(response2, "OK"))
            {
                for(i = 0; i <= wifi_read_index; i++)
                    write_to_terminal_unsigned8(wifi_read[i]); 
                Wifi_Read_Reset();
                CONTROL_BITS2.wifi_read_demand = false;
                Reset_Timer(wait_wifi);
                pwriStates = Done;
            }
            else if(Wait_MilliSec(wait_wifi, 5000))
            {
                pwriStates = Executing;
            }   
            break;
        }
        case Done :
        {
            pwriStates = Pending;
            return true;
        }
    }
    return false;
}


bool Wifi_Power(uint8_t mode)
{
    static WIFI_STATES pwrStates = Pending;
  
    switch(pwrStates)
    {        
        case Pending :
        {
            if(CONTROL_BITS3.wifi_engine_enabled)
                pwrStates = Executing;
            break;
        }
        case Executing :
        {
            if(Wait_MilliSec(wait_wifi,500))
            {
                Wifi_Pwr(mode);
                pwrStates = Waiting;
            }
            break;
        }
        case Waiting :
        {
            if(!strcmp(response2, "OK"))
            {
                Reset_Timer(wait_wifi);
                pwrStates = Done;
            }
            else if(Wait_MilliSec(wait_wifi, 2000))
            {
                pwrStates = Executing;
            }   
            break;
        }
        case Done :
        {
            pwrStates = Pending;
            return true;
        }
    }
    return false;
}

bool Wifi_Echo(MODE mode)
{
    static WIFI_STATES wecState = Pending;
    uint8_t i;
    switch(wecState)
    { 
        case Pending :
        {
            if(CONTROL_BITS3.wifi_engine_enabled)
                wecState = Executing;
            break;
        }
        case Executing :
        {
            Wifi_Ech(mode);
            wecState = Waiting;
            CONTROL_BITS2.wifi_read_demand = true;
            break;
        }
        case Waiting :
        {
            if(!strcmp(response2, "OK"))
            {
                wecState = Done;
            }
            else if(Wait_MilliSec(wait_wifi, 2000))
            {
                wecState = Executing;
            }    
            else
                
            break;
        }
        case Done :
        {
            wecState = Pending;
            return true;
        }
    }   
    return false;
}


bool Wifi_Restore()
{
    static WIFI_STATES rstState = Pending;
        
    switch(rstState)
    { 
        case Pending :
        {
            if(CONTROL_BITS3.wifi_engine_enabled)
                rstState = Executing;
            break;
        }
        case Executing :
        {
            if(Wait_MilliSec(wait_wifi,500))
            {
                rstState = Waiting;
                Wifi_Rst();
            }
            break;
        }
        case Waiting :
        {
            if(!strcmp(response2, "OK"))
            {
                rstState = Done;
                Reset_Timer(wait_wifi);
            }
            else if(Wait_MilliSec(wait_wifi,2000))
            {
                rstState = Executing;
            }    
            break;
        }
        case Done :
        {
            CONTROL_BITS2.tcp_client_created = false;
            CONTROL_BITS2.tcp_server_created = false;
            rstState = Pending;
            return true;
        }
    }
    return false;

}

bool Wifi_Version()
{
    static WIFI_STATES vrsState = Pending;
    uint8_t i = 0;
    
    switch(vrsState)
    { 
        case Pending :
        {
            if(CONTROL_BITS3.wifi_engine_enabled)
                vrsState = Executing;
            break;
        }
        case Executing :
        {
            if(Wait_MilliSec(wait_wifi,500))
            {
                CONTROL_BITS2.wifi_read_demand = true;
                Wifi_Vrsn();
                vrsState = Waiting;
            }   
            break;
        }    
        case Waiting :
        {
            if(!strcmp(response2, "OK"))
            {
                for(i = 0; i < wifi_read_index; i++)
                    write_to_terminal_unsigned8(wifi_read[i]); 
                CONTROL_BITS2.wifi_read_demand = false;
                Wifi_Read_Reset();
                Reset_Timer(wait_wifi);
                vrsState = Done;
            } 
            else if(Wait_MilliSec(wait_wifi, 2000))
            {
                wifi_read_index = 0;
                vrsState = Executing;
            }    
            break;
        }    
        case Done :
        {
            vrsState = Pending;
            return true;
        } 
    }    
    return false;
}

bool Wifi_Update(char * ssid, char * pwd)
{
    static UPDATE_STATES updtState = U_Version;
    uint8_t i;
    
    if(CONTROL_BITS3.wifi_engine_enabled)
    {
        switch(updtState)
        { 
            case U_Version :
            {
                if(Wifi_Version())
                {
                    updtState = U_Create_Client;
                }    
                break;
            }    
            case U_Create_Client :
            {
                if(Create_Client(ssid, pwd))
                {
                    updtState = U_Update;
                }
                break;
            }    
            case U_Update :
            {
                if(Wait_MilliSec(wait_wifi, 5000))
                {    
                    Wifi_Updt();
                    CONTROL_BITS2.wifi_read_demand = true;
                    updtState = U_Update_Waiting;
                }
                break;
            } 
            case U_Update_Waiting :
            {
                if(!strcmp(response2, "OK"))
                {
                    for(i = 0; i < wifi_read_index; i++)
                        write_to_terminal_unsigned8(wifi_read[i]); 
                    CONTROL_BITS2.wifi_read_demand = false;
                    Reset_Timer(wait_wifi);
                    Wifi_Read_Reset();
                    updtState = U_Update_Done;
                }
                else if(Wait_MilliSec(wait_wifi, 5000))
                {
                    wifi_read_index = 0;
                    updtState = U_Update;
                }    
                break;
            } 
            case U_Update_Done :
            {
                return true; 
            } 
        }    
        return false;
    }    
}

/** Server Functions **/


/* These are the functions that can be used to set the module as server. Also the functions that can be used only after being server
are here*/

bool Create_TCP_Server(char * ip, char * ssid, char * pwd, char * port)
{
    static SERVER_CREATE_STATES srvState = Set_IP_Adress;
    
    if(CONTROL_BITS3.wifi_engine_enabled)
    {    
        switch(srvState)
        { 
            case Set_IP_Adress :
            {
                if(Wait_MilliSec(wait_wifi,500))
                {
                    SetIPAdress(ip);
                    srvState = Set_IP_Adress_Waiting;
                }    
                break;
            }    
            case Set_IP_Adress_Waiting :
            {
                if(!strcmp(response2, "OK"))
                {
                    Reset_Timer(wait_wifi);
                    srvState = Set_IP_Adress_Done;
                }
                else if(Wait_MilliSec(wait_wifi,2000))
                {
                    srvState = Set_IP_Adress;
                }    
                break;
            } 
            case Set_IP_Adress_Done :
            {
                if(Wait_MilliSec(wait_wifi,500))
                {
                    srvState = TCP_Server_Configuration;
                }
                break;
            } 
            case TCP_Server_Configuration :
            {   
                TCP_Server_Configure(ssid, pwd, 3, 0);   
                srvState = TCP_Server_Configuration_Waiting;
                break;
            }    
            case TCP_Server_Configuration_Waiting :
            {   
                if(!strcmp(response2, "OK"))
                {
                    Reset_Timer(wait_wifi);
                    srvState = TCP_Server_Configuration_Done;
                }    
                else if(Wait_MilliSec(wait_wifi,2000))
                {
                    srvState = TCP_Server_Configuration;
                }   
                break;
            } 
            case TCP_Server_Configuration_Done :
            {
                if(Wait_MilliSec(wait_wifi,500))
                {    
                    srvState = Set_Server_Mode;
                }
                break;
            }   
            case Set_Server_Mode:
            {
                SetWifi_Mode(wifi_mode_ap);   
                srvState = Set_Server_Mode_Waiting;
                break;
            }        
            case Set_Server_Mode_Waiting:
            {
                if(!strcmp(response2, "OK"))
                {      
                    Reset_Timer(wait_wifi);
                    srvState = Set_Server_Mode_Done;
                }
                else if(Wait_MilliSec(wait_wifi,2000))
                {
                    srvState = Set_Server_Mode;
                }   
                break;
            }   
            case Set_Server_Mode_Done:
            {
                if(Wait_MilliSec(wait_wifi,500))
                { 
                    srvState = Server_Mux;
                }
                break;
            }   
            case Server_Mux:
            {
                Allow_Multiple_Connection(ON);
                srvState = Server_Mux_Waiting;           
                break;
            }
            case Server_Mux_Waiting :
            {   
                if(!strcmp(response2, "OK"))
                {
                    Reset_Timer(wait_wifi);
                    srvState = Server_Mux_Done;
                }    
                else if(Wait_MilliSec(wait_wifi,2000))
                {
                    srvState = Server_Mux;
                }   
                break;
            } 
            case Server_Mux_Done :
            {   
                if(Wait_MilliSec(wait_wifi,500))
                {
                    srvState = Server_Create;
                }    
                break;
            } 
            case Server_Create:
            {
                Server_TCP_Connection(Create_server, port);
                srvState = Server_Done;
                break;
            }
            case Server_Done:
            {                   
                if(!strcmp(response2, "OK"))
                {    
                    srvState = Server_Created;
                    Reset_Timer(wait_wifi);
                }
                else if(Wait_MilliSec(wait_wifi,2000))
                {
                    srvState = Server_Create;
                }    
                break;
            }
            case Server_Created :
            {
                srvState = Set_IP_Adress;
                CONTROL_BITS2.tcp_server_created = true;
                return true;
            }    
        } 
        return false;
    }    
}

bool Data_Send_as_AP(uint8_t id, uint8_t length, char data[])
{
    static WIFI_STATES ds_states = Pending;
    
    if(CONTROL_BITS3.wifi_engine_enabled)
    {
        switch (ds_states)
        {
            case Pending:
            {
                CONTROL_BITS2.wifi_data_sending = true;
                Prepare_to_Send_Data_as_AP(id, length);
                ds_states = Executing;
                break;
            }
            case Executing:
            {
                if(Wait_MilliSec(wait_wifi,500))  
                {
                    uint16_t i;
                    wifiSend = true;
                    for(i = 0; i < length; i++)
                        mikropData.usart2BQTxData[i] = data[i];

                    
                    mikropData.usart2BQTxData[i + 1] =  '\r';
                    mikropData.usart2BQTxData[i + 2] =  '\n';
                    USART2_TX_SIZE = length + 2;
                    
                  
                    ds_states = Waiting;
                }
                break;
            }
            case Waiting:
            {
                if(Wait_MilliSec(wait_wifi,100))
                {
                    ds_states = Done;
                }
                break;
            }
            case Done:
            {   
                CONTROL_BITS2.wifi_data_sending = false;
                ds_states = Pending;
                return true;
            }
        } 
        return false;
    }    
}

/***** Client Functions *****/

/* These are the functions that can be used to be client of any esp8266 server. Also the functions that can be used only after being client
are here*/

bool Create_Client(char * ssid, char * pwd)
{
    static CLIENT_STATES Client_States = Client_Create;
    
    if(CONTROL_BITS3.wifi_engine_enabled)
    {
        switch(Client_States)
        {
            case Client_Create :
            {
                if(Wait_MilliSec(wait_wifi,500))
                {    
                    SetWifi_Mode(wifi_mode_client);   
                    Client_States = Client_Wait;
                }
                break;
            }    
            case Client_Wait :
            {
                if(!strcmp(response2, "OK"))
                {
                    Reset_Timer(wait_wifi);
                    Client_States = Connect_AP;
                }
                else if(Wait_MilliSec(wait_wifi,2000))
                {
                    Client_States = Client_Create;
                }   
                break;    
            }    
            case Connect_AP :
            {
                if(Wait_MilliSec(wait_wifi, 500))
                {    
                    Connect_to_AP(ssid, pwd); 
                    Client_States = Connection_Wait;
                }
                break;
            }    
            case Connection_Wait:
            {  
                if(!strcmp(response2, "OK")) 
                {
                    Client_States = Client_Create_Done;
                    Reset_Timer(wait_wifi);
                }
                else if(Wait_MilliSec(wait_wifi, 2000))
                {
                    Client_States = Connect_AP;
                }    
                break;
            } 

            case Client_Create_Done :
            {
                CONTROL_BITS2.tcp_client_created = true;
                Client_States = Client_Create;
                return true;
            }   
        }        
        return false;
    }    
    return false;
}

bool TCP_Connection(char * ip, char * port)
{
    static WIFI_STATES tcpStates = Pending;
   
    switch(tcpStates)
    {
        case Pending :
        {
            if(CONTROL_BITS3.wifi_engine_enabled)
                tcpStates = Executing;
            break;
        }
        case Executing :
        {
            if(Wait_MilliSec(wait_wifi,500))
            { 
                TCP_Connection_to_AP(ip, port);
                tcpStates = Waiting;
            }
            break;
        }   
        case Waiting :
        {
            if(!strcmp(response2, "OK")) 
            {
                tcpStates = Done;
                Reset_Timer(wait_wifi);
            }
            else if(Wait_MilliSec(wait_wifi, 2000))
            {
                tcpStates = Executing;
            }    
            break;
        }   
        case Done :
        {
            tcpStates = Pending;
            return true;
        }   
    }
    return false;    
}

bool Data_Send_as_Client(uint8_t length, char * data)
{
    static WIFI_STATES ds_states = Pending;
   
    switch (ds_states)
    {
        case Pending:
        {
            if(Wait_MilliSec(wait_wifi,1000))  
            {    
                CONTROL_BITS2.wifi_data_sending = true;
                Prepare_to_Send_Data_as_Client(length);
                ds_states = Executing;
            }
            break;
        }
        case Executing:
        {
            if(Wait_MilliSec(wait_wifi,500))  
            { 
                uint8_t i;
                wifiSend = true;
                for(i = 0; i < length; i++)
                    mikropData.usart2BQTxData[i] = data[i];

                mikropData.usart2BQTxData[i + 1] =  '\r';
                mikropData.usart2BQTxData[i + 2] =  '\n';
                USART2_TX_SIZE = length + 2;
                ds_states = Waiting;
            }
            break;
        }
        case Waiting:
        {
            if(Wait_MilliSec(wait_wifi,500))
            {
                ds_states = Done;
            }
            break;
        }
        case Done:
        {   
            CONTROL_BITS2.wifi_data_sending = false;
            ds_states = Pending;
            return true;
        }
    } 
    return false;
}


/*****  Network Functions *****/

/* These functions are created by using Wifi functions and provide a local network among at most 255 Esp8266 wifi modules */

void Add_Data_Client_Buffer(uint8_t range_begin, uint8_t range_end, char * data)
{
    uint8_t i;
  
    for(i = range_begin; i <= range_end; i++)
    {    
        if(!strcmp(Client_Data_Buffer[i].data1, "EMPTY"))
        {
            Client_Data_Buffer[i].data1 = data;
        }    
        else if(!strcmp(Client_Data_Buffer[i].data2, "EMPTY"))
        {
            Client_Data_Buffer[i].data2 = data;
        }    
        else if(!strcmp(Client_Data_Buffer[i].data3, "EMPTY"))
        {
            Client_Data_Buffer[i].data3 = data;
        }   
        Client_Data_Buffer[i].empty = false;
    }                                  
}

void Client_Buffer_Empty()
{
    uint16_t i;
    for(i = 1; i < CDB_SIZE; i++)
    {    
        Client_Data_Buffer[i].empty = true;
        Client_Data_Buffer[i].data1 = "EMPTY";
        Client_Data_Buffer[i].data2 = "EMPTY";
        Client_Data_Buffer[i].data3 = "EMPTY";
    }    
}


void Join_Network() 
{                   
    static NETWORK_STATES nw_states = Network_Wait;
    static uint8_t server_id = 0;
    static uint8_t server_id_tmp;
    static uint8_t server_id_tmp2;
    static uint8_t server_id_digit_number;
    static char server_ssid[9] = "MIKROP";
    static char server_ip[15] = "192.168.";
    static uint8_t data_length;    
    static bool client_flag = false, server_flag = false;
    uint8_t i = 0;
    
    switch (nw_states)
    {
        case Network_Wait:
        {
            if(CONTROL_BITS3.wifi_engine_enabled)
            {
                if(server_id = IsAnyClientData())
                {    
                   client_flag = true;
                   nw_states = Network_Change_Status;
                   break;
                }   
                else if (!CONTROL_BITS2.tcp_server_created)
                {
                   server_flag = true; 
                   nw_states = Network_Change_Status;
                   break;
                }   
                else
                {
                    nw_states = Control;
                    break;
                }
            }
            else
                return;
        }
        case Network_Change_Status:
        {
            if(Wifi_Restore())
            {    
                if(client_flag)
                    nw_states = Network_Client;
                else
                    nw_states = Network_Server;
            }    
            break;     
        }
        case Network_Client:
        {
            server_id_tmp = server_id;
            server_id_tmp2 = server_id_tmp;
            server_id_digit_number = number_of_digits(server_id_tmp);
            for(i = 5 + server_id_digit_number ; i >= 6; i--)
            {    
                server_ssid[i] = (server_id_tmp2 % 10) + '0';
                server_id_tmp2 /= 10; 
            }    
            ssid_length = 6 + server_id_digit_number;
           
            nw_states = Network_Client_Wait;
            break;
        }
        case Network_Client_Wait:
        {
            if(Create_Client(server_ssid, ""))
            {
                Reset_Timer(wait_wifi2);
                ssid_length = 0;
                nw_states = Network_Client_TCP_Connection;
            }  
            else if (Wait_MilliSec(wait_wifi2, 15000))
            {
                Reset_Timer(wait_wifi);
                Client_Data_Buffer[server_id].data1 = "EMPTY";
                Client_Data_Buffer[server_id].data2 = "EMPTY";
                Client_Data_Buffer[server_id].data3 = "EMPTY";
                Client_Data_Buffer[server_id].empty = true;
                ssid_length = 0;
                client_flag = false;
                nw_states = Network_Wait;
            }    
            break;
        }
        case Network_Client_TCP_Connection:
        {
            for(i = 7 + server_id_digit_number; i >= 8; i--)
            {    
                server_ip[i] = (server_id_tmp % 10) + '0';
                server_id_tmp /= 10; 
            }    
            server_ip[8 + server_id_digit_number] = '.';
            server_ip[9 + server_id_digit_number] = '1';
           
            ip_length = 10 + server_id_digit_number;
            nw_states = Network_Client_TCP_Connection_Wait;
            break;
        }
        case Network_Client_TCP_Connection_Wait:
        {
            if(TCP_Connection(server_ip, "458")) 
            {
                Reset_Timer(wait_wifi2);
                ip_length = 0;
                nw_states = Network_Client_Data_Send;
            }    
            else if (Wait_MilliSec(wait_wifi2, 15000))
            {
                Reset_Timer(wait_wifi);
                Client_Data_Buffer[server_id].data1 = "EMPTY";
                Client_Data_Buffer[server_id].data2 = "EMPTY";
                Client_Data_Buffer[server_id].data3 = "EMPTY";
                Client_Data_Buffer[server_id].empty = true;
                ip_length = 0;
                client_flag = false;
                nw_states = Network_Wait;
            }    
            break;
        }
        
        case Network_Client_Data_Send:
        { 
            if(strcmp(Client_Data_Buffer[server_id].data1, "EMPTY"))
            {    
                data_length = strlen(Client_Data_Buffer[server_id].data1);
                nw_states = Network_DATA1;
            }    
            else if(strcmp(Client_Data_Buffer[server_id].data2, "EMPTY"))
            {    
                data_length = strlen(Client_Data_Buffer[server_id].data2);
                nw_states = Network_DATA2;
            }    
            else if(strcmp(Client_Data_Buffer[server_id].data3, "EMPTY"))
            {
                data_length = strlen(Client_Data_Buffer[server_id].data3);
                nw_states = Network_DATA3;
            }
            else
               nw_states = Network_NODATA; 
            break;
        }
        
        case Network_DATA1:
        {
            if(Data_Send_as_Client(data_length, Client_Data_Buffer[server_id].data1))
            {    
                Client_Data_Buffer[server_id].data1 = "EMPTY";
                if(strcmp(Client_Data_Buffer[server_id].data2, "EMPTY"))
                {    
                    data_length = strlen(Client_Data_Buffer[server_id].data2);
                    nw_states = Network_DATA2;
                }    
                else if(strcmp(Client_Data_Buffer[server_id].data3, "EMPTY"))
                {    
                    data_length = strlen(Client_Data_Buffer[server_id].data3);
                    nw_states = Network_DATA3;
                }    
                else
                    nw_states = Network_NODATA;
            }    
            break;
        }
        case Network_DATA2:
        {
            if(Data_Send_as_Client(data_length, Client_Data_Buffer[server_id].data2))
            {    
                Client_Data_Buffer[server_id].data2 = "EMPTY";
                if(strcmp(Client_Data_Buffer[server_id].data3, "EMPTY"))
                {    
                    data_length = strlen(Client_Data_Buffer[server_id].data3);
                    nw_states = Network_DATA3;
                }    
                else
                    nw_states = Network_NODATA; 
            }    
            break;
        }
        case Network_DATA3:
        {
            if(Data_Send_as_Client(data_length, Client_Data_Buffer[server_id].data3))
            {
                Client_Data_Buffer[server_id].data3 = "EMPTY";
                nw_states = Network_NODATA;  
            }    
            break;
        }
        case Network_NODATA:
        {
            Client_Data_Buffer[server_id].empty = true;
            client_flag = false;
            server_flag = false;
            nw_states = Network_Wait;
            break;
        }
        case Network_Server:
        {
            server_id_tmp = MIKROPID;
            server_id_tmp2 = server_id_tmp;
            server_id_digit_number = number_of_digits(server_id_tmp);
            for(i = 5 + server_id_digit_number ; i >= 6; i--)
            {    
                server_ssid[i] = (server_id_tmp2 % 10) + '0';
                server_id_tmp2 /= 10; 
            }    
          
            for(i = 7 + server_id_digit_number; i >= 8; i--)
            {    
                server_ip[i] = (server_id_tmp % 10) + '0';
                server_id_tmp /= 10; 
            }    
            server_ip[8 + server_id_digit_number] = '.';
            server_ip[9 + server_id_digit_number] = '1';
            
            ssid_length = 6 + server_id_digit_number;
            ip_length = 10 + server_id_digit_number;
            nw_states = Network_Server_Wait;
            break;
            
        }
        case Network_Server_Wait:
        {
            if(Create_TCP_Server(server_ip, server_ssid, "", "458")) 
            {
                ssid_length = 0;
                ip_length = 0;
                client_flag = false;
                server_flag = false;
                nw_states = Control;
            }    
            break;
        }
        case Control:
        {
            if(Wait_MilliSec(wait_irda, 10000))
            {
                nw_states = Network_Wait;
                break;
            }
        }  
    }    
}


