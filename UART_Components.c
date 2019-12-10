/* 
      This file includes the functions of IRDA and WIFI modules who use UART communication protocol.
      
*/

/***** IRDA Functions ****/

void IRDA_Port_Select(uint8_t irdaport) // Since it's a system having 3 IRDA ports, this function is used for select the port number to use for infrared data transfer among the robots
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

void IRDA_Port_Select_Auto(uint8_t irdaports, uint16_t time_period) //Makes the IRDA port selection automatically
{
    CONTROL_BITS.autoports = irdaports;
    autoport_period = time_period;
}

uint8_t Transmit(uint8_t dest, char datatype1, char datatype2, uint16_t irda_data, uint16_t time, uint8_t pac_imp, uint8_t br_ttl) // Add the package to transmit into the Transmit_Buffer. So the packages are transmitted in order
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

void USART1R_Task (void) //The read task fonction of IRDA mechanism. With these function the robot uses the UART Bus for communicate with microprocessor and read the infrared messages in the environment 
{
    switch (usart1BQRState)
    {
        case USART_INIT:    
        {
            STATUS_FLAGS.usart1rtasks = 1;
            isusart1rconfigured = false;
            mikropData.usart1BQRxRead = 0;
            usart1BQRState = USART_WORKING;
            if(!U1STAbits.URXEN) 
            { 
                irdar_reset_counter = 0;
                U1STAbits.URXEN = 1;
                usart1BQRState = USART_INIT;
            }    
            break;
        }

        case USART_WORKING:
        { 
            /* Has the RX buffer been read by the USART driver and transferred to the application by the BufferQueueEventHandler function? */
            if (mikropData.usart1BQRxBufferHandle == DRV_USART_BUFFER_HANDLE_INVALID)
            {
                /* Process this buffer */
                if (mikropData.usart1BQRxRead == 0)
                {
                    DRV_USART_BufferAddRead(mikropData.handleUSART1,
                        (DRV_USART_BUFFER_HANDLE * const)&mikropData.usart1BQRxBufferHandle,
                        &mikropData.usart1BQRxData[0], PACKET_SIZE * 2);
                        IRDA_READ = IRDA_RD_PENDING;
                }
            }
 
           
            if(mikropData.usart1BQRxRead == PACKET_SIZE * 2)
            {     
                irdar_reset_counter = 0;
                IRDA_READ = IRDA_RD_DONE;
                static uint8_t pi1;
                if (mikropData.usart1BQRxData[0] != 'A') 
                {
                    if ((pi1 = IsAnyPI()) !=  PACKET_SIZE * 2 && pi1 + PACKET_SIZE < PACKET_SIZE * 2)
                    {  
                        if (IsSignificant_IRDA_Package(mikropData.usart1BQRxData[pi1 + 3]))
                        {
                            uint8_t i;
                            for (i = 0; i < PACKET_SIZE; i++) 
                            {    
                                mikropData.usart1BQRxData[i] = mikropData.usart1BQRxData[pi1 + i];
                            }
                            isreaddatanull = false;
                        }
                        else 
                          USARTBQRX_Empty();  
                    }
                    else 
                        USARTBQRX_Empty();
                }
            }    
          
                        
            if (IRDA_READ == IRDA_RD_DONE)
            {
                if (mikropData.usart1BQRxData[2] == MIKROPID)
                    USARTBQRX_Empty();       
                usart1BQRState = USART_DONE;    
            }
            else if (U1STAbits.RIDLE && !U1STAbits.URXDA)
            {
                STATUS_FLAGS.usart1rtasks = 0;
                irdar_reset_counter++;

                if(irdar_reset_counter == IRDAR_RESET)
                {
                    CONTROL_BITS2.wait_for_de = 0;
                    CONTROL_BITS.transmit_restriction = 0;
                    STATUS_FLAGS.usart1rtasks = 0;
                    U1STAbits.URXEN = 0; // 0 = UARTx receiver is disabled. UxRX pin is ignored by the UARTx module. UxRX pin is controlled by port.
                    U1STAbits.OERR = 0; //This bit is set in hardware and can only be cleared (= 0) in software. Clearing a previously set OERR bit resets the receiver buffer and the RSR to an empty state.  
                    USARTBQRX_Empty();
                    protocol = DATA_Wait;
                    Receiver_Transmitter = Receiver;
                    usart1BQRState = USART_INIT;
                    break;
                }
            }
                
            break;
        }

        case USART_DONE:
        {
            STATUS_FLAGS.usart1rtasks = 0;
            usart1BQRState = USART_INIT;
            isusart1rconfigured = true;
            break;
        }
    }
}


void USART1W_Task (void) //The write task fonction of IRDA mechanism. With these function the robot uses the UART Bus for communicate with microprocessor and send the packages in the Transmit_Buffer as the infrared messages in the environment 
{
    switch (usart1BQWState)
    {
        default:
        case USART_INIT:    
        {
            if(!U1STAbits.UTXEN)
            { 
                U1STAbits.UTXEN = 1;
            }
            STATUS_FLAGS.usart1wtasks = 1;
            isusart1wconfigured = false;
            mikropData.usart1BQTxWritten = 0;
            usart1BQWState = USART_WORKING;
            break;
        }

        case USART_WORKING:
        {
            
            /* Check to see if the Application owns the TX buffer */
            if (mikropData.usart1BQTxBufferHandle == DRV_USART_BUFFER_HANDLE_INVALID)
            {

                /* If we have data to transmit, then queue the buffer */
                if (mikropData.usart1BQTxWritten == 0)
                {   
                    irdaw_reset_counter++;
                    if (protocol == OK || protocol == DATA_Send)
                    {   
                        IRDA_WRITE = IRDA_WR_PENDING;
                        if(U1STAbits.RIDLE || (!(!U1STAbits.RIDLE && U1STAbits.TRMT && !PRSENSOR_ENABLE.pen))) //ridle yoksa, varsa da benden kaynakl?ysa
                            DRV_USART_BufferAddWrite(mikropData.handleUSART1,
                            (DRV_USART_BUFFER_HANDLE * const)&mikropData.usart1BQTxBufferHandle,
                            &mikropData.usart1BQTxData[0], PACKET_SIZE * 2);

                    }
                    else if (FIFO_size(IRDA) && irdaw_reset_counter == IRDAW_RESET)
                    {
                        U1STAbits.UTXEN = 0;
                        STATUS_FLAGS.usart1wtasks = 0;
                        usart1BQWState = USART_INIT;
                        FIFO_Empty(IRDA);
                        protocol = DATA_Wait;
                        Receiver_Transmitter = Receiver;
                        USARTBQTX_Empty();
                        break;
                    }        
                }
            }
             
            if (mikropData.usart1BQTxWritten == PACKET_SIZE * 2)
            {
                irdaw_reset_counter = 0; 
                IRDA_WRITE = IRDA_WR_DONE;
                if (protocol == DATA_Send)
                {    
                    protocol = OK_Wait;
                }    
                if (protocol == OK)
                {    
                    protocol = DATA_Wait;
                }    
            }
            
           
            
            if (IRDA_WRITE == IRDA_WR_DONE)
            {
                usart1BQWState = USART_DONE;
   
            }
            break;
        }
        case USART_DONE:
        {
            isusart1wconfigured = true;
            STATUS_FLAGS.usart1wtasks = 0;
            break;
        }
    }
}



/***** WIFI Functions *****/

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

/***** Server Functions *****/ 

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
