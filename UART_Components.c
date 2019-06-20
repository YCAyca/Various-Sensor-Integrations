/* 
      This file includes the functions of IRDA and WIFI modules who use UART communication protocol instead of I2C.
      
      


*/

/***** IRDA Functions ****/

void IRDA_Port_Select(uint8_t irdaport) # Since it's a system having 3 IRDA ports, this function is used for select the port number to use for infrared data transfer among the robots
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

void IRDA_Port_Select_Auto(uint8_t irdaports, uint16_t time_period) #Makes the IRDA port selection automatically
{
    CONTROL_BITS.autoports = irdaports;
    autoport_period = time_period;
}

uint8_t Transmit(uint8_t dest, char datatype1, char datatype2, uint16_t irda_data, uint16_t time, uint8_t pac_imp, uint8_t br_ttl) # Add the package to transmit into the Transmit_Buffer. So the packages are transmitted in order
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

void USART1R_Task (void) #The read task fonction of IRDA mechanism. With these function the robot uses the UART Bus for communicate with microprocessor and read the infrared messages in the environment 
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


void USART1W_Task (void) #The write task fonction of IRDA mechanism. With these function the robot uses the UART Bus for communicate with microprocessor and send the packages in the Transmit_Buffer as the infrared messages in the environment 
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
