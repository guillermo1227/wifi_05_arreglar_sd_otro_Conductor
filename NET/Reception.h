/**********************************************************************************
 * Copyright (C) 2018-2021 LASECÂ®ï¸� Telecomunicaciones S.A.P.I. de C.V.
* All rights reserved.
*
* This document is the property of LASECÂ®ï¸� Telecomunicaciones S.A.P.I. de C.V.
* It is considered confidential and proprietary.
*
* This document may not be reproduced or transmitted in any form,
* in whole or in part, without the express written permission of
* LASECÂ®ï¸� Telecomunicaciones S.A.P.I. de C.V.
*
*********************************************************************************/

#ifndef _Reception_H
#define _Reception_H

#define TCP_NUMBER      6


uint8_t count_tcp=0;
uint8_t count_stream=0;
/*************** Tcp Configurator Thread ***************/
/* Thread to publish data to the cloud */

void Main_Thread_TCP(wiced_thread_arg_t arg){
    uint8_t  key=1;

    uint8_t _send_file;

    while (1){
        wiced_rtos_lock_mutex(&pubSubMutex);

//        printf("%d\n",key);


        switch(key){
            case 1:
//                tcp_file_sys();
//                send_request_date();
                key=tcp_client_aca();
                break;

            #ifdef GEOLOC
            case 2:
                key=tcp_client_geo();
                break;
            case 3:
                key=tcp_gateway();
                break;
             #else
            case 2:
                key=tcp_gateway();
                break;
             #endif


        }


        wiced_rtos_unlock_mutex(&pubSubMutex);

    }
}



void send_request_date()
{
    wiced_tcp_socket_t socket;                      // The TCP socket
    wiced_tcp_stream_t stream;                      // The TCP stream
    char sendMessage[140];
    wiced_result_t result;
    wiced_ip_address_t INITIALISER_IPV4_ADDRESS( server_ip_address, s1 );
    wiced_mac_t myMac;
    wiced_ip_address_t myIpAddress;
    wl_bss_info_t ap_info_buffer;
    wiced_security_t ap_security;

    // Open the connection to the remote server via a socket
    result = wiced_tcp_create_socket(&socket, WICED_STA_INTERFACE);
    if(result!=WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("Failed to create socket %d\n",result));
        return;
    }
    else {
        printf("ok1\n");
    }

    result = wiced_tcp_bind(&socket,WICED_ANY_PORT);
    if(result!=WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("Failed to bind socket %d\n",result));
        wiced_tcp_delete_socket(&socket);
        return;
    }
    else {
        printf("ok2\n");
    }


    result = wiced_tcp_connect(&socket,&server_ip_address,TCP_SERVER_PORT,TCP_CLIENT_CONNECT_TIMEOUT); // 2 second timeout
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "Failed connect = [%d]\n", result ));
        wiced_tcp_delete_socket(&socket);
        wiced_rtos_set_semaphore(&tcpGatewaySemaphore);
        return;
    }

    wiced_wifi_get_mac_address(&myMac);                             // Se obtiene la MAC del dispositivo
    wiced_ip_get_ipv4_address( WICED_STA_INTERFACE, &myIpAddress);  // Se obtiene la IP del dispositivo
    wwd_wifi_get_ap_info(&ap_info_buffer, &ap_security);            // Se obtiene la MAC de la red a la que estamos conectados
    /*Ectract data to peripherals*/



    sprintf(mac_wifi,"%02X:%02X:%02X:%02X:%02X:%02X",myMac.octet[0],myMac.octet[1],myMac.octet[2],myMac.octet[3],myMac.octet[4],myMac.octet[5]);
    sprintf(mac_ap,"%02X:%02X:%02X:%02X:%02X:%02X",ap_info_buffer.BSSID.octet[0], ap_info_buffer.BSSID.octet[1],ap_info_buffer.BSSID.octet[2],ap_info_buffer.BSSID.octet[3],ap_info_buffer.BSSID.octet[4],ap_info_buffer.BSSID.octet[5]);
    sprintf(ip,"%u.%u.%u.%u", (uint8_t)(GET_IPV4_ADDRESS(myIpAddress) >> 24),(uint8_t)(GET_IPV4_ADDRESS(myIpAddress)>> 16),(uint8_t)(GET_IPV4_ADDRESS(myIpAddress) >> 8),(uint8_t)(GET_IPV4_ADDRESS(myIpAddress) >> 0));

    sprintf(sendMessage,"\nH;%s,%s,%s,%s,%s\r\n",mac_wifi,mac_ap,ip,time_get(&i2c_rtc),date_get_log(&i2c_rtc));


    // Format the data per the specification in section 6
//    sprintf(sendMessage,"\nhola\r\n"); // 5 is the register from the lab manual
    WPRINT_APP_INFO(("Sent Message=%s\n",sendMessage)); // echo the message so that the user can see something

    // Initialize the TCP stream
    wiced_tcp_stream_init(&stream, &socket);

    // Send the data via the stream
    wiced_tcp_stream_write(&stream, sendMessage, strlen(sendMessage));
    // Force the data to be sent right away even if the packet isn't full yet
    wiced_tcp_stream_flush(&stream);


    // Get the response back from the WWEP server
      char rbuffer[49] = {0}; // The first 11 bytes of the buffer will be sent by the server. Byte 12 will stay 0 to null terminate the string
      result = wiced_tcp_stream_read(&stream, rbuffer, 49, 4500); // Read 11 bytes from the buffer - wait up to 500ms for a response
      if(result == WICED_SUCCESS)
      {
          WPRINT_APP_INFO(("Server Response=%s\n",rbuffer));
      }
      else
      {
          WPRINT_APP_INFO(("Malformed response\n"));
      }
      char _cresult [23];
      memset(_cresult,NULL,23);
      if(strstr(rbuffer,"T:")){
        strncpy(_cresult,strstr(rbuffer,"T:"),21);
//          T:24/04/2023-13:51:06
          split_date_endpoint(&_cresult,strlen(_cresult));
          time_set(aux_time,&i2c_rtc);
          date_set(aux_date_y,&i2c_rtc);

        printf(">%s\n",_cresult);
//          printf("Aqui ira la funcion para setear la hora:  posicion %s \n",strstr(rbuffer,"T:"));
      }
    // Delete the stream and socket
    wiced_tcp_stream_deinit(&stream);
    wiced_tcp_delete_socket(&socket);
    wiced_rtos_set_semaphore(&tcpGatewaySemaphore);

}

int tcp_gateway( void ){
    send_data_task=WICED_TRUE;

    int state=0;
    wiced_mac_t myMac;
    wiced_tcp_socket_t socket;                      // The TCP socket
    wiced_tcp_stream_t stream;                      // The TCP stream
    char sendMessage[80];
    wiced_result_t result;
    wiced_packet_t* tx_packet;
    uint8_t *tx_data;
    uint16_t available_data_length;

    wiced_ip_address_t INITIALISER_IPV4_ADDRESS( ip_address, s1);

        wiced_ip_address_t INITIALISER_IPV4_ADDRESS( server_ip_address, s1 );


        WPRINT_APP_INFO(("Event Thread Tcp client\n"));
//        WPRINT_APP_INFO( ("Count of bluetooth with mac sssssssss -> %d \r\n",s_count_x) );


        result = wiced_tcp_create_socket(&socket, WICED_STA_INTERFACE);

        if ( result != WICED_SUCCESS )
        {
            try_n=try_n+1;
            if(try_n==TCP_DOWN_NUMBER){
//                   set_name();
                check_sound_onoff();
                 wiced_rtos_delay_milliseconds(100);
//                     set_name();
               wiced_framework_reboot();
            }

        }
        else{
//            try_n=0;
            WPRINT_APP_INFO(("e  falied 1\n"));

        }

        result = wiced_tcp_bind( &socket, WICED_ANY_PORT ); /* Poner any port para que actualice el puerto de manera automatica */
        if(result!=WICED_SUCCESS)
        {
            try_n=try_n+1;
            if(try_n==TCP_DOWN_NUMBER){
//                   set_name();
                check_sound_onoff();

                 wiced_rtos_delay_milliseconds(100);

               wiced_framework_reboot();
            }

            WPRINT_APP_INFO(("falied 2\n"));

            wiced_tcp_delete_socket(&socket); /* Delete socket and return*/
        }
        else{
            try_n=0;
            WPRINT_APP_INFO((" e falied 2\n"));

        }

        result = wiced_tcp_connect(&socket,&server_ip_address,TCP_SERVER_PORT,2500); // 2 second timeout
        if ( result != WICED_SUCCESS )
        {
            try_n=try_n+1;
            if(try_n==TCP_DOWN_NUMBER){
////                   set_name();
                check_sound_onoff();
                     wiced_rtos_delay_milliseconds(100);


               wiced_framework_reboot();
            }

            WPRINT_APP_INFO(("falied 3\n"));

            wiced_tcp_delete_socket(&socket);

        }
        else{
            try_n=0;
            WPRINT_APP_INFO(("  e  falied 3\n"));

        }

        WPRINT_APP_INFO(("try %d\n",try_n));


        wiced_ip_address_t myIpAddress;
        wl_bss_info_t ap_info_buffer;
        wiced_security_t ap_security;
        // Format the data per the specification in section 6
        wwd_wifi_get_ap_info(&ap_info_buffer, &ap_security);            // Se obtiene la MAC de la red a la que estamos conectados
        wiced_ip_get_ipv4_address( WICED_STA_INTERFACE, &myIpAddress);  // Se obtiene la IP del dispositivo
        wiced_wifi_get_mac_address(&myMac);                             // Se obtiene la MAC del dispositivo



        sprintf(mac_wifi,"%02X:%02X:%02X:%02X:%02X:%02X",myMac.octet[0],myMac.octet[1],myMac.octet[2],myMac.octet[3],myMac.octet[4],myMac.octet[5]);

        sprintf(mac_ap,"%02X:%02X:%02X:%02X:%02X:%02X",ap_info_buffer.BSSID.octet[0], ap_info_buffer.BSSID.octet[1],ap_info_buffer.BSSID.octet[2],ap_info_buffer.BSSID.octet[3],ap_info_buffer.BSSID.octet[4],ap_info_buffer.BSSID.octet[5]);
        sprintf(ip,"%u.%u.%u.%u", (uint8_t)(GET_IPV4_ADDRESS(myIpAddress) >> 24),(uint8_t)(GET_IPV4_ADDRESS(myIpAddress)>> 16),(uint8_t)(GET_IPV4_ADDRESS(myIpAddress) >> 8),(uint8_t)(GET_IPV4_ADDRESS(myIpAddress) >> 0));

        // Initialize the TCP stream
        wiced_tcp_stream_init(&stream, &socket);
//
//        for(int f=0;f<100;f++){
//            memcpy(data_btt[f].mac_bt,"01:01:01:01:01:01",17);
//            memcpy(data_btt[f].rssi,"-85",5);
//            memcpy(data_btt[f].fallen,"0",2);
//        }
//
////
////
//        s_count_x=100;
//        uint8_t coun;

              if((s_count_x<=limit_data)){
                  WPRINT_APP_INFO(("Multiple Tcp client\n"));
                  if(s_count_x==limit_data){
                      data_send_bt=limit_data;
      //                count_bt=0;
                  }
                  else if(s_count_x<limit_data){
                      data_send_bt=s_count_x;
      //                count_bt=0;
                  }
                  if(s_count_x!=0){
                      WPRINT_APP_INFO( (">> es igual a %d en %s\n\n\n",s_count_x,mac_ap) );

                      for(int f=0;f<data_send_bt;f++){
                          if(f==0){
      //                    sprintf(data_out,"\nV;%s,%s,%s,%s,%s\r\n",mac_wifi,mac_ap,ip,time_get(&i2c_rtc),date_get(&i2c_rtc));
//                              sprintf(data_out,"\nH;%s,%s,%s,%s,%s\r\n",mac_wifi,mac_ap,ip,time_get(&i2c_rtc),date_get_log(&i2c_rtc));
                              result=wiced_tcp_stream_write(&stream, data_out, strlen(data_out));
                              memcpy(data_btt[f].mac_bt,NULL,17);
                                memcpy(data_btt[f].type,NULL,17);
                                memcpy(data_btt[f].rssi,NULL,4);
                                memcpy(data_btt[f].fallen,NULL,2);
                              if(result==WICED_TCPIP_SUCCESS){
                                  wiced_uart_transmit_bytes(WICED_UART_1,(("%s",data_out)),strlen(data_out));
                                  send_data_task=WICED_TRUE;
                              }
                          }
                          else{
                          wiced_rtos_delay_microseconds( 10 );
                          sprintf(data_out,"\nB;%s,%s,%s,%s,%s,%s\r\n",mac_ap,data_btt[f].mac_bt,mac_wifi,data_btt[f].type,data_btt[f].rssi,data_btt[f].fallen);
//                          sprintf(data_out,"\nB;%s,%s,%s,%s\r\n",mac_ap,data_btt[f].mac_bt,mac_wifi,data_btt[f].rssi);

                                          memcpy(data_btt[f].mac_bt,NULL,17);
                                          memcpy(data_btt[f].type,NULL,17);
                                          memcpy(data_btt[f].rssi,NULL,4);
                                          memcpy(data_btt[f].fallen,NULL,2);


                          result=wiced_tcp_stream_write(&stream, data_out, strlen(data_out));
                          if(result==WICED_TCPIP_SUCCESS){
//                              wiced_uart_transmit_bytes(WICED_UART_1,(("%s",data_out)),strlen(data_out));
                              send_data_task=WICED_TRUE;
//                              return 1;
                           }
                          }
                      }
                      s_count_x=0;
                      data_send_bt=0;
                  }
                  else{
      //                sprintf(data_out,"\nV;%s,%s,%s,%s,%s\r\n",mac_wifi,mac_ap,ip,time_get(&i2c_rtc),date_get(&i2c_rtc));
                      sprintf(data_out,"\nL;%s,1600,%d%d%d%d0000000000000,%s,%s,%s,%s\r\n",mac_wifi,t1,t2,t3,t4,mac_ap,ip,time_get(&i2c_rtc),date_get(&i2c_rtc));
//                      sprintf(data_out,"\nH;%s,%s,%s,%s,%s\r\n",mac_wifi,mac_ap,ip,time_get(&i2c_rtc),date_get_log(&i2c_rtc));
//                      sprintf(data_out,"\nHVT:%s\r\n",data_to_json_acarreo(&log_accarreos,s_Mac_W));

                      result=wiced_tcp_stream_write(&stream, data_out, strlen(data_out));
                         if(result==WICED_TCPIP_SUCCESS){
                             wiced_uart_transmit_bytes(WICED_UART_1,(("%s",data_out)),strlen(data_out));
                             send_data_task=WICED_TRUE;
                          }

                  }

              }

//              memset(sendMessage,NULL,80);
              memset(data_out,NULL,1000);
//              key=1;
        wiced_tcp_stream_flush(&stream);

        wiced_rtos_delay_milliseconds( 1500 );

        // Delete the stream and socket
        wiced_tcp_stream_deinit(&stream);
        wiced_tcp_delete_socket(&socket);
        return 1;


}

int tcp_client_aca( )
{

    uint8_t state=0;

//    wiced_rtos_lock_mutex(&pubSubMutex);

    send_data_task=WICED_TRUE;
    send_data_task=WICED_TRUE;

    WPRINT_APP_INFO(("Event Thread Tcp client\n"));
//        counter_tcp_guardian=0;
        char* resultado;
        wiced_result_t result;
        wiced_mac_t myMac;
        wiced_ip_address_t myIpAddress;
        wl_bss_info_t ap_info_buffer;
        wiced_security_t ap_security;

        wiced_packet_t*          packet;
        wiced_packet_t*          rx_packet;
        unsigned char*                    tx_data;
        unsigned char*                    rx_data;
        uint16_t                 rx_data_length;
        uint16_t                 available_data_length;
        int                      connection_retries;

             wiced_ip_address_t INITIALISER_IPV4_ADDRESS( server_ip_address, s1 );

             /* Connect to the remote TCP server, try several times */
             connection_retries = 0;
             do
             {
                 result = wiced_tcp_connect( &tcp_client_socket, &server_ip_address, TCP_SERVER_PORT, TCP_CLIENT_CONNECT_TIMEOUT );
                 connection_retries=connection_retries+1;
             }
             while( ( result != WICED_SUCCESS ) && ( connection_retries < TCP_CONNECTION_NUMBER_OF_RETRIES ) );
             if ( result != WICED_SUCCESS )
                {
                WPRINT_APP_INFO(("Unable to connect to the server! Halt.\n"));
                try_n=try_n+1;
                if(try_n==TCP_DOWN_NUMBER){
    //                check_satat_gpio();

                        wiced_framework_reboot();
                }
                wiced_uart_transmit_bytes(WICED_UART_1,TCP_CONNECT_FAILED,sizeof(TCP_CONNECT_FAILED)-1);
//                return WICED_ERROR;

             }
             else{
                 try_n=0;
             }
             /* Create the TCP packet. Memory for the tx_data is automatically allocated */
             if (wiced_packet_create_tcp(&tcp_client_socket, TCP_PACKET_MAX_DATA_LENGTH, &packet, (uint8_t**)&tx_data, &available_data_length) != WICED_SUCCESS)
             {
                 WPRINT_APP_INFO(("TCP packet creation failed\n"));
                 try_n=try_n+1;
                 if(try_n==TCP_DOWN_NUMBER){
    //                 check_satat_gpio();

                     wiced_framework_reboot();
                 }
                wiced_uart_transmit_bytes(WICED_UART_1,CREATE_FAILED,sizeof(CREATE_FAILED)-1);

//                 return WICED_ERROR;
             }
             else{
                 try_n=0;
             }
//             wiced_tcp_socket_t socket;                      // The TCP socket
//        wiced_tcp_stream_t stream;                      // The TCP stream
//        wiced_result_t result;




      WPRINT_APP_INFO(("try %d\n",try_n));


    // Format the data per the specification in section 6
    wwd_wifi_get_ap_info(&ap_info_buffer, &ap_security);            // Se obtiene la MAC de la red a la que estamos conectados
    wiced_ip_get_ipv4_address( WICED_STA_INTERFACE, &myIpAddress);  // Se obtiene la IP del dispositivo
    wiced_wifi_get_mac_address(&myMac);                             // Se obtiene la MAC del dispositivo




      sprintf(mac_wifi,"%02X:%02X:%02X:%02X:%02X:%02X",myMac.octet[0],myMac.octet[1],myMac.octet[2],myMac.octet[3],myMac.octet[4],myMac.octet[5]);
      sprintf(mac_ap,"%02X:%02X:%02X:%02X:%02X:%02X",ap_info_buffer.BSSID.octet[0], ap_info_buffer.BSSID.octet[1],ap_info_buffer.BSSID.octet[2],ap_info_buffer.BSSID.octet[3],ap_info_buffer.BSSID.octet[4],ap_info_buffer.BSSID.octet[5]);
      sprintf(ip,"%u.%u.%u.%u", (uint8_t)(GET_IPV4_ADDRESS(myIpAddress) >> 24),(uint8_t)(GET_IPV4_ADDRESS(myIpAddress)>> 16),(uint8_t)(GET_IPV4_ADDRESS(myIpAddress) >> 8),(uint8_t)(GET_IPV4_ADDRESS(myIpAddress) >> 0));




             int f=0;
                   if(sent_file_flag==WICED_TRUE){
                       coun=read_data(ACARREO_ROOT,date_get(&i2c_rtc),&fs_handle);

                       /* get the first token */
                       token = strtok(filebuf, s);
                       /* walk through other tokens */
                       if(coun!=0){

                           while( token != NULL ) {
                               //            printf( " >>>>>  %s\n", token );
                                 wiced_rtos_delay_microseconds( 10 );
                                 sprintf(tx_data,"\nHVT:%s\r\n",token);

//                                 if((f>=get_gpio_menssage(LOG_ID_LASET))&&(f<=get_gpio_menssage(LOG_ID_LASET))){

                                     wiced_packet_set_data_end(packet, (uint8_t*)tx_data + strlen(tx_data));
                             /* Send the TCP packet */
                                      if (wiced_tcp_send_packet(&tcp_client_socket, packet) == WICED_SUCCESS)
                                      {
                                         wiced_uart_transmit_bytes(WICED_UART_1,(("%s\r\n",tx_data)),strlen(tx_data));

                                      }


//                                 }
                                token = strtok(NULL, s);
                                coun--;

                                 f++;
                               }
                           f=0;
                       }

                               wiced_rtos_set_semaphore(&tcpGatewaySemaphore);

                             memset(filebuf,NULL,LOCAL_BUFFER_SIZE);

                             if(coun<=2){
//                                 sent_file_flag=WICED_FALSE;
//                                 break;
                             }
                             else{

                             }
                       wiced_rtos_get_semaphore(&tcpGatewaySemaphore,WICED_WAIT_FOREVER);
//                       goto

                   }
                   else if((sent_file_flag==WICED_FALSE)){

//                       for(int y=0;y<=12;y++){
//                           wiced_rtos_delay_microseconds( 10 );
//                       sprintf(tx_data,"\nH;%s,%s,%s,%s,%s\r\n",mac_wifi,mac_ap,ip,time_get(&i2c_rtc),date_get_log(&i2c_rtc));
                       sprintf(tx_data,"\nL;%s,1600,%d%d%d%d0000000000000,%s,%s,%s,%s\r\n",mac_wifi,t1,t2,t3,t4,mac_ap,ip,time_get(&i2c_rtc),date_get(&i2c_rtc));

//                       sprintf(tx_data,"\nL;8C:45:00:D6:39:EC,1600,00000000000000000,22:19:BE:2F:0D:96,10.90.120.41\r\n");

                        wiced_packet_set_data_end(packet, (uint8_t*)tx_data + strlen(tx_data));
                        /* Send the TCP packet */
                                if (wiced_tcp_send_packet(&tcp_client_socket, packet) == WICED_SUCCESS)
                                {
                                    wiced_uart_transmit_bytes(WICED_UART_1,(("%s\r\n",tx_data)),strlen(tx_data));
                                }

//                       }

                   }

                   memset(filebuf,NULL,LOCAL_BUFFER_SIZE);
                  wiced_rtos_delay_milliseconds( 1000 );

                int msm=3;
                do{
                result = wiced_tcp_receive(&tcp_client_socket, &rx_packet, 3500);
                if( result != WICED_SUCCESS )
                {
                /* Delete packet, since the receive failed */
                wiced_packet_delete(rx_packet);

                /* Close the connection */
                wiced_tcp_disconnect(&tcp_client_socket);
//                return WICED_ERROR;
                }
                else{
                 WPRINT_APP_INFO((" packets receive\n"));
                 /* Get the contents of the received packet */
                               wiced_packet_get_data(rx_packet, 0, (uint8_t**)&rx_data, &rx_data_length, &available_data_length);

                               if (rx_data_length != available_data_length)
                               {
                                   WPRINT_APP_INFO(("Fragmented packets not supported\n"));

                                /* Delete packet, since the receive failed */
                               wiced_packet_delete(rx_packet);

                                /* Close the connection */
                               wiced_tcp_disconnect(&tcp_client_socket);
               //                 return WICED_ERROR;
                               }
                }

                /* Null terminate the received string */
                rx_data[rx_data_length] = '\x0';
                WPRINT_APP_INFO(("%s\r\n",rx_data));

                if(strstr(rx_data,"DA")||(strstr(rx_data,"A1"))){
                    printf("\n\n\n\t\t\tRecepcion de evacuacion\n\n\n");
                    sent_file_flag=WICED_TRUE;
                    set_gpio_menssage(LOG_ID_LASET,4);
                }

                if(strstr(rx_data,"HVT:")){

                    limit_log=id_revived(rx_buffer3);

                    if(!(limit_log==coun)){
                        set_gpio_menssage(LOG_ID_LASET,limit_log);
                        sent_file_flag=WICED_TRUE;
                    }
                }
                //
                msm=msm-1;

                wiced_packet_delete(rx_packet);

                }  while(msm!=0);


                memset(rx_data,NULL,sizeof(rx_data));

//                key=2;
                count_tcp++;

            #ifdef GEOLOC
                if(count_tcp==TCP_NUMBER){
                    state=2;
                    count_tcp=0;
                    sent_file_flag=WICED_TRUE;

                }
                else if(count_tcp==TCP_NUMBER/2){
                    sent_file_flag=WICED_FALSE;
                    state=1;
                }
                else{
                    state=1;
                }
            #else
                if(count_tcp==TCP_NUMBER){
                    state=2;
                    count_tcp=0;
                    sent_file_flag=WICED_TRUE;

                }
                else if(count_tcp==TCP_NUMBER/2){
                    sent_file_flag=WICED_FALSE;
                    state=1;
                }
                else{
                    state=1;
                }
            #endif

//             wiced_rtos_delay_milliseconds( 1000 );
             wiced_packet_delete(packet);
             wiced_packet_delete(rx_packet);
             wiced_tcp_disconnect(&tcp_client_socket);

//             wiced_rtos_unlock_mutex(&pubSubMutex);

//             wiced_rtos_set_semaphore(&tcpReceptionSemaphore);

             return state;

}

int tcp_client_geo( )
{

    uint8_t state=0;

//    wiced_rtos_lock_mutex(&pubSubMutex);

    send_data_task=WICED_TRUE;
    send_data_task=WICED_TRUE;

    WPRINT_APP_INFO(("Event Thread Tcp client\n"));
//        counter_tcp_guardian=0;
        char* resultado;
        wiced_result_t result;
        wiced_mac_t myMac;
        wiced_ip_address_t myIpAddress;
        wl_bss_info_t ap_info_buffer;
        wiced_security_t ap_security;

        wiced_packet_t*          packet;
        wiced_packet_t*          rx_packet;
        unsigned char*                    tx_data;
        unsigned char*                    rx_data;
        uint16_t                 rx_data_length;
        uint16_t                 available_data_length;
        int                      connection_retries;

             wiced_ip_address_t INITIALISER_IPV4_ADDRESS( server_ip_address, s1 );

             /* Connect to the remote TCP server, try several times */
             connection_retries = 0;
             do
             {
                 result = wiced_tcp_connect( &tcp_client_socket, &server_ip_address, TCP_SERVER_PORT, TCP_CLIENT_CONNECT_TIMEOUT );
                 connection_retries=connection_retries+1;
             }
             while( ( result != WICED_SUCCESS ) && ( connection_retries < TCP_CONNECTION_NUMBER_OF_RETRIES ) );
             if ( result != WICED_SUCCESS )
                {
                WPRINT_APP_INFO(("Unable to connect to the server! Halt.\n"));
                try_n=try_n+1;
                if(try_n==TCP_DOWN_NUMBER){
    //                check_satat_gpio();

                        wiced_framework_reboot();
                }
                wiced_uart_transmit_bytes(WICED_UART_1,TCP_CONNECT_FAILED,sizeof(TCP_CONNECT_FAILED)-1);
//                return WICED_ERROR;

             }
             else{
                 try_n=0;
             }
             /* Create the TCP packet. Memory for the tx_data is automatically allocated */
             if (wiced_packet_create_tcp(&tcp_client_socket, TCP_PACKET_MAX_DATA_LENGTH, &packet, (uint8_t**)&tx_data, &available_data_length) != WICED_SUCCESS)
             {
                 WPRINT_APP_INFO(("TCP packet creation failed\n"));
                 try_n=try_n+1;
                 if(try_n==TCP_DOWN_NUMBER){
    //                 check_satat_gpio();

                     wiced_framework_reboot();
                 }
                wiced_uart_transmit_bytes(WICED_UART_1,CREATE_FAILED,sizeof(CREATE_FAILED)-1);

//                 return WICED_ERROR;
             }
             else{
                 try_n=0;
             }
//             wiced_tcp_socket_t socket;                      // The TCP socket
//        wiced_tcp_stream_t stream;                      // The TCP stream
//        wiced_result_t result;




      WPRINT_APP_INFO(("try %d\n",try_n));


    // Format the data per the specification in section 6
    wwd_wifi_get_ap_info(&ap_info_buffer, &ap_security);            // Se obtiene la MAC de la red a la que estamos conectados
    wiced_ip_get_ipv4_address( WICED_STA_INTERFACE, &myIpAddress);  // Se obtiene la IP del dispositivo
    wiced_wifi_get_mac_address(&myMac);                             // Se obtiene la MAC del dispositivo




      sprintf(mac_wifi,"%02X:%02X:%02X:%02X:%02X:%02X",myMac.octet[0],myMac.octet[1],myMac.octet[2],myMac.octet[3],myMac.octet[4],myMac.octet[5]);
      sprintf(mac_ap,"%02X:%02X:%02X:%02X:%02X:%02X",ap_info_buffer.BSSID.octet[0], ap_info_buffer.BSSID.octet[1],ap_info_buffer.BSSID.octet[2],ap_info_buffer.BSSID.octet[3],ap_info_buffer.BSSID.octet[4],ap_info_buffer.BSSID.octet[5]);
      sprintf(ip,"%u.%u.%u.%u", (uint8_t)(GET_IPV4_ADDRESS(myIpAddress) >> 24),(uint8_t)(GET_IPV4_ADDRESS(myIpAddress)>> 16),(uint8_t)(GET_IPV4_ADDRESS(myIpAddress) >> 8),(uint8_t)(GET_IPV4_ADDRESS(myIpAddress) >> 0));




             coun=read_data(SF_ROOT,date_get(&i2c_rtc),&fs_handle);
             int f=0;
                   if(sent_file_flag==WICED_TRUE){

                       /* get the first token */
                       token = strtok(filebuf, s);
                       /* walk through other tokens */
                       if(coun!=0){

                           while( token != NULL ) {
                               //            printf( " >>>>>  %s\n", token );
                                 wiced_rtos_delay_microseconds( 10 );
                                 sprintf(tx_data,"\nVH:%s\r\n",token);

                                 if((f>=get_gpio_menssage(LOG_ID_LASET))&&(f<=get_gpio_menssage(LOG_ID_LASET))){

                                     wiced_packet_set_data_end(packet, (uint8_t*)tx_data + strlen(tx_data));
                             /* Send the TCP packet */
                                      if (wiced_tcp_send_packet(&tcp_client_socket, packet) == WICED_SUCCESS)
                                      {
                                         wiced_uart_transmit_bytes(WICED_UART_1,(("%s\r\n",tx_data)),strlen(tx_data));

                                      }


                                 }
                                token = strtok(NULL, s);
                                coun--;

                                 f++;
                               }
                           f=0;
                       }

                               wiced_rtos_set_semaphore(&tcpGatewaySemaphore);

                             memset(filebuf,NULL,LOCAL_BUFFER_SIZE);

                             if(coun<=2){
//                                 sent_file_flag=WICED_FALSE;
//                                 break;
                             }
                             else{

                             }
                       wiced_rtos_get_semaphore(&tcpGatewaySemaphore,WICED_WAIT_FOREVER);
//                       goto

                   }
                   else if((sent_file_flag==WICED_FALSE)){

//                       for(int y=0;y<=12;y++){
//                           wiced_rtos_delay_microseconds( 10 );
                       sprintf(tx_data,"\nH;%s,%s,%s,%s,%s\r\n",mac_wifi,mac_ap,ip,time_get(&i2c_rtc),date_get_log(&i2c_rtc));
//                       sprintf(tx_data,"\nL;8C:45:00:D6:39:EC,1600,00000000000000000,22:19:BE:2F:0D:96,10.90.120.41\r\n");

                        wiced_packet_set_data_end(packet, (uint8_t*)tx_data + strlen(tx_data));
                        /* Send the TCP packet */
                                if (wiced_tcp_send_packet(&tcp_client_socket, packet) == WICED_SUCCESS)
                                {
                                    wiced_uart_transmit_bytes(WICED_UART_1,(("%s\r\n",tx_data)),strlen(tx_data));
                                }

//                       }

                   }

                   memset(filebuf,NULL,LOCAL_BUFFER_SIZE);

                int msm=3;
                do{
                result = wiced_tcp_receive(&tcp_client_socket, &rx_packet, 2000);
                if( result != WICED_SUCCESS )
                {
                /* Delete packet, since the receive failed */
                wiced_packet_delete(rx_packet);

                /* Close the connection */
                wiced_tcp_disconnect(&tcp_client_socket);
//                return WICED_ERROR;
                }
                else{
                 WPRINT_APP_INFO((" packets receive\n"));

                }
                /* Get the contents of the received packet */
                wiced_packet_get_data(rx_packet, 0, (uint8_t**)&rx_data, &rx_data_length, &available_data_length);

                if (rx_data_length != available_data_length)
                {
                    WPRINT_APP_INFO(("Fragmented packets not supported\n"));

                 /* Delete packet, since the receive failed */
                wiced_packet_delete(rx_packet);

                 /* Close the connection */
                wiced_tcp_disconnect(&tcp_client_socket);
//                 return WICED_ERROR;
                }
                /* Null terminate the received string */
                rx_data[rx_data_length] = '\x0';
                WPRINT_APP_INFO(("%s\r\n",rx_data));

//                if(strstr(rx_data,"DA")||(strstr(rx_data,"A1"))){
//                    printf("\n\n\n\t\t\tRecepcion de evacuacion\n\n\n");
//                    sent_file_flag=WICED_TRUE;
//                    set_gpio_menssage(LOG_ID_LASET,4);
//                }

                if(strstr(rx_data,"HVT:")){

                    limit_log=id_revived(rx_buffer3);

                    if(!(limit_log==coun)){
                        set_gpio_menssage(LOG_ID_LASET,limit_log);
                        sent_file_flag=WICED_TRUE;
                    }
                }
                //
                msm=msm-1;

                wiced_packet_delete(rx_packet);

                }  while(msm!=0);



//                key=2;
                count_tcp++;

                if(count_tcp==TCP_NUMBER){
                    state=3;
                    count_tcp=0;
                    sent_file_flag=WICED_TRUE;

                }
                else if(count_tcp==TCP_NUMBER/2){
                    sent_file_flag=WICED_FALSE;
                    state=2;
                }
                else{
                    state=2;
                }

             wiced_rtos_delay_milliseconds( 1000 );

             wiced_packet_delete(packet);
             wiced_packet_delete(rx_packet);
             wiced_tcp_disconnect(&tcp_client_socket);

//             wiced_rtos_unlock_mutex(&pubSubMutex);

//             wiced_rtos_set_semaphore(&tcpReceptionSemaphore);

             return state;

}



#endif  /* stdbool.h */