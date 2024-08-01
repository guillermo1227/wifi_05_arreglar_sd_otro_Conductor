#include <math.h>
#include <malloc.h>
#include "wiced.h"

#include "IMU/i2c_lsm6dsm.h"
#include  "UART/manager_menssage.h"
//#include "GPIO/manager_gpio.h"

#define RTC_DEVICE_ADDR        (0x68)
#define RTC_WOAMI_REG         (0x00)//(0x02 | 0x80)
#define SEGREG_X8                (0x00)
#define MINREG_X8                (0x01)
#define HREG_X8                  (0x02)
#define YEARREG_X8                (0x06)
#define MONTHREG_X8               (0x05)
#define DAYREG_X8                 (0x04)





#define NUM_I2C_MESSAGE_RETRIES   (3)
    int ConvertExa(int numberDec);
    int powInt(int x, int y);
    int parseInt(char* chars);






void init_rtc(wiced_i2c_device_t * i2c_device_rtc){
    i2c_device_rtc->address=RTC_DEVICE_ADDR;
    i2c_device_rtc->address_width=I2C_ADDRESS_WIDTH_7BIT;
    i2c_device_rtc->port=WICED_I2C_1;
    i2c_device_rtc->speed_mode=I2C_HIGH_SPEED_MODE;

    /* Initialize I2C */
       if ( wiced_i2c_init( i2c_device_rtc ) != WICED_SUCCESS )
       {
           WPRINT_APP_INFO( ( "I2C Initialization Failed\n" ) );
           wiced_uart_transmit_bytes(WICED_UART_1,"I2C Initialization Failed\n",strlen("I2C Initialization Failed\n"));
       }
       else
       {
           wiced_uart_transmit_bytes(WICED_UART_1,"I2C succes RTC\n",strlen("I2C succes RTC\n"));
       }

       /* Probe I2C bus for temperature sensor */
       if( wiced_i2c_probe_device( i2c_device_rtc, NUM_I2C_MESSAGE_RETRIES ) != WICED_TRUE )
       {
           wiced_uart_transmit_bytes(WICED_UART_1,"I2C probe Failed\n",strlen("I2C probe Failed\n"));
       }
       else
       {
           wiced_uart_transmit_bytes(WICED_UART_1,"I2C probe Succes\n",strlen("I2C probe Succes\n"));
       }

}


int time_set(char *input , wiced_i2c_device_t * i2c_device_rtc)
{
    int chourtwodigits;
    int cmintwodigits;
    int csegtwodigits;
//    uint8_t *wbuf;
//    uint8_t *rbuf;

    uint8_t wbuf[4];
    uint8_t rbuf[4];
    //printf("size %zu \r\n", strlen(input));

    if(strlen(input) == 8){
        printf("Numero aceptado: %s \r\n", input);
        char * token = strtok(input, ":");
        int hourtwodigits = parseInt(token);
//        printf("First %d \r\n", hourtwodigits);
        chourtwodigits=ConvertExa(hourtwodigits);

        token = strtok(NULL, ":");
        int mintwodigits = parseInt(token);
//        printf("Second %d \r\n", mintwodigits);
        cmintwodigits = ConvertExa(mintwodigits);

        token = strtok(NULL, ":");
        int sectwodigits = parseInt(token);
//        printf("Third %d \r\n", sectwodigits);
        csegtwodigits = ConvertExa(sectwodigits);


        /* Power-up the device */
        wbuf[0] = HREG_X8;
        wbuf[1] = (uint8_t)chourtwodigits;
        wiced_i2c_write( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 2);
//        WPRINT_APP_INFO( ( "Writing HOURS: %x\n", wbuf[1]) );

        wbuf[0] = MINREG_X8;
        wbuf[1] =(uint8_t) cmintwodigits;
        wiced_i2c_write( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 2);
//        WPRINT_APP_INFO( ( "Writing MINUTES: %x\n", wbuf[1]) );

        wbuf[0] = SEGREG_X8;
        wbuf[1] = (uint8_t)csegtwodigits;
        wiced_i2c_write( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 2);
//        WPRINT_APP_INFO( ( "Writing SECONDS: %x\n", wbuf[1]) );

    }else{
        printf("ERROR LA CADENA NO SE INGRESO CORRECTAMENTE\r\n--------------\r\n--------------\r\n--------------\r\n--------------\r\n");
    }

}

int date_set(char *input,wiced_i2c_device_t *i2c_device_rtc)
{
    int cdaytwodigits;
    int cmonthtwodigits;
    int cbyearfourdigits;

    uint8_t wbuf[4];
    uint8_t rbuf[4];
    //printf("size %zu \r\n", strlen(input));
    if(strlen(input) == 8){
        printf("Fecha aceptada: %s \r\n", input);  //day declaration
        char * token = strtok(input, "/");
        int daytwodigits = parseInt(token);
        cdaytwodigits=ConvertExa(daytwodigits);


        token = strtok(NULL, "/");
        int monthtwodigits = parseInt(token);
        cmonthtwodigits = ConvertExa(monthtwodigits);

        token = strtok(NULL, "/");
        int yearfourdigits = parseInt(token);
        cbyearfourdigits = ConvertExa(yearfourdigits);

        /* Power-up the device */
            wbuf[0] = DAYREG_X8;
            wbuf[1] = cdaytwodigits;
            wiced_i2c_write( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 2);
//            WPRINT_APP_INFO( ( "Writing DAY: %x\n", wbuf[1]) );

            wbuf[0] = MONTHREG_X8;
            wbuf[1] = cmonthtwodigits;
            wiced_i2c_write( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 2);
//            WPRINT_APP_INFO( ( "Writing MONTHS: %x\n", wbuf[1]) );

            wbuf[0] = YEARREG_X8;
            wbuf[1] = cbyearfourdigits;
            wiced_i2c_write( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 2);
//            WPRINT_APP_INFO( ( "Writing YEAR: %x\n", wbuf[1]) );

    }else{
        printf("ERROR LA CADENA NO SE INGRESO CORRECTAMENTE\r\n--------------\r\n--------------\r\n--------------\r\n--------------\r\n");
    }



}


char * time_get(wiced_i2c_device_t * i2c_device_rtc){
    uint8_t *wbuf;
    uint8_t *rbuf;
    uint8_t clk[3];
    char *_time_txt;

    wbuf = malloc(sizeof(uint8_t)*4);
    rbuf = malloc(sizeof(uint8_t)*4);

    memset(wbuf, 0, (sizeof(uint8_t)*4));
    memset(rbuf, 0, (sizeof(uint8_t)*4));

    // Leemos aqui los segundos
    //    //wbuf[0] = HTS221_T0_DEGC_X8;
    wbuf[0] = SEGREG_X8;
    wiced_i2c_write( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    clk[0] = rbuf[0];

    wbuf[0] = MINREG_X8;
    wiced_i2c_write( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    clk[1] = rbuf[0];

    wbuf[0] = HREG_X8;
    wiced_i2c_write( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    clk[2] = rbuf[0];

    free( rbuf );
    free( wbuf );

    _time_txt = malloc(sizeof(char)*11);

    sprintf(_time_txt, "%02x:%02x:%02x", clk[2],clk[1], clk[0]);
//    WPRINT_APP_INFO( ( "TIEMPO REAL======>  %s\n" , _time_txt) );
    return _time_txt;
}



char * date_get(wiced_i2c_device_t * i2c_device_rtc){
    uint8_t *wbuf;
    uint8_t *rbuf;
    uint8_t clk[3];
    char *_date_txt;

    wbuf = malloc(sizeof(uint8_t)*4);
    rbuf = malloc(sizeof(uint8_t)*4);

    memset(wbuf, 0, (sizeof(uint8_t)*4));
    memset(rbuf, 0, (sizeof(uint8_t)*4));


    wbuf[0] = DAYREG_X8;
    wiced_i2c_write( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    clk[0] = rbuf[0];

    wbuf[0] = MONTHREG_X8;
    wiced_i2c_write( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    clk[1] = rbuf[0];

    wbuf[0] = YEARREG_X8;
    wiced_i2c_write( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    clk[2] = rbuf[0];


    free( rbuf );
    free( wbuf );


    _date_txt = malloc(sizeof(char)*11);

    sprintf(_date_txt, "%02x_%02x_20%02x", clk[0], clk[1], clk[2]);
    // WPRINT_APP_INFO( ( "FECHA======>  %s\n" , _date_txt) );

    return _date_txt;
}


char * date_get_log(wiced_i2c_device_t * i2c_device_rtc){
    uint8_t *wbuf;
    uint8_t *rbuf;
    uint8_t clk[3];
    char *_date_txt;

    wbuf = malloc(sizeof(uint8_t)*4);
    rbuf = malloc(sizeof(uint8_t)*4);

    memset(wbuf, 0, (sizeof(uint8_t)*4));
    memset(rbuf, 0, (sizeof(uint8_t)*4));


    wbuf[0] = DAYREG_X8;
    wiced_i2c_write( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    clk[0] = rbuf[0];

    wbuf[0] = MONTHREG_X8;
    wiced_i2c_write( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    clk[1] = rbuf[0];

    wbuf[0] = YEARREG_X8;
    wiced_i2c_write( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, wbuf, 1 );
    wiced_i2c_read( i2c_device_rtc, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, rbuf, 1 );
    clk[2] = rbuf[0];


    free( rbuf );
    free( wbuf );


    _date_txt = malloc(sizeof(char)*11);

    sprintf(_date_txt, "%02x/%02x/20%02x", clk[0], clk[1], clk[2]);
    // WPRINT_APP_INFO( ( "FECHA======>  %s\n" , _date_txt) );

    return _date_txt;
}




int powInt(int x, int y)
    {
        for (int i = 0; i < y; i++)
        {
            x *= 10;
        }
        return x;
    }

int parseInt(char* chars)
    {
        int sum = 0;
        int len = strlen(chars);
        for (int x = 0; x < len; x++)
        {
            int n = chars[len - (x + 1)] - '0';
            sum = sum + powInt(n, x);
        }
        return sum;
    }
int ConvertExa(int numberDec)
    {
        int quotient, module;
        long int temp;
        quotient = numberDec / 10;
        module = numberDec % 10;
        //printf("quoting %d: ", quotient);
        //printf("quoting %d: ", module);
        if( quotient == 1){
            temp =numberDec + 6;

        }else if( quotient == 2){
            temp =numberDec + 12;

        }else if( quotient == 3){
            temp =numberDec + 18;

        }else if( quotient == 4){
            temp =numberDec + 24;

        }else if( quotient == 5){
            temp =numberDec + 30;

        }else{
            temp = numberDec;
        }

        return temp;
    }
