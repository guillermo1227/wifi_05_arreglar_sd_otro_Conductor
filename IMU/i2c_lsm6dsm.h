#ifndef  _IMU_MANAGER_LSM6D_H_
#define  _IMU_MANAGER_LSM6D_H_

#include "wiced.h"
#include <math.h>
#include <stdbool.h>
#include  "UART/manager_menssage.h"
#include "GPIO/manager_gpio_vh.h"
//#include "wiced_platform.h"

#define NUM_I2C_MESSAGE_RETRIES   (3)

#define MAX11615_SLAVE_ADDR         (0x36)
#define MAX11615_SETUP_BYTE         (0b00110110)
#define NUM_I2C_MESSAGE_RETRIES     (3)

//************************************** DEFINE MINI-MAX VALUES FOR HC PROTOCOL ******************************************
#define ACC_MAX      1      // Acceleration tolerance for Hombre Inactivo and Hombre Caido
#define ACC_MIN     -1
#define ACCV_MAX     4      // Tolerance acceleration for Tag Vehiculo
#define ACCV_MIN    -4
#define ACC_G_MAX    10
#define ACC_G_MIN   -10
#define G_MAX        40
#define G_MIN       -40
#define TVG_MAX      3
#define TVG_MIN     -3
#define WIX_MAX     105
#define WIX_MIN     80
#define WIY_MAX     10
#define WIY_MIN     -10

//*************************************************** AVERAGE FILTER   **************************************************
#define n_readings 20      // Number of int

uint8_t index_x = 0;
int readings_x[n_readings];
int total_x = 0;
int average_x = 0;
uint8_t index_y = 0;
int readings_y[n_readings];
int total_y = 0;
int average_y = 0;



wiced_bool_t faslh_imu=WICED_FALSE;
int imu_alert_count;

//*************************************************** DEFINE MASSAGE'S ***************************************************

char hi_counter_print[5] = {};
char hc_counter_print[5] = {};
char tv_counter_print[5] = {};

//*************************************************** DEFINE REGISTER'S ************************************************
#define i2c_address (0x6A)      // This register is gyroscope address
#define WHO_AM_I    (0x0F)      // This register is for testing communication with LSM6DSM

#define CTRL1_XL                0x10        // Address control register 1 (accelerometer), this register must be 0x70
#define CTRL1_ODR_XL_100       (0x04<<4)    //100Hz
#define CTRL1_ODR_XL_200       (0x05<<4)    //200Hz
#define CTRL1_ODR_XL_400       (0x06<<4)    //400Hz
#define CTRL1_ODR_XL_800       (0x07<<4)    //833Hz (High performance)
#define CTRL1_FS_XL_2G         (0x00<<2)    //Accelerometer full-scale (00: ±2 g; 01: ±16 g; 10: ±4 g; 11: ±8 g)
#define CTRL1_FS_XL_16G        (0x01<<2)
#define CTRL1_FS_XL_4G         (0x10<<2)
#define CTRL1_FS_XL_8G         (0x11<<2)
#define CTRL1_LPF1_BW_SEL_XL   (0x00<<1)    //Manufacture recommend this bit pull down
#define CTRL1_BW0_XL           (0x00)       //Only select and modify this register if ODR_XL is faster that 1.67kHz

#define CTRL2_G                 0x11        // Address control register 2 (gyroscope), this register must be 0x70
#define CTRL2_ODR_G_100       (0x04<<4)     //100Hz
#define CTRL2_ODR_G_200       (0x05<<4)     //200Hz
#define CTRL2_ODR_G_400       (0x06<<4)     //400Hz
#define CTRL2_ODR_G_800       (0x07<<4)     //833Hz (High performance)
#define CTRL2_FS_G_250        (0x00<<2)     //Gyroscope full-scale (00: 250 dps; 01: 500 dps; 10: 1000 dps; 11: 2000 dps)
#define CTRL2_FS_G_500        (0x01<<2)     //500dps
#define CTRL2_FS_G_1000       (0x10<<2)     //1000dps
#define CTRL2_FS_G_2000       (0x11<<2)     //2000dps
#define CTRL2_FS_125          (0x00<<1)     //Gyroscope full-scale at 125

#define STATUS_REG              0x1E        // Address control STATUS REG, this register  allow get new reading

// READING REGISTER'S GYROSCOPE             // These address are Gyroscope and Accelerometer output
#define OUTX_L_G    (0x22)
#define OUTX_H_G    (0x23)
#define OUTY_L_G    (0x24)
#define OUTY_H_G    (0x25)
#define OUTZ_L_G    (0x26)
#define OUTZ_H_G    (0x27)
// READING REGISTER'S ACCELEROMETER
#define OUTX_L_XL   (0x28)
#define OUTX_H_XL   (0x29)
#define OUTY_L_XL   (0x2A)
#define OUTY_H_XL   (0x2B)
#define OUTZ_L_XL   (0x2C)
#define OUTZ_H_XL   (0x2D)

//************************************************* INITIALIZE GLOBAL VARIABLE'S *********************************************
//  DEFINE FOR GETTING ACC AND GYRO VALUE
#define ACELERACION_X   0x00
#define ACELERACION_Y   0x01
#define ACELERACION_Z   0x10
#define GYRO_X          0x11
#define GYRO_Y          0x100

//  INITIALIZE BUTTON USER 1 FOR START AND STOP LOOP CODE
volatile wiced_bool_t buttonPress = WICED_FALSE;

//  WRITING AND READING BUFFER
uint8_t tx_buffer[2];
uint8_t rx_buffer1[1];

//  GET AND READ IMU SENSOR VARIABLES
int gget_x[2], gget_z[2];   // gget_x[0] is actual gyro value ; gget_x[1] is previous gyro value
struct{
    float ax;
    float ay;
    float az;

    int16_t gx_actual;
    int16_t gy_actual;
    int16_t gx_previo;
    int16_t gy_previo;

    int16_t gx_diff;
    int16_t gy_diff;

    int16_t gx;
    int16_t gy;

    int8_t ax_actual;
    int8_t ax_previous;
    int8_t ay_actual;
    int8_t ay_previous;
    int8_t az_actual;
    int8_t az_previous;

    int8_t ax_diff;
    int8_t ay_diff;
    int8_t az_diff;
} imu;

//  SETUP I2C MASTER
const wiced_i2c_device_t i2c_device_lsm6dsm = {
        .port = WICED_I2C_2,
        .address = i2c_address,
        .address_width = I2C_ADDRESS_WIDTH_7BIT,
        .speed_mode = I2C_STANDARD_SPEED_MODE
};

//**************************************************** DEFINE FALLED MAN ********************************************
static wiced_timer_t hc_timer;          // Timer hc is for hombre caido
static wiced_timer_t wi_timer;          // Timer wi is for wifi inactivo
uint8_t hi_counter;
uint8_t tv_counter;
uint8_t wi_counter;

wiced_bool_t hi_flag = WICED_FALSE;
wiced_bool_t hc_flag = WICED_FALSE;
wiced_bool_t tv_flag = WICED_FALSE;
wiced_bool_t wi_flag = WICED_FALSE;
wiced_bool_t BLE_flag = WICED_FALSE;    // THIS FLAG WILL BE ACTIVE BY BLE
wiced_bool_t flag_hc_is_active=WICED_FALSE;


//********************************************** DEFINE FUNCTION ****************************************************
void imu_init();
wiced_result_t imu_read();
void hi_protocol();
void hc_protocol();
void tv_protocol();
void wi_protocol();
void button_isr();

//*******************************************************************************************************************
//***************************************************** START THE MACHAKA *******************************************
//*******************************************************************************************************************
void button_isr(void* arg){
    buttonPress ^= WICED_TRUE;
    wiced_gpio_output_high( WICED_LED1 );
}

void imu_init(){

    wiced_uart_transmit_bytes(WICED_UART_1,"Iniciar I2C\n", strlen("Iniciar I2C\n"));

    tx_buffer[0] = WHO_AM_I;
    //  INITIALIZATION I2C
    wiced_i2c_init( &i2c_device_lsm6dsm );

    //  PROBE INICILIZED I2C
    if (wiced_i2c_init( &i2c_device_lsm6dsm ) != WICED_SUCCESS ){
        wiced_uart_transmit_bytes(WICED_UART_1, COM_ERROR, sizeof(COM_ERROR)-1);
    } else{ wiced_uart_transmit_bytes(WICED_UART_1, COM_EXITO, sizeof(COM_EXITO)-1); }

    //  PROBE CONNECTION I2C ADDRESS IMU SENSOR
    if(wiced_i2c_probe_device( &i2c_device_lsm6dsm, NUM_I2C_MESSAGE_RETRIES ) != WICED_TRUE ){
        wiced_uart_transmit_bytes(WICED_UART_1, PROBE_ERROR, sizeof(PROBE_ERROR)-1);
    } else{ wiced_uart_transmit_bytes(WICED_UART_1, PROBE_EXITO, sizeof(PROBE_EXITO)-1);}

    //  PROBE WRITING AND READING REGISTER WITH WHO AM I
    wiced_i2c_write(&i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &tx_buffer[0], 1 );
    wiced_i2c_read(&i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &rx_buffer1, sizeof(rx_buffer1) );

    if( rx_buffer1[0] != 0x6A ){
        wiced_uart_transmit_bytes(WICED_UART_1, WAI_ERROR, sizeof(WAI_ERROR)-1);
    } else{ wiced_uart_transmit_bytes(WICED_UART_1, WAI_EXITO, sizeof(WAI_EXITO)-1); }

    //  POWER-UP THE DEVICE / ENABLE ALL SENSOR'S
    //  POWER-UP ACCELEROMETER                  Configure Linear acceleration sensor control register 1 (r/w)
    tx_buffer[0]= CTRL1_XL;
    tx_buffer[1]= ( CTRL1_ODR_XL_100 | CTRL1_FS_XL_2G | CTRL1_LPF1_BW_SEL_XL | CTRL1_BW0_XL );
    wiced_i2c_write( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &tx_buffer, 2);
    wiced_i2c_write( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &tx_buffer[0], 1);
    wiced_i2c_read( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &rx_buffer1, sizeof(rx_buffer1));

    //  POWER-UP GYROSCOPE                      Configure Angular rate sensor control register 2 (r/w).
    tx_buffer[0]= CTRL2_G;
    tx_buffer[1]= ( CTRL2_ODR_G_100 | CTRL2_FS_G_250 | CTRL2_FS_125 );
    wiced_i2c_write( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &tx_buffer, 2 );
    wiced_i2c_write( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &tx_buffer[0], 1 );
    wiced_i2c_read( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &rx_buffer1, sizeof(rx_buffer1) );

    //  READ STATUS_REG (SHOULD BE 0X07) TO CHECK CORRECT CONFIGURATION
    tx_buffer[0]=STATUS_REG;
    wiced_i2c_write( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &tx_buffer[0], 1 );
    wiced_i2c_read( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &rx_buffer1, sizeof(rx_buffer1) );
    if(rx_buffer1[0] == 0x07){
        wiced_uart_transmit_bytes(WICED_UART_1, STATUS_EXITO, sizeof(STATUS_EXITO)-1);
    }
    else{
    wiced_uart_transmit_bytes(WICED_UART_1,"IMU SENSOR IT'S NOT READY FOR GET VALUES\r\n", strlen("IMU SENSOR IT'S NOT READY FOR GET VALUES\r\n"));
    }

    for(int thisReading_x = 0; thisReading_x < n_readings; thisReading_x++){
        readings_x[thisReading_x] = 0;
    }
    for(int thisReading_y = 0; thisReading_y < n_readings; thisReading_y++){
        readings_y[thisReading_y] = 0;
    }
}

wiced_result_t imu_read(){
    tx_buffer[0] = STATUS_REG;      // CHECK CORRECT CONNECTION FOR DISCARD ERROR
    uint8_t i;
    uint8_t acc[8],g[8];                                // Access to register variables
    int16_t axdata, aydata, azdata, gxdata, gydata, gzdata;     // Get value without processing
    float get_gxdata, get_gydata, get_gzdata;           // Get data gyroscope with processing
    int16_t accgx, accgy;                               // Acceleration data for gyroscope

    // READ STATUS_REGISTER (SHOULD BE 0x07 = 0x00000111);
    wiced_i2c_write( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &tx_buffer[0], 1);
    wiced_i2c_read( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &rx_buffer1, sizeof(rx_buffer1));
    //WPRINT_APP_INFO(("STATUS: 0x%x IS READY\t\t", rx_buffer[0]));

    char text2[20];
    sprintf(text2,"");
    wiced_uart_transmit_bytes(WICED_UART_1,"Ver val\n", strlen("Ver val\n"));
    if(rx_buffer1[0]==0x07){
        //  -   -   -   -   -   -   -   -   GET VALUE FROM GYIROSCOPE    -   -   -   -   -   -   -   -   -   -
        // READ REGISTER OUTX_H | OUTX_L GYROSCOPE
        tx_buffer[0] = OUTX_H_G;
        wiced_i2c_write( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &tx_buffer[0], 1);
        wiced_i2c_read( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &rx_buffer1, sizeof(rx_buffer1));
        g[1] = rx_buffer1[0];

        tx_buffer[0] = OUTX_L_G;
        wiced_i2c_write( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &tx_buffer[0], 1);
        wiced_i2c_read( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &rx_buffer1, sizeof(rx_buffer1));
        g[0] = rx_buffer1[0];

        // READ REGISTER OUTY_H | OUTY_L GYROSCOPE
        tx_buffer[0] = OUTY_H_G;
        wiced_i2c_write( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &tx_buffer[0], 1);
        wiced_i2c_read( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &rx_buffer1, sizeof(rx_buffer1));
        g[3] = rx_buffer1[0];

        tx_buffer[0] = OUTY_L_G;
        wiced_i2c_write( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &tx_buffer[0], 1);
        wiced_i2c_read( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &rx_buffer1, sizeof(rx_buffer1));
        g[2] = rx_buffer1[0];

        // READ REGISTER OUTZ_H | OUTZ_L GYROSCOPE
        tx_buffer[0] = OUTZ_H_G;
        wiced_i2c_write( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &tx_buffer[0], 1);
        wiced_i2c_read( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &rx_buffer1, sizeof(rx_buffer1));
        g[5] = rx_buffer1[0];

        tx_buffer[0] = OUTZ_L_G;
        wiced_i2c_write( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &tx_buffer[0], 1);
        wiced_i2c_read( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &rx_buffer1, sizeof(rx_buffer1));
        g[4] = rx_buffer1[0];

        //  -   -   -   -   -   -   -   -   GET VALUE FROM ACCELEROMETER    -   -   -   -   -   -   -   -   -
        // READ REGISTER OUTX_H | OUTX_L ACCELEROMETER
        tx_buffer[0] = OUTX_H_XL;
        wiced_i2c_write( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &tx_buffer[0], 1);
        wiced_i2c_read( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &rx_buffer1, sizeof(rx_buffer1));
        acc[1] = rx_buffer1[0];

        tx_buffer[0] = OUTX_L_XL;
        wiced_i2c_write( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &tx_buffer[0], 1);
        wiced_i2c_read( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &rx_buffer1, sizeof(rx_buffer1));
        acc[0] = rx_buffer1[0];

        // READ REGISTER OUTY_H | OUTY_L ACCELEROMETER
        tx_buffer[0] = OUTY_H_XL;
        wiced_i2c_write( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &tx_buffer[0], 1);
        wiced_i2c_read( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &rx_buffer1, sizeof(rx_buffer1));
        acc[3] = rx_buffer1[0];

        tx_buffer[0] = OUTY_L_XL;
        wiced_i2c_write( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &tx_buffer[0], 1);
        wiced_i2c_read( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &rx_buffer1, sizeof(rx_buffer1));
        acc[2] = rx_buffer1[0];

        // READ REGISTER OUTZ_H | OUTZ_L ACCELEROMETER
        tx_buffer[0] = OUTZ_H_XL;
        wiced_i2c_write( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &tx_buffer[0], 1);
        wiced_i2c_read( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &rx_buffer1, sizeof(rx_buffer1));
        acc[5] = rx_buffer1[0];

        tx_buffer[0] = OUTZ_L_XL;
        wiced_i2c_write( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &tx_buffer[0], 1);
        wiced_i2c_read( &i2c_device_lsm6dsm, WICED_I2C_START_FLAG | WICED_I2C_STOP_FLAG, &rx_buffer1, sizeof(rx_buffer1));
        acc[4] = rx_buffer1[0];

        //  -   -   -   -   -   -   -   -   -   -   -   -   CONVERT DATA    -   -   -   -   -   -   -   -   -   -   -   -
        //  GET ACCELEROMETER AND GYROSCOPE DATA FROM REGISTER
        axdata =(acc[1]<<8 | acc[0]);
        aydata =(acc[3]<<8 | acc[2]);
        azdata =(acc[5]<<8 | acc[4]);
        gxdata =(g[1]<<8 | g[0]);
        gydata =(g[3]<<8 | g[2]);
        gzdata =(g[5]<<8 | g[4]);

        //  CONVERT DATA FROM REGISTER TO PHYSICAL VALUE (m/s and g/s)
        imu.ax = axdata*(9.81/16384.0);                     // transform data to m/s 2*(g=9.81)/(2^16/2)
        imu.ay = aydata*(9.81/16384.0);                     // for 4g scale 4*9.81/(2^16/2)
        imu.az = azdata*(9.81/16384.0);                     // for 8g scale 8*9.81/(2^16/2)
        // Modify X gyro signal to match geometrically
        // Calibrate gyro ~= 0
        get_gxdata = -(gxdata*(250.0/32768.0));                 // transform data to g/s, for 250 scale 250/(2*1024*16)
        for(i=0; i<= 3; i++){
            if(get_gxdata <= 0){
                get_gxdata++;
            }
            if(get_gxdata >= 0){
                get_gxdata--;}
        }
        get_gydata = -(gydata*(250.0/32768.0));                // Modify Y gyro value to match geometrically
        for(i=0; i<= 3; i++){
            if(get_gydata <= 0){
                get_gydata++;
            } else { get_gydata--; }
        }
        get_gzdata = -(gzdata*(250.0/32768.0));                // Modify Y gyro value to match geometrically
        for(i=0; i<= 3; i++){
            if(get_gzdata <= 0){
                get_gzdata++;
            } else { get_gzdata--; }
        }
        //  GET ANGULAR POSITION VALUE FROM ACCELEROMETER
        accgx = -(atan(azdata/sqrt(pow(axdata,2) + pow(aydata,2)))*(180.0/3.1416)); // Modify X gyro value to match geometrically
        accgy = atan(axdata/sqrt(pow(azdata,2) + pow(aydata,2)))*(180.0/3.1416);

        //  GET ANGULAR POSITION VALUE FROM GYROSCOPE
        gget_x[0] = get_gxdata + gget_x[1];    // Sum actual gyro value X + previous gyro value X
        gget_z[0] = get_gzdata + gget_z[1];    // Sum actual gyro value X + previous gyro value X
        gget_x[1] = gget_x[0];
        gget_z[1] = gget_z[0];

        //  COMPLEMENTARY FILTER
        imu.gx = 0.98*(accgx) + 0.02*(gget_x[0]);       // Average acc_gyroX and g_gyroX for complemetary filter
        imu.gy = 0.98*(accgy) + 0.02*(gget_z[0]);       // Average acc_gyroY and g_gyroY for complemetary filter

        //  AVERAGE FILTER
        total_x = total_x - readings_x[index_x];
        readings_x[index_x] = imu.gx;
        total_x = total_x + readings_x[index_x];
        index_x ++;
        if(index_x >= n_readings){
            index_x = 0;
        }
        average_x = total_x / n_readings;


        total_y = total_y - readings_y[index_y];
        readings_y[index_y] = imu.gy;
        total_y = total_y + readings_y[index_y];
        index_y ++;

        if(index_y >= n_readings){
            index_y = 0;
        }
        average_y = total_y / n_readings;

        //  PRINT VALUE FROM ACCELEROMETER AND GYROSCOPE
        /* Esto estaba comentado */
        printf("Informacion de giroscopio *********** \n");
        char texto[50];
        sprintf(texto,"Gx: %5.2f Gy: %5.2f --> Gx: %5.2f\t Gy: %5.2f \n",imu.gx, imu.gy,get_gxdata, get_gydata);
        WPRINT_APP_INFO(("Ax: %5.2f\t Ay: %5.2f\t Az: %5.2f\t", imu.ax, imu.ay, imu.az));
        WPRINT_APP_INFO(("Gx: %d\t Gy: %d\n\n\n\n", accgx, accgy));
        WPRINT_APP_INFO(("Gx: %d\t Gy: %d\n\n", gget_x[0], gget_z[0]));
        WPRINT_APP_INFO(("Gx: %d\t Gy: %d\n\n", imu.gx, imu.gy));
        WPRINT_APP_INFO(("Gx: %5.2f\t Gy: %5.2f\n\n", get_gxdata, get_gydata));

        wiced_uart_transmit_bytes(WICED_UART_1,"Giroscopio\n", strlen("Giroscopio\n"));
        wiced_uart_transmit_bytes(WICED_UART_1, texto, strlen(texto));
        /* Hasta aqui estaba comentado */


//        imu.gx=accgx;
//        imu.gy=accgy;

        return WICED_SUCCESS;

    }
}

float get_imu(uint8_t a){
        imu_read();
        if(a == 0x00){
            return (imu.ax);
        }
        if(a == 0x01){
            return (imu.ay);
        }
        if(a == 0x10){
            return (imu.az);
                }
        if(a == 0x11){
            return (imu.gx);
        }
        if(a == 0x100){
            return (imu.gy);
        }
    }
//
//void hc_protocol(){
//    imu_read();
//    //WPRINT_APP_INFO(("Gx: %d,\t Gy: %d\t, CONTADOR: %d\n\n", imu.gx, imu.gy, hc_counter));
//
//    imu.ax_actual = (int)imu.ax;
//    imu.ax_diff = imu.ax_actual - imu.ax_previous;
//    imu.ax_previous = imu.ax_actual;
//    imu.ay_actual = (int)imu.ay;
//    imu.ay_diff = imu.ay_actual - imu.ay_previous;
//    imu.ay_previous = imu.ay_actual;
//    imu.az_actual = (int)imu.az;
//    imu.az_diff = imu.az_actual - imu.az_previous;
//    imu.az_previous = imu.az_actual;
//
//    //WPRINT_APP_INFO(("Ax diff: %d ; Ay diff: %d ; Az diff: %d\n\n", imu.ax_diff, imu.ay_diff, imu.az_diff));
//    if(flag_hc_is_active){
//        if( (ACC_G_MAX < imu.ax_diff || imu.ax_diff < ACC_G_MIN) || (ACC_G_MAX < imu.ay_diff || imu.ay_diff < ACC_G_MIN) || (ACC_G_MAX < imu.az_diff || imu.az_diff < ACC_G_MIN) || (hc_gflag == WICED_TRUE) ){
//            hc_gflag = WICED_TRUE;
//            hc_counter ++;
//            sprintf(hc_counter_print,"%d\r\n", hc_counter);
//            if(hc_counter < imu_alert_count){
//                wiced_uart_transmit_bytes(WICED_UART_1, HC_G_ALERT, sizeof(HC_G_ALERT)-1);
//                wiced_uart_transmit_bytes(WICED_UART_1, HC_NO_MOVE, sizeof(HC_NO_MOVE)-1);
//                wiced_uart_transmit_bytes(WICED_UART_1, hc_counter_print, strlen(hc_counter_print)-1);
//            }
//            if(hc_counter >= imu_alert_count){
//
//                //CHECK IF GYROSCOPE HAVE ANY ANGLE HIGHER THAT
//                if( (G_MAX < imu.gx || imu.gx < G_MIN) || (G_MAX < imu.gy || imu.gy < G_MIN) ){
//                    wiced_uart_transmit_bytes(WICED_UART_1, HC_FALLED, sizeof(HC_FALLED)-1);
//                    wiced_uart_transmit_bytes(WICED_UART_1, hc_counter_print, strlen(hc_counter_print)-1);
//
//                    //CHECK IF ACCELEROMETER DON'T HAVE MOVEMENTES
//                    if( (ACC_MIN < imu.ax_diff && imu.ax_diff < ACC_MAX) && (ACC_MIN < imu.ay_diff && imu.ay_diff < ACC_MAX) && (ACC_MIN < imu.az_diff && imu.az_diff < ACC_MAX) ){
//                        if(hc_counter==(imu_alert_count*2)){
//                            faslh_imu=WICED_TRUE;
//                        }
//
//                        if(hc_counter >= (imu_alert_count*2)){                               // Equivalent to 30 seconds
//                            wiced_uart_transmit_bytes(WICED_UART_1, HC_WAKE_UP, sizeof(HC_WAKE_UP)-1);
//                            if(hc_counter == (imu_alert_count*2+25)){                           // Equivalent to 45 seconds
//                                wiced_rtos_stop_timer(&hc_timer);
//                                hc_flag = WICED_FALSE;
//                                GPIO_falledman=1;
//                                wiced_uart_transmit_bytes(WICED_UART_1, HC_ALERT_HC, sizeof(HC_ALERT_HC)-1);
//                            }
//                        }
//                    } else{hc_counter = 0; hc_gflag = WICED_FALSE; }
//                } else{hc_counter = 0; hc_gflag = WICED_FALSE;}
//            }
//        }
//
//    }
//
//
//}
//
//void wi_protocol(){
//    imu_read();
//    //WPRINT_APP_INFO(("%d\t%d\t%d\t%d\t%d\n", imu.gx, average_x, imu.gy, average_y, wi_counter));
////    wiced_gpio_output_high( WICED_LED1 );
//    if(BLE_flag == WICED_TRUE){
////        wiced_uart_transmit_bytes(WICED_UART_1, CENTER, sizeof(CENTER)-1);
//
//        if( (WIX_MIN < average_x && average_x < WIX_MAX) && (WIY_MIN < average_y && average_y < WIY_MAX) ){
////                wiced_gpio_output_high( WICED_LED1 );
//
//                // ******* AQUI SE DESACTIVA EL WIFI ********
//                wiced_uart_transmit_bytes(WICED_UART_3, BLE_CENTER_R, sizeof(BLE_CENTER_R)-1);
//
//
//        }
//        else{
////            wi_counter = 0;
//        }
//    }
//}

#endif /* _I2C_LSM6DSM_ */
