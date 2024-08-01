#include "stdbool.h"
#include "wiced.h"
#include "IMU/i2c_lsm6dsm.h"


void displayThread(wiced_thread_arg_t arg);

void girsocopeThread(wiced_thread_arg_t arg)
{
    uint8_t queue_str[2];
    while(1)
    {
        //wiced_rtos_get_semaphore(&displaySemaphore, WICED_WAIT_FOREVER);
        //wiced_rtos_pop_from_queue(&pubQueue, &queue_str, WICED_WAIT_FOREVER);
        wiced_rtos_lock_mutex( &i2cMutex); /* Agregado por mi */
        wiced_uart_transmit_bytes(WICED_UART_1,"Tomo hilo sensor\n",strlen("Tomo hilo sensor\n"));
        imu_read();
        wiced_rtos_unlock_mutex(&i2cMutex);
    }
}
