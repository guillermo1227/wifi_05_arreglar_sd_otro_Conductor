#ifndef _MANAGER_MENSSAGE_H
#define _MANAGER_MENSSAGE_H
#include "stdbool.h"
#include "wiced.h"

wiced_bool_t hide_on;



#define lasec         "----------------Lasec Telecomunicaciones, S.A.P.I. De C.V.---------------- \r\n"
#define VERSION     "Smart lamp 2.1 Version 5.8.2.15\r\n"
#define V_C         "SL:5.8.2.15"


// se le movio en la version 2.1.2.227 en la parte de escritura cuando se reinicia por servidor caido
// se le movio en la version 2.1.2.229 se habilito la configuracion por serial , y minimizo a la potencia AL; 50%
// se le movio en la version 2.1.2.231.1 se arreglo la parte del configurador en la toma de valores
// Smart lamp 2.1 Version 3.1: Cambio de nomenclatura en la versionificacion y optimiizacion de codigo
// Smart lamp 2.1 Version 3.3: Se volvio a integrar el setear la maca en cada inicio de la lampara y potencia de 29, version inestables
// Smart lamp 2.1 Version 3.6 : mejora de la version 3.3
// Smart lamp 2.1 Version 3.9 : Se guarda en memoria la bandera de la luz para habilitar la confirmacion despues dde una desconexion
// Smart lamp 2.1 Version 3.11 : Se agrega el envioo de pulso al bt para antiocolision
// Smart lamp 2.1 Version 4.1 :se cambia cadenas de envio y metodo para integrarse a la red
// Smart lamp 2.1 Version 4.3 :se cambia cadenas de envio y metodo para integrarse a la red
// Smart lamp 2.1 Version 4.6 :resolvio el problema de la confirmacion automatica
// Smart lamp 2.1 Version 4.8 :se integra mejoras en mac y en gpio de anticolision
// Smart lamp 2.1 Version 4.10 :se integra metodo de borrado de gpio
// Smart lamp 2.1 Version 4.12  :se desintegra el accionamiento del imu
// Smart lamp 2.1.1 B Version 5.8.2.15  :Se cambia potencia a 15 deb Y tambien se cambia tiempo de hilo de boton e hilo de tcp client


#define UART1           "UART 1 \r\n"
#define UART2           "UART 2 \r\n"

#define ENABLE          "ACCESS GRANTED\r\n"
#define DISABLE         "ACCESS DENIED\r\n"
#define BTN_EVENT       "SEG\r\n"
#define SOS_EVENT       "HELP EVENT\r\n"
#define CHARGER_EVENT   "CHARGER EVENT\r\n"

#define SET_NETWORK   "SETTING NETWORK\r\n"

#define        ERROR "No Value\r\n"
#define        TIMEOUT "TIMEOUT\r\n"

#define YES             "YES\r\n"
#define NOT             "NOT\r\n"
#define OK              "OK\r\n"
#define OK_S              "OK SAVE\r\n"
#define NOT_S             "NOT SAVE\r\n"

#define CREATE_FAILED    "CREATE SOCKET FAILED!\r\n"
#define TCP_BIND_FAILED    "TCP BIND FAILED!\r\n"
#define TCP_CONNECT_FAILED    "TCP CONNECT FAILED!\r\n"

#define BLE_ONLINE      "BLE Online\r\n"
#define WIFI_ONLINE      "WiFi Online\r\n"
#define BLE_SLEEP       "BLE sleep\r\n"
#define TEST_STR        "\r\nLasec\r\n"
#define BLE_CENTER_R     "apagame pls\r\n"

#define LIGTH_SEQUENCE       "LIGTH SEQUENCE\r\n"
#define LIGTH_SWEET         "LIGTH SWEET\r\n"
#define LIGTH_STRONG       "LIGTH STRONG\r\n"
#define LIGTH_NOT       "LIGTH NOT\r\n"
#define Pin_high       "pin high\r\n"
#define Pin_high_R       "pin high reset\r\n"


#define LAMP_READY     "ST4RT THE CHANGE\r\n"
#define NET_READY     "Ready to configure the network\r\n"
#define IMU_MSM       "INERTIAL MEASUREMENT UNIT\r\n"
#define ADC_MSM       "ANALOG DIGITAL CONVERTER\r\n"

#define OK_A        "OK_A\r\n"
#define OK_SC        "OK_S\r\n"
#define OK_K        "OK_K\r\n"
#define OK_G        "OK_G\r\n"
#define OK_I        "OK_I\r\n"
#define OK_M        "OK_M\r\n"
#define OK_MB       "MAC_BT\r\n"
#define OK_C        "OK_C\r\n"


#define OTA         "OTA\r\n"
#define NETWORK_FALIED "Bringing up network interface failed !\r\n"

#define GET_MAC_BT     "GETBT\r\n"

#define INICIO "INICIO\r\n"
#define ACCESO "ACCESO CORRECTO\r\n"


#define COM_ERROR           "\nCommunication i2c failed\r\n"
#define COM_EXITO           "\nCommunication i2c is complete\r\n"
#define PROBE_ERROR         "IMU sensor connection is't recognized\r\n"
#define PROBE_EXITO         "IMU sensor connection is done\r\n"
#define WAI_ERROR           "Had error while reading address WHO AM I (0x6A)\r\n\n"
#define WAI_EXITO           "It's OK while reading address WHO AM I\r\n\n"
#define STATUS_EXITO        "IMU SENSOR IT'S READY FOR GET VALUES\r\n"
#define HC_NO_MOVE          "\n\nTiempo sin moverse: "
#define HC_FALLED           "\nTiempo caido: "
#define HC_OK               "ALL IS WELL\r\n"
#define HC_MOVE_ON          "MOVE ON, MOVE ON, MOVE ON!\r\n\n"
#define HC_WAKE_UP          "WAKE UP, WAKE UP, WAKE UP!\r\n\n"
#define PUSH_BRAKE          "PUSH BRAKE, PUSH BRAKE, PUSH BRAKE\r\n\n"
#define HC_ALERT_BT         "SEA\n"
#define HC_ALERT_HC         "SEA\n"
#define HC_G_ALERT          "\n   * * * ACELERACION REPENTINA * * *\r\n"
#define CENTER              "CENTRO DE DESPACHO\r\n"


#define ZOM_BT      "SOM\r\n"

#endif  /* stdbool.h */
