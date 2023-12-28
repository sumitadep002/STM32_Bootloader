
#include "main.h"
#include <stdio.h>
#ifdef LINUX_HOST

/* If you are here, then you may be trying to run this STM32_Programmer Application on Linux Host.
   Note1 : This file should implement the functions mentioned in the "LinuxSerialPort.h"
   Note2 : Take a reference from the source file "WindowsSerialPort.c
 */

 #include "Serial.h"

/* Code Begin */

void Serial_Port_Configuration(void)
{
printf("Work In Progress..\n");
}
uint32_t read_serial_port(uint8_t *pBuffer, uint32_t len)
{
printf("Work In Progress..\n");
}
void Close_serial_port(void)
{
printf("Work In Progress..\n");
}
void purge_serial_port(void)
{
printf("Work In Progress..\n");
}
void Write_to_serial_port(uint8_t *data_buf, uint32_t len)
{
printf("Work In Progress..\n");
}


/* Code End */

 #endif
