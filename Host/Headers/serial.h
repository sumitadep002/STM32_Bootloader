#ifndef SERIAL_H
#define SERIAL_H

//Serial port related prototypes
void Serial_Port_Configuration(void);
uint32_t read_serial_port(uint8_t *pBuffer, uint32_t len);
void Close_serial_port(void);
void purge_serial_port(void);
void Write_to_serial_port(uint8_t *data_buf, uint32_t len);


#endif // SERIAL_H
