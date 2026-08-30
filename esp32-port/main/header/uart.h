#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdbool.h>
#include "header/project_globals.h"

// dev branch'teki blink/header/uart.h ile ayni fonksiyon listesi.
// NOT: vUARTTask ve uart_receive_interrupt_handler bu dosyada DEGIL,
// dev'de de main.c'de - onlar ayri bir asamada (main.c'nin tek cekirdege
// gore task yapisi kurulurken) portlanacak. Bu dosya sadece "yardimci
// fonksiyonlar" katmani.

// UART Initialization
uint8_t initUART();
// UART gorevinin watchdog kalp atisi. Uzun sureli gonderimlerde (300 baud'da
// tek bir readout 30 saniyeyi asabiliyor) gorev mesaj dongusune donemedigi icin
// watchdog cihazi okumanin ortasinda resetliyordu. Gonderim donguleri bunu
// periyodik cagirmali.
//
// dev'de bu, task_health_flags icindeki WDT_FLAG_UART bitini tazeliyordu;
// ESP portunda karsiligi ESP-IDF'in kendi Task Watchdog Timer'ini (TWDT)
// beslemek. Cagiran gorev TWDT'ye kayitli degilse sessizce hicbir sey yapmaz.
void uartTaskHeartbeat(void);
// uart_puts + kalp atisi. Cok satirli gonderimlerde bunu kullan.
void uartSendLine(const char *line);
// uart_wait_tx_done + kalp atisi. Suresiz bloklamak yerine parcali bekler.
void uartWaitTxDone(void);
// This function check the data which comes when State is Listening, and compares the message to defined strings, and returns a ListeningState value to process the request
enum ListeningStates checkListeningData(uint8_t *data_buffer, uint8_t size);
// This function deletes a character from a given string
void deleteChar(uint8_t *str, uint8_t len, char chr);
void parseACRequestDate(uint8_t *buffer, uint8_t *start_date, uint8_t *end_date);
// This function sets the device's baud rate according to given number like 0,1,2,3,4,5,6
void set_device_baud_rate(uint8_t b_rate);
uint8_t verifyHourMinSec(uint8_t hour, uint8_t min, uint8_t sec);
uint8_t verifyYearMonthDay(uint8_t year, uint8_t month, uint8_t day);
// This function sets time via UART
void setTimeFromUART(uint8_t *buffer);
// This function sets date via UART
void setDateFromUART(uint8_t *buffer);
// This function generates a production info message and sends it to UART
void sendProductionInfo();
// This function gets a password and controls the password, if password is true, device sends an ACK message, if not, device sends NACK message
void passwordHandler(uint8_t *buffer);
void setThresholdValue(uint8_t *data);
#if CONF_THRESHOLD_PIN_ENABLED
void resetThresholdPIN();
void setThresholdPIN();
#endif
void readTime();
void readDate();
void readSerialNumber();
void sendLastVRMSXValue(enum ListeningStates vrmsState);
void sendThresholdObis();
bool control_serial_number(uint8_t *identification_req_buf, size_t req_size);
size_t create_identify_response_message(char *response_buf, size_t buf_size);
uint8_t exract_baud_rate_and_mode_from_message(uint8_t *msg_buf, size_t msg_len, int8_t *requested_mode);
void send_readout_message(uint8_t request_mode);
void set_init_baud_rate();
void send_programming_acknowledgement();
uint8_t is_message_break_command(uint8_t *buf);

#endif
