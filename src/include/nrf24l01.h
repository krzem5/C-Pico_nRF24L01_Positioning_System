#ifndef __NRF24L01_H__
#define __NRF24L01_H__ 1
#include <stdint.h>



#define NRF24L01_ID_MASK 0xffffffffff
#define NRF24L01_CHANNEL_MASK 127



typedef uint64_t nrf24l01_id_t;



typedef uint8_t nrf24l01_channel_t;



typedef struct _NRF24L01_PINS{
	uint8_t ce;
	uint8_t csn;
	uint8_t sck;
	uint8_t tx;
	uint8_t rx;
	spi_inst_t* spi;
} nrf24l01_pins_t;



typedef struct _NRF24L01_ANTENA{
	nrf24l01_pins_t pins;
	nrf24l01_id_t message_id;
	nrf24l01_channel_t channel;
} nrf24l01_antena_t;



typedef struct _NRF24L01_RECEIVER{
	nrf24l01_pins_t pins;
	nrf24l01_id_t message_id;
	nrf24l01_channel_t channel;
} nrf24l01_receiver_t;



void nrf24l01_antena_broadcast(const nrf24l01_antena_t* antena);



void nrf24l01_antena_init(const nrf24l01_pins_t* pins,nrf24l01_id_t message_id,nrf24l01_channel_t channel,nrf24l01_antena_t* antena);



void nrf24l01_receiver_init(const nrf24l01_pins_t* pins,nrf24l01_id_t message_id,nrf24l01_channel_t channel,nrf24l01_receiver_t* receiver);



void nrf24l01_receiver_update(const nrf24l01_receiver_t* receiver);



#endif
