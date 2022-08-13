#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <nrf24l01.h>
#include <nrf24l01_constants.h>
#include <pico/bootrom.h>
#include <stdio.h>



#define SET_CSN(pins,state) \
	do{ \
		gpio_put((pins)->csn,(state)); \
		sleep_us(5); \
	} while (0)

#define PAYLOAD_SIZE 8



static uint8_t _get_status(const nrf24l01_pins_t* pins){
	uint8_t command=COMMAND_NOP;
	uint8_t status;
	spi_init(pins->spi,10000000);
	spi_set_format(pins->spi,8,SPI_CPOL_0,SPI_CPHA_0,SPI_MSB_FIRST);
	SET_CSN(pins,0);
	spi_write_read_blocking(pins->spi,&command,&status,1);
	SET_CSN(pins,1);
	spi_deinit(pins->spi);
	return status;
}



static void _write_command(const nrf24l01_pins_t* pins,uint8_t command){
	uint8_t status;
	spi_init(pins->spi,10000000);
	spi_set_format(pins->spi,8,SPI_CPOL_0,SPI_CPHA_0,SPI_MSB_FIRST);
	SET_CSN(pins,0);
	spi_write_read_blocking(pins->spi,&command,&status,1);
	SET_CSN(pins,1);
	spi_deinit(pins->spi);
}



static void _write_register(const nrf24l01_pins_t* pins,uint8_t regsiter,uint8_t value){
	uint8_t write_buffer[2]={COMMAND_W_REGISTER|regsiter,value};
	uint8_t read_buffer[2];
	spi_init(pins->spi,10000000);
	spi_set_format(pins->spi,8,SPI_CPOL_0,SPI_CPHA_0,SPI_MSB_FIRST);
	SET_CSN(pins,0);
	spi_write_read_blocking(pins->spi,write_buffer,read_buffer,2);
	SET_CSN(pins,1);
	spi_deinit(pins->spi);
}



static void _write_register_address(const nrf24l01_pins_t* pins,uint8_t regsiter,nrf24l01_id_t address){
	uint8_t write_buffer[6]={COMMAND_W_REGISTER|regsiter,address,address>>8,address>>16,address>>24,address>>32};
	uint8_t read_buffer[6];
	spi_init(pins->spi,10000000);
	spi_set_format(pins->spi,8,SPI_CPOL_0,SPI_CPHA_0,SPI_MSB_FIRST);
	SET_CSN(pins,0);
	spi_write_read_blocking(pins->spi,write_buffer,read_buffer,6);
	SET_CSN(pins,1);
	spi_deinit(pins->spi);
}



static void _init_pins(const nrf24l01_pins_t* pins){
	gpio_init(pins->ce);
	gpio_init(pins->csn);
	gpio_set_dir(pins->ce,GPIO_OUT);
	gpio_set_dir(pins->csn,GPIO_OUT);
	gpio_set_function(pins->sck,GPIO_FUNC_SPI);
	gpio_set_function(pins->tx,GPIO_FUNC_SPI);
	gpio_set_function(pins->rx,GPIO_FUNC_SPI);
	gpio_put(pins->ce,0);
	gpio_put(pins->csn,1);
	sleep_ms(5);
}



void nrf24l01_antena_broadcast(const nrf24l01_antena_t* antena){
	uint64_t time=to_us_since_boot(get_absolute_time());
	uint8_t write_buffer[PAYLOAD_SIZE+1]={COMMAND_W_TX_PAYLOAD};
	*((uint64_t*)(write_buffer+1))=time;
	uint8_t read_buffer[PAYLOAD_SIZE+1];
	spi_init(antena->pins.spi,10000000);
	spi_set_format(antena->pins.spi,8,SPI_CPOL_0,SPI_CPHA_0,SPI_MSB_FIRST);
	SET_CSN(&(antena->pins),0);
	spi_write_read_blocking(antena->pins.spi,write_buffer,read_buffer,PAYLOAD_SIZE+1);
	SET_CSN(&(antena->pins),1);
	spi_deinit(antena->pins.spi);
	gpio_put(antena->pins.ce,1);
	while (!(_get_status(&(antena->pins))&(BIT_TX_DS|BIT_MAX_RT)));
	gpio_put(antena->pins.ce,0);
	_write_register(&(antena->pins),REGISTER_STATUS,BIT_RX_DR|BIT_TX_DS|BIT_MAX_RT);
}



void nrf24l01_antena_init(const nrf24l01_pins_t* pins,nrf24l01_id_t message_id,nrf24l01_channel_t channel,nrf24l01_antena_t* antena){
	message_id&=NRF24L01_ID_MASK;
	channel&=NRF24L01_CHANNEL_MASK;
	antena->pins=*pins;
	antena->message_id=message_id;
	antena->channel=channel;
	_init_pins(pins);
	_write_register(pins,REGISTER_CONFIG,0);
	_write_register(pins,REGISTER_EN_AA,0);
	_write_register(pins,REGISTER_EN_RXADDR,0);
	_write_register(pins,REGISTER_SETUP_AW,0b11);
	_write_register(pins,REGISTER_SETUP_RETR,0);
	_write_register(pins,REGISTER_RF_CH,channel);
	_write_register(pins,REGISTER_RF_SETUP,1);
	for (uint8_t i=0;i<6;i++){
		_write_register(pins,REGISTER_RX_ADDR_P0+i,0);
		_write_register(pins,REGISTER_RX_PW_P0+i,PAYLOAD_SIZE);
	}
	_write_register_address(pins,REGISTER_TX_ADDR,message_id);
	_write_register(pins,REGISTER_DYNPD,0);
	_write_register(pins,REGISTER_FEATURE,0);
	_write_register(pins,REGISTER_STATUS,BIT_RX_DR|BIT_TX_DS|BIT_MAX_RT);
	_write_command(pins,COMMAND_FLUSH_TX);
	_write_command(pins,COMMAND_FLUSH_RX);
	_write_register(pins,REGISTER_CONFIG,BIT_PWR_UP);
	sleep_ms(100);
}



void nrf24l01_receiver_init(const nrf24l01_pins_t* pins,nrf24l01_id_t message_id,nrf24l01_channel_t channel,nrf24l01_receiver_t* receiver){
	message_id&=NRF24L01_ID_MASK;
	channel&=NRF24L01_CHANNEL_MASK;
	receiver->pins=*pins;
	receiver->message_id=message_id;
	receiver->channel=channel;
	_init_pins(pins);
	_write_register(pins,REGISTER_CONFIG,0);
	_write_register(pins,REGISTER_EN_AA,0);
	_write_register(pins,REGISTER_EN_RXADDR,BIT_ERX_P1);
	_write_register(pins,REGISTER_SETUP_AW,0b11);
	_write_register(pins,REGISTER_SETUP_RETR,0);
	_write_register(pins,REGISTER_RF_CH,channel);
	_write_register(pins,REGISTER_RF_SETUP,1);
	for (uint8_t i=0;i<6;i++){
		_write_register_address(pins,REGISTER_RX_ADDR_P0+i,0);
		_write_register(pins,REGISTER_RX_PW_P0+i,PAYLOAD_SIZE);
	}
	_write_register_address(pins,REGISTER_RX_ADDR_P1,message_id);
	_write_register_address(pins,REGISTER_TX_ADDR,0);
	_write_register(pins,REGISTER_DYNPD,0);
	_write_register(pins,REGISTER_FEATURE,0);
	_write_register(pins,REGISTER_STATUS,BIT_RX_DR|BIT_TX_DS|BIT_MAX_RT);
	_write_command(pins,COMMAND_FLUSH_TX);
	_write_command(pins,COMMAND_FLUSH_RX);
	_write_register(pins,REGISTER_CONFIG,BIT_PWR_UP|BIT_PRIM_RX);
	sleep_ms(100);
	gpio_put(pins->ce,1);
}



void nrf24l01_receiver_update(const nrf24l01_receiver_t* receiver){
	if (_get_status(&(receiver->pins))&BIT_RX_DR){
		uint8_t write_buffer[PAYLOAD_SIZE+1]={COMMAND_R_RX_PAYLOAD};
		uint8_t read_buffer[PAYLOAD_SIZE+1];
		spi_init(receiver->pins.spi,10000000);
		spi_set_format(receiver->pins.spi,8,SPI_CPOL_0,SPI_CPHA_0,SPI_MSB_FIRST);
		SET_CSN(&(receiver->pins),0);
		spi_write_read_blocking(receiver->pins.spi,write_buffer,read_buffer,PAYLOAD_SIZE+1);
		SET_CSN(&(receiver->pins),1);
		spi_deinit(receiver->pins.spi);
		_write_register(&(receiver->pins),REGISTER_STATUS,BIT_RX_DR);
		uint64_t time=*((uint64_t*)(read_buffer+1));
		printf("%llu\n",time);
	}
}
