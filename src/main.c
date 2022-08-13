#include <hardware/spi.h>
#include <nrf24l01.h>
#include <pico/bootrom.h>
#include <pico/stdlib.h>



#define CHANNEL 12
#define MESSAGE_ID 0x3132313231
#define TYPE 1



int main(){
	nrf24l01_pins_t pins={
		0,
		1,
		2,
		3,
		4,
		spi0
	};
#if TYPE==0
	nrf24l01_antena_t antena;
	nrf24l01_antena_init(&pins,MESSAGE_ID,CHANNEL,&antena);
	while (1){
		nrf24l01_antena_broadcast(&antena);
		sleep_ms(1);
	}
#else
	stdio_init_all();
	stdio_usb_init();
	nrf24l01_receiver_t receiver;
	nrf24l01_receiver_init(&pins,MESSAGE_ID,CHANNEL,&receiver);
	while (getchar_timeout_us(1)==PICO_ERROR_TIMEOUT){
		nrf24l01_receiver_update(&receiver);
	}
	reset_usb_boot(0,0);
#endif
}
