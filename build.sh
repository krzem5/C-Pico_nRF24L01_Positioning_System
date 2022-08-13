#!/bin/bash
if [ ! -d "build" ]; then
	mkdir build
	cd build
	cmake ..
	cd ..
fi
cd build
make -j16&&[[ -d "$PICO_DRIVE_PATH" ]]&&cp nrf24l01_positioning_system.uf2 "$PICO_DRIVE_PATH/nrf24l01_positioning_system.uf2"
cd ..
