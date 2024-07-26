cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(pico_corolla)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

if (TARGET tinyusb_device)
	# rest of your project
	include_directories($ENV{PICO_SDK_PATH}/src/rp2_common/cmsis/stub/CMSIS/Core/Include)
	include_directories($ENV{PICO_SDK_PATH}/src/rp2_common/cmsis/stub/CMSIS/Device/RaspberryPi/RP2040/Include)
	include_directories(include)

	add_executable(main
		src/main.c

		include/can2040.c
		include/can2040.h
	)

	target_link_libraries(main pico_stdlib)

	pico_add_extra_outputs(main)

	# enable usb output, disable uart output
	pico_enable_stdio_usb(main 1)
	pico_enable_stdio_uart(main 0)
	# create map/bin/hex/uf2 file etc.
	pico_add_extra_outputs(main)
elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()

