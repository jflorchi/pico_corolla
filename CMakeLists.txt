cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(pico_corolla)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

if (TARGET tinyusb_device)
	# Add ./include to the global include path (can be refined later if needed)
	include_directories(include)

	# Define the can2040 library
	add_library(can2040_lib
		include/can2040.c
	)
	# Explicitly add the directory containing RP2040.h and core_cm0plus.h for can2040_lib
	target_include_directories(can2040_lib PRIVATE
		${PICO_SDK_PATH}/src/rp2_common/cmsis/stub/CMSIS/Device/RP2040/Include
		${PICO_SDK_PATH}/src/rp2_common/cmsis/stub/CMSIS/Core/Include
	)
	# Link can2040 library against pico_stdlib AND hardware_pio to get SDK includes/libs
	target_link_libraries(can2040_lib PRIVATE pico_stdlib hardware_pio)

	# Define the main executable
	add_executable(main
		src/main.c
		# can2040.h is needed by main.c, but included via include_directories()
	)

	# Explicitly add the directory containing RP2040.h and core_cm0plus.h for main.c
	target_include_directories(main PRIVATE
		${PICO_SDK_PATH}/src/rp2_common/cmsis/stub/CMSIS/Device/RP2040/Include
		${PICO_SDK_PATH}/src/rp2_common/cmsis/stub/CMSIS/Core/Include
	)

	# Link main against pico_stdlib, our can2040 library, hardware_spi, and hardware_adc
	target_link_libraries(main PRIVATE pico_stdlib can2040_lib hardware_spi hardware_adc)

	pico_add_extra_outputs(main)

	# enable usb output, disable uart output
	pico_enable_stdio_usb(main 1)
	pico_enable_stdio_uart(main 0)
	# create map/bin/hex/uf2 file etc.
	pico_add_extra_outputs(main)
elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()
