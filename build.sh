#!/bin/bash
cd build
cmake -DPICO_BOARD=pico_w .. && make main
cd ..
