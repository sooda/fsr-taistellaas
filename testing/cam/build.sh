#!/bin/bash

# LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/u/76/tliski/unix/Documents/fsr/ext-libs/lib/ ./test

#export PKG_CONFIG_PATH=/u/76/tliski/unix/Documents/fsr/ext-libs/lib/pkgconfig
g++ hsv_test.cpp -o test `pkg-config opencv --cflags --libs` -I/usr/include/opencv
