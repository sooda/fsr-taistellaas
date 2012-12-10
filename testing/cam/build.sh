#!/bin/bash

# LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/u/76/tliski/unix/Documents/fsr/ext-libs/lib/ ./test

export PKG_CONFIG_PATH=/u/76/tliski/unix/Documents/fsr/ext-libs/lib/pkgconfig
g++ test.cpp -o test `pkg-config opencv --cflags --libs` -I/u/76/tliski/unix/Documents/fsr/ext-libs/include/
