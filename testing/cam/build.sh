#!/bin/bash

g++ eigentest.cpp -o test `pkg-config opencv --cflags --libs` -I/usr/include/eigen3
