#!/bin/bash

g++ test.cpp -o test `pkg-config opencv --cflags --libs`
