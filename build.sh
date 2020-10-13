#!/bin/sh

gcc -c uart.c
g++ -o lidar demo.cpp uart.o
g++ -o record record.cpp uart.o
