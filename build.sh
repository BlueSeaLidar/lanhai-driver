#!/bin/sh

mkdir bin
gcc -c -o ./bin/uart.o uart.c
# g++ -o record record.cpp uart.o
g++ -o ./bin/uart-demo parser.cpp user.cpp uart_demo.cpp ./bin/uart.o
g++ -o ./bin/udp-demo parser.cpp user.cpp udp_demo.cpp ./bin/uart.o
g++ -o ./bin/tcp-demo parser.cpp user.cpp tcp_demo.cpp ./bin/uart.o
