#!/bin/bash

#valgrind \
#gdb --args \
./f_user \
 --label=SRS \
 --fifo=fifo.bin \
 --buf=size=1Gi \
 --max-ev-size=0x10000 \
 --server=drasi,dest=localhost:8000 --server=trans --server=stream
