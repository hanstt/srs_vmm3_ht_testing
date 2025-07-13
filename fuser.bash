#!/bin/bash

./build/m_read_meb \
 --label=SRS \
 --srs=../srs-debian-bullseye-x86-64-v0.1.3-root-6-32-08/bin/srs_control \
 --buf=size=100Mi --max-ev-size=0x10000 \
 --server=drasi,dest=localhost:8000 --server=trans --server=stream
