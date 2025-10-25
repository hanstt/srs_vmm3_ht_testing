#!/bin/bash

. common.bash

#valgrind --leak-check=full --show-reachable=yes \
#gdb --args \
./filter_vmm \
	--label=FILT \
	--port=$TPC_PORT \
	--drasi=localhost \
	--merge-mode=event \
	--merge-no-validate \
	--buf=size=6Gi \
	--max-ev-size=0x1000000 \
	--server=stream:8001
