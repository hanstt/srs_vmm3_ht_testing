#!/bin/bash

./filter_vmm \
	--label=FILT \
	--port=8000 \
	--drasi=localhost \
	--server=drasi,dest=localhost:9000 \
	--merge-mode=event \
	--merge-no-validate \
	--buf=size=1Gi \
	--max-ev-size=0x100000 \
	--server=stream:8001
