#!/bin/bash

./filter_vmm \
	--label=FILT \
	--port=8000 \
	--drasi=localhost \
	--merge-mode=event \
	--merge-no-validate \
	--buf=size=100Mi \
	--max-ev-size=65536 \
	--server=stream:8001
