#!/bin/bash

../drasi/bin/lwrocmerge \
	--label=TO \
	--port=9000 \
	--merge-mode=wr \
	--server=trans:9001,nohold \
	--server=stream:9002 \
	--buf=size=100Mi \
	--max-ev-size=0x100000 \
	--drasi=localhost:8000
