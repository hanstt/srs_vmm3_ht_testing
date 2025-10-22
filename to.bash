#!/bin/bash

. common.bash

set -x

#../drasi/bin/lwrocmerge\
/home/hydra-tpc/r3b/R3Bsw/drasi/bin/lwrocmerge\
        --label=hydra\
        --port=$TO_PORT\
        --merge-mode=wr\
        --server=stream:$TO_PORTstream\
        --server=trans:$TO_PORTtrans\
        --buf=size=2Gi\
        --max-ev-size=0x410000\
        --merge-ts-nodata-warn=30s \
        --merge-ts-analyse-ref=17 \
        --drasi=$TPC_NODE:$TPC_PORT\
        --drasi=$PLASTIC_NODE:$PLASTIC_PORT\
        --file-writer

#        --drasi=$RIO_NODE:$RIO_PORT\


#        --merge-ts-analyse-ref=16
