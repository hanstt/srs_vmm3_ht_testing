1. Check if screen session runs:
screen -ls

If it does, goto 3.

2. Create screen session:
cd hydra-drasi/judypee/srs_vmm3_ht_testing
screen -S hydra_tpc_daq

3. Check that screen windows are prepared:
0 log  1 tree  2 srsc  3 fuser  4 filter

If all are available, goto 5

4. Prepare windows:
ctrl+a A, type 'log'
./log.bash
ctrl+c
ctrl+a A, type 'tree'
./tree.bash
ctrl+c
ctrl+a A, type 'srsc'
# Don't run anything here yet.
ctrl+a c
ctrl+a A, type 'fuser'
# Don't run anything here yet.
ctrl+a c
ctrl+a A, type 'filter'
# Don't run anything here yet.

5. To stop daq:
ctrl+a 2, ctrl+c
ctrl+a n, ctrl+c
ctrl+a n, ctrl+c

6. To start daq:
ctrl+a 2
./srsc.bash
ctrl+a n
./fuser.bash
ctrl+a n
./filter.bash

7. To destroy the whole session:
ctrl+a \
