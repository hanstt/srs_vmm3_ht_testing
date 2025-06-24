CC:=gcc
DRASI_CONFIG:=../drasi/bin/drasi-config.sh

FUSER:=fuser
GEN:=gen
TEST:=test
ROOTORAMA:=rootorama
UNGRAY:=ungray
UNGRAY_TABLE:=ungray_table.h
FILTER_VMM:=filter_vmm

CFLAGS:=-ggdb -Werror -O3
ROOT_CFLAGS:=$(shell root-config --cflags)
LDFLAGS:=
ROOT_LIBS:=$(shell root-config --libs)

all: $(GEN) $(TEST) $(ROOTORAMA) $(FILTER_VMM) heaptest

$(FUSER): fuser.o
	$(CC) $(LDFLAGS) -o $@ $<

$(GEN): gen.o
	$(CC) $(LDFLAGS) -o $@ $<

$(TEST): test.o
	$(CC) $(LDFLAGS) -o $@ $<

$(ROOTORAMA): rootorama.o
	$(CXX) $(LDFLAGS) -o $@ $< $(ROOT_LIBS)

$(FILTER_VMM): filter_vmm.o
	$(CC) $(LDFLAGS) -o $@ $< $(ROOT_LIBS) $(shell CC=gcc $(DRASI_CONFIG) --merge --libs)

test.o rootorama.o: $(UNGRAY_TABLE)

$(UNGRAY_TABLE): $(UNGRAY)
	./$< > $@.tmp
	mv -f $@.tmp $@

$(UNGRAY): ungray.o
	$(CC) $(LDFLAGS) -o $@ $<

heaptest: heaptest.o
	$(CC) -o $@ $<
heaptest.o: heap.h

%.o: %.c Makefile
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.cpp Makefile
	$(CXX) $(CFLAGS) $(ROOT_CFLAGS) -c -o $@ $<

filter_vmm.o: filter_vmm.c Makefile
	$(CC) $(CFLAGS) $(shell CC=gcc $(DRASI_CONFIG) --cflags) -c -o $@ $<

clean:
	rm -f *.o $(FUSER) $(GEN) $(TEST) $(FILTER_VMM)
