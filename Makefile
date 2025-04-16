FUSER:=fuser
GEN:=gen
TEST:=test
ROOTORAMA:=rootorama
UNGRAY:=ungray
UNGRAY_TABLE:=ungray_table.h

CFLAGS:=-ggdb -Werror -O3
ROOT_CFLAGS:=$(shell root-config --cflags)
LDFLAGS:=
ROOT_LIBS:=$(shell root-config --libs)

all: $(GEN) $(TEST) $(ROOTORAMA)

$(FUSER): fuser.o
	$(CC) $(LDFLAGS) -o $@ $<

$(GEN): gen.o
	$(CC) $(LDFLAGS) -o $@ $<

$(TEST): test.o
	$(CC) $(LDFLAGS) -o $@ $<

$(ROOTORAMA): rootorama.o
	$(CXX) $(LDFLAGS) -o $@ $< $(ROOT_LIBS)

test.o rootorama.o: $(UNGRAY_TABLE)

$(UNGRAY_TABLE): $(UNGRAY)
	./$< > $@.tmp
	mv -f $@.tmp $@

$(UNGRAY): ungray.o
	$(CC) $(LDFLAGS) -o $@ $^

%.o: %.c Makefile
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.cpp Makefile
	$(CXX) $(CFLAGS) $(ROOT_CFLAGS) -c -o $@ $<

clean:
	rm -f *.o $(FUSER) $(GEN) $(TEST)
