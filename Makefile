FUSER:=fuser
GEN:=gen
TEST:=test
UNGRAY:=ungray
UNGRAY_TABLE:=ungray_table.h

CFLAGS:=-ggdb -pg -Werror -Wshadow
LDFLAGS:=-pg

all: $(GEN) $(TEST)

$(FUSER): fuser.o
	$(CC) $(LDFLAGS) -o $@ $<

$(GEN): gen.o
	$(CC) $(LDFLAGS) -o $@ $<

$(TEST): test.o $(UNGRAY_TABLE)
	$(CC) $(LDFLAGS) -o $@ $<

$(UNGRAY_TABLE): $(UNGRAY)
	./$< > $@.tmp
	mv -f $@.tmp $@

$(UNGRAY): ungray.o
	$(CC) $(LDFLAGS) -o $@ $^

%.o: %.c Makefile
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.cpp Makefile
	$(CXX) $(CFLAGS) -c -o $@ $<

clean:
	rm -f *.o $(FUSER) $(GEN) $(TEST)
