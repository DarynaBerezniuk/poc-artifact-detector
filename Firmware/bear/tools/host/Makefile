SRC = ../../main/madgwick.c ../../main/pdr.c ../../main/pf.c ../../main/range_filter.c
CFLAGS = -O2 -Wall -Wextra
HDR = $(wildcard ../../main/*.h)

all: sim replay

sim: sim.c $(SRC) $(HDR)
	$(CC) $(CFLAGS) -o $@ sim.c $(SRC) -lm

replay: replay.c ../../main/pf.c $(HDR)
	$(CC) $(CFLAGS) -o $@ replay.c ../../main/pf.c -lm

test: sim
	./sim

clean:
	rm -f sim replay
