SRC = ../../main/madgwick.c ../../main/pdr.c ../../main/pf.c ../../main/range_filter.c
CFLAGS = -O2 -Wall -Wextra
HDR = $(wildcard ../../main/*.h)

all: sim replay

sim: sim.c $(SRC) $(HDR)
	$(CC) $(CFLAGS) -o $@ sim.c $(SRC) -lm

replay: replay.c ../../main/nav_core.c ../../main/pf.c ../../main/range_filter.c $(HDR)
	$(CC) $(CFLAGS) -o $@ replay.c ../../main/nav_core.c ../../main/pf.c ../../main/range_filter.c -lm

LOG ?= ../../../../../../logs.txt/2890\ MOTION\ gyro\ bias.txt

test: sim
	./sim

replay-log: replay
	./replay < $(LOG)

clean:
	rm -f sim replay
