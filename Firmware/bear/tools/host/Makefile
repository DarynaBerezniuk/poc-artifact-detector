SRC = ../../main/madgwick.c ../../main/pdr.c ../../main/pf.c ../../main/range_filter.c
CFLAGS = -O2 -Wall -Wextra

all: sim replay

sim: sim.c $(SRC)
	$(CC) $(CFLAGS) -o $@ $^ -lm

replay: replay.c ../../main/pf.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

test: sim
	./sim

clean:
	rm -f sim replay
