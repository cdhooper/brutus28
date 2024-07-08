BOARD_REV ?= 2

DEFS := -DBOARD_REV=$(BOARD_REV)

PROGS=brutus term
all: $(PROGS)

brutus: brutus.c
	cc -g -O3 -o $@ $< $(DEFS)

term: term.c
	cc -g -O3 -o $@ $< $(DEFS) -lpthread

clean:
	rm -f $(PROGS)
