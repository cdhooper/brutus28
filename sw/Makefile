BOARD_REV ?= 2

DEFS := -DBOARD_REV=$(BOARD_REV)

brutus: brutus.c
	cc -g -O3 -o $@ $< $(DEFS)
