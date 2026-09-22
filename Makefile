# Makefile to build the project
# NOTE: This file must not be changed.

# Parameters
CC = gcc
CFLAGS = -Wall

BIN = bin/
CABLE = cable/
SRC = src/

TX_SERIAL_PORT = /tmp/ttyS10
RX_SERIAL_PORT = /tmp/ttyS11
BAUD_RATE = 9600

TX_FILE = penguin.gif
RX_FILE = penguin-received.gif

# Main
.PHONY: all
all: main cable

main: $(SRC)/*.c | bin_dir
	$(CC) $(CFLAGS) -o $(BIN)/$@ $^

.PHONY: run_tx
run_tx: main
	./$(BIN)/main $(TX_SERIAL_PORT) $(BAUD_RATE) tx $(TX_FILE)

.PHONY: run_rx
run_rx: main
	./$(BIN)/main $(RX_SERIAL_PORT) $(BAUD_RATE) rx $(RX_FILE)

# File check
.PHONY: check_files
check_files:
	diff -s $(TX_FILE) $(RX_FILE) || exit 0

# Cable
cable: $(CABLE)/cable.c
	$(CC) $(CFLAGS) -o $(BIN)/$@ $^

.PHONY: run_cable
run_cable: cable
	@which socat || { echo "Error: Could not find socat. Install socat and try again."; exit 1; }
	./$(BIN)/cable

# Create bin directory
.PHONY: bin_dir
bin_dir:
	mkdir -p $(BIN)

# Clean
.PHONY: clean
clean:
	rm -f $(BIN)/main
	rm -f $(BIN)/cable
	rm -f $(RX_FILE)
