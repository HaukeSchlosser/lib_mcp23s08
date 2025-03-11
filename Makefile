CC = gcc

CFLAGS = -Wall -fPIC
LDFLAGS = -shared

SRC_DIR = src
BUILD_DIR = build

SRC = $(SRC_DIR)/lib_mcp23s08.c
OBJ = $(BUILD_DIR)/lib_mcp23s08.o
TARGET = $(BUILD_DIR)/lib_mcp23s08.so
HEADER = $(SRC_DIR)/lib_mcp23s08.h

PREFIX = /usr/local
LIBDIR = $(PREFIX)/lib
INCLUDEDIR = $(PREFIX)/include

all: $(TARGET)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(OBJ): $(SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET): $(OBJ)
	$(CC) $(LDFLAGS) -o $@ $<

install: $(TARGET)
	sudo cp $(TARGET) $(LIBDIR)/
	sudo cp $(HEADER) $(INCLUDEDIR)/
	sudo ldconfig
	@echo "Library installed to $(LIBDIR)/$(TARGET)"
	@echo "Header installed to $(INCLUDEDIR)/$(HEADER)"

uninstall:
	sudo rm -f $(LIBDIR)/lib_mcp23s08.so
	sudo rm -f $(INCLUDEDIR)/lib_mcp23s08.h
	sudo ldconfig
	@echo "Library removed from $(LIBDIR)/lib_mcp23s08.so"
	@echo "Header removed from $(INCLUDEDIR)/lib_mcp23s08.h"

clean:
	rm -rf $(BUILD_DIR)
	@echo "Cleaned up build files"

cleanall: clean uninstall
