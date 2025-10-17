CC = gcc

CFLAGS  = -Wall -O2 -fPIC
LDFLAGS = -shared -Wl,-soname,libmcp23s08.so.0

SRC_DIR   = src
BUILD_DIR = build

SRC    = $(SRC_DIR)/lib_mcp23s08.c
OBJ    = $(BUILD_DIR)/lib_mcp23s08.o

LIBBASE = libmcp23s08
MAJ = 0
MIN = 1

TARGET    = $(BUILD_DIR)/$(LIBBASE).so.$(MAJ).$(MIN)
SONAME    = $(LIBBASE).so.$(MAJ)
DEVELINK  = $(LIBBASE).so
COMPAT    = lib_mcp23s08.so

HEADER = $(SRC_DIR)/lib_mcp23s08.h

PREFIX     = /usr/local
LIBDIR     = $(PREFIX)/lib
INCLUDEDIR = $(PREFIX)/include

.PHONY: all install uninstall clean cleanall

all: $(TARGET)

$(BUILD_DIR):
	@mkdir -p "$(BUILD_DIR)"

$(OBJ): $(SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c "$<" -o "$@"

$(TARGET): $(OBJ)
	$(CC) $(LDFLAGS) -o "$@" "$<"

install: $(TARGET)
	@echo "Use 'sudo make install' if you need root permissions"
	@mkdir -p "$(DESTDIR)$(LIBDIR)" "$(DESTDIR)$(INCLUDEDIR)"
	install -m 0755 "$(TARGET)" "$(DESTDIR)$(LIBDIR)/"
	ln -sf "$(LIBBASE).so.$(MAJ).$(MIN)" "$(DESTDIR)$(LIBDIR)/$(SONAME)"
	ln -sf "$(SONAME)" "$(DESTDIR)$(LIBDIR)/$(DEVELINK)"
	ln -sf "$(DEVELINK)" "$(DESTDIR)$(LIBDIR)/$(COMPAT)"
	install -m 0644 "$(HEADER)" "$(DESTDIR)$(INCLUDEDIR)/lib_mcp23s08.h"
	-ldconfig || true
	@echo "Installed: $(LIBDIR)/$(LIBBASE).so.$(MAJ).$(MIN)"
	@echo "Symlinks : $(LIBDIR)/$(SONAME), $(LIBDIR)/$(DEVELINK), $(LIBDIR)/$(COMPAT)"

uninstall:
	@echo "Use 'sudo make uninstall' if you need root permissions"
	-rm -f "$(DESTDIR)$(LIBDIR)/$(DEVELINK)" \
	      "$(DESTDIR)$(LIBDIR)/$(SONAME)" \
	      "$(DESTDIR)$(LIBDIR)/$(LIBBASE).so.$(MAJ).$(MIN)" \
	      "$(DESTDIR)$(LIBDIR)/$(COMPAT)"
	-rm -f "$(DESTDIR)$(INCLUDEDIR)/lib_mcp23s08.h"
	-ldconfig || true
	@echo "Uninstalled."

clean:
	@rm -rf "$(BUILD_DIR)"
	@echo "Cleaned build files"

cleanall: clean uninstall
	@echo "Cleaned all installed files"