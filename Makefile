
LIBUSB = 1
CFLAGS = -O2 -Wall -Wextra -std=c99 -pedantic
CFLAGS += -DUSE_LIBUSB=$(LIBUSB)
APPNAME = ch341tool

ifeq ($(LIBUSB), 1)
LIBS = -lusb-1.0
endif

.PHONY: all clean
all: $(APPNAME)

clean:
	$(RM) $(APPNAME)

$(APPNAME): $(APPNAME).c ch341a.h
	$(CC) -s $(CFLAGS) -o $@ $^ $(LIBS)
