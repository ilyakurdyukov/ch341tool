
LIBUSB = 1
MFRC522v2 = 0
CFLAGS = -O2 -Wall -Wextra -pedantic
CFLAGS += -DUSE_LIBUSB=$(LIBUSB) -DARDUINO_MFRC522v2=$(MFRC522v2)
APPNAME = ch341tool
OBJDIR = obj
OBJS =

.PHONY: all clean
all: $(APPNAME)

CXXFLAGS := $(CFLAGS) -std=c++11 -fno-exceptions -fno-rtti
CFLAGS += -std=c99

ifeq ($(LIBUSB), 1)
LIBS = -lusb-1.0
endif

ifeq ($(MFRC522v2), 1)
MFRC522v2_INC = -IArduino -IArduino/Arduino_MFRC522v2/src
SRCS = Arduino MFRC522DriverCH341 \
	MFRC522v2 MFRC522Debug MFRC522Hack

OBJS += $(SRCS:%=$(OBJDIR)/%.o)

$(OBJDIR):
	mkdir -p $@

-include $(OBJS:.o=.d)

$(OBJDIR)/%.o: Arduino/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) $(MFRC522v2_INC) -MMD -MP -MF $(@:.o=.d) -c -o $@ $<

$(OBJDIR)/%.o: Arduino/Arduino_MFRC522v2/src/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) $(MFRC522v2_INC) -MMD -MP -MF $(@:.o=.d) -c -o $@ $<
endif

clean:
	$(RM) -r $(APPNAME) $(OBJDIR)

$(APPNAME): $(APPNAME).c $(OBJS) ch341a.h sdcard.h rc522.h
	$(CC) -s $(CFLAGS) $< $(OBJS) -o $@ $(LIBS)

