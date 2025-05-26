CC = gcc
CFLAGS = -Wall -Wextra -O2
LIBS = -lm -llzo2 -lucl
TARGET = efsdump
SOURCE = efsdump.c

.PHONY: all clean install test

all: $(TARGET)

$(TARGET): $(SOURCE)
	$(CC) $(CFLAGS) -o $(TARGET) $(SOURCE) $(LIBS)

clean:
	rm -f $(TARGET)

install: $(TARGET)
	install -D $(TARGET) $(DESTDIR)/usr/local/bin/$(TARGET)
	install -D -m 644 README.md $(DESTDIR)/usr/local/share/doc/$(TARGET)/README.md

test: $(TARGET)
	@echo "Testing help output..."
	./$(TARGET) -h
	@echo "Testing on sample image (if available)..."
	@if [ -f ../SZ-AppR.bin ]; then \
		echo "Listing partitions:"; \
		./$(TARGET) -l ../SZ-AppR.bin; \
		echo "Listing contents of partition 0:"; \
		./$(TARGET) -c -p 0 ../SZ-AppR.bin | head -10; \
	else \
		echo "No test image found at ../SZ-AppR.bin"; \
	fi

# Development targets
debug: CFLAGS += -g -DDEBUG
debug: $(TARGET)

# Check for required libraries
check-deps:
	@echo "Checking for required libraries..."
	@pkg-config --exists lzo2 || (echo "Error: liblzo2-dev not found. Install with: sudo apt-get install liblzo2-dev" && exit 1)
	@echo "LZO library: OK"
	@echo "UCL library: checking..."
	@echo '#include <ucl/ucl.h>' | $(CC) -x c - -lucl -o /dev/null 2>/dev/null || (echo "Error: libucl-dev not found. Install with: sudo apt-get install libucl-dev" && exit 1)
	@echo "UCL library: OK"
	@echo "All dependencies satisfied!"
