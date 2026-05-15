CC = gcc
CFLAGS = -Wall -Wextra -O2
LIBS = -lm -llzo2 -lucl
IFS_LIBS = -lm -lz -llzo2 -lucl
TARGETS = efsdump ifsdump decompress-qnx-elf
EFS_SOURCE = efsdump.c
IFS_SOURCE = ifsdump.c
DECOMPRESS_SOURCE = decompress-qnx-elf.c

.PHONY: all clean install test lint

all: $(TARGETS)

efsdump: $(EFS_SOURCE)
	$(CC) $(CFLAGS) -o efsdump $(EFS_SOURCE) $(LIBS)

ifsdump: $(IFS_SOURCE)
	$(CC) $(CFLAGS) -o ifsdump $(IFS_SOURCE) $(IFS_LIBS)

decompress-qnx-elf: $(DECOMPRESS_SOURCE)
	$(CC) $(CFLAGS) -o decompress-qnx-elf $(DECOMPRESS_SOURCE) $(LIBS)

clean:
	rm -f $(TARGETS)

lint:
	$(CC) $(CFLAGS) -Werror -fsyntax-only $(EFS_SOURCE)
	$(CC) $(CFLAGS) -Werror -fsyntax-only $(IFS_SOURCE)
	$(CC) $(CFLAGS) -Werror -fsyntax-only $(DECOMPRESS_SOURCE)

install: $(TARGETS)
	install -D efsdump $(DESTDIR)/usr/local/bin/efsdump
	install -D ifsdump $(DESTDIR)/usr/local/bin/ifsdump
	install -D decompress-qnx-elf $(DESTDIR)/usr/local/bin/decompress-qnx-elf
	install -D -m 644 README.md $(DESTDIR)/usr/local/share/doc/dumpers/README.md

test: $(TARGETS)
	@echo "Testing efsdump help output..."
	./efsdump -h
	@echo "Testing ifsdump help output..."
	./ifsdump -h
	@echo "Testing decompress-qnx-elf help output..."
	./decompress-qnx-elf -h
	@echo "Testing on sample images (if available)..."
	@if [ -f ../SZ-AppR.bin ]; then \
		echo "Testing efsdump with SZ-AppR.bin:"; \
		./efsdump -l ../SZ-AppR.bin; \
		echo "Listing contents of partition 0:"; \
		./efsdump -c -p 0 ../SZ-AppR.bin | head -10; \
	else \
		echo "No EFS test image found at ../SZ-AppR.bin"; \
	fi
	@if [ -f ../SZ-OSR.bin ]; then \
		echo "Testing ifsdump with SZ-OSR.bin:"; \
		./ifsdump ../SZ-OSR.bin | head -10; \
	else \
		echo "No IFS test image found at ../SZ-OSR.bin"; \
	fi

# Development targets
debug: CFLAGS += -g -DDEBUG
debug: $(TARGETS)

# Check for required libraries
check-deps:
	@echo "Checking for required libraries..."
	@pkg-config --exists lzo2 || (echo "Error: liblzo2-dev not found. Install with: sudo apt-get install liblzo2-dev" && exit 1)
	@echo "LZO library: OK"
	@pkg-config --exists zlib || (echo "Error: zlib1g-dev not found. Install with: sudo apt-get install zlib1g-dev" && exit 1)
	@echo "zlib library: OK"
	@echo "UCL library: checking..."
	@echo '#include <ucl/ucl.h>' | $(CC) -x c - -lucl -o /dev/null 2>/dev/null || (echo "Error: libucl-dev not found. Install with: sudo apt-get install libucl-dev" && exit 1)
	@echo "UCL library: OK"
	@echo "All dependencies satisfied!"
