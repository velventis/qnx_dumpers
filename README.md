# QNX Filesystem Extraction Tools

A comprehensive collection of tools to extract files from QNX filesystem images. This suite includes extractors for both QNX Compressed-EFS and QNX IFS (Image File System) formats, supporting various compression methods (LZO, UCL, zlib) commonly found in QNX embedded systems.

## Tools Included

- **efsdump** - QNX Compressed-EFS filesystem extractor
- **ifsdump** - QNX IFS (Image File System) extractor

---

# QNX Compressed-EFS (efsdump)

## Overview

QNX Compressed-EFS is a specialized filesystem format used in QNX real-time operating systems, particularly in embedded devices. Unlike regular EFS, this variant compresses file data using either LZO or UCL compression algorithms to reduce storage requirements in resource-constrained environments.

## QNX Compressed-EFS Filesystem Structure

### High-Level Organization

The QNX Compressed-EFS filesystem is organized in a hierarchical structure:

```
Flash Image
├── Partition 0 (Boot partition)
│   ├── Boot Info (QSSL_F3S signature)
│   ├── Unit Info (alignment, unit size)
│   └── Units (logical blocks)
│       ├── Unit 1
│       ├── Unit 2
│       └── ...
└── Partition 1 (Application partition)
    ├── Boot Info
    ├── Unit Info
    └── Units
```

### Key Data Structures

#### Boot Info Structure (`boot_info_s`)
Located at the beginning of each partition, contains:
- `sig[8]`: Filesystem signature ("QSSL_F3S")
- `rev_major`, `rev_minor`: Version information
- `unit_total`: Total number of logical units
- `unit_spare`: Number of spare units
- `root`: Extent pointer to root directory

#### Unit Info Structure (`unit_info_s`)
Describes the layout and alignment of logical units:
- `unit_pow2`: Log2 of unit size (typically 16 = 64KB units)
- `endian`: Endianness indicator
- `boot`: Pointer to boot info structure

#### Extent Header Structure (`head_s`)
Each logical unit contains multiple extents with headers:
- `text_offset_hi`, `text_offset_lo`: 48-bit offset to file data
- `text_size`: Size of the file/directory data
- `next`: Pointer to next extent in chain
- `status[2]`: Status flags including extent type

#### Directory Entry Structure (`dirent_s`)
Directory entries contain metadata about files and subdirectories:
- `struct_size`: Size of the directory entry
- `namelen`: Length of the filename
- `stat`: File statistics (mode, uid, gid, timestamps)
- `first`: Pointer to first extent containing file data
- `name`: Variable-length filename

#### Extent Types
The filesystem uses different extent types:
- `EXTTYPE_DIR` (0): Directory entries
- `EXTTYPE_FILE` (1): File data extents
- `EXTTYPE_SYS` (2): System/metadata extents
- `EXTTYPE_XIP` (3): Execute-in-place extents

### File Compression

Files in the compressed-EFS use one of two compression methods:

#### Compression Detection
Files are identified as compressed by examining the first 8 bytes for the magic signature `"iwlyfmbp"`.

#### Compression Header Structure (`deflate_filehdr_t`)
- `sig[8]`: Magic signature "iwlyfmbp"
- `usize`: Uncompressed file size
- `cmptype`: Compression type (0=LZO, 1=UCL)
- `blksize`: Block size for decompression

#### Block Structure (`cmphdr_t`)
Compressed files are divided into blocks:
- `next`: Offset to next block (0 = last block)
- `usize`: Uncompressed size of this block

### Directory Hierarchy and Traversal Strategy

#### Directory-Centric Approach
The filesystem uses a directory-centric traversal model where:

1. **Root Directory**: The filesystem starts with a root directory entry pointed to by `boot_info.root`
2. **Directory Contents**: Each directory entry contains an extent chain (`first` pointer) that lists its contents
3. **Extent Chains**: Following the `next` pointers in extent headers traverses all items within a directory
4. **Recursive Structure**: Subdirectories have their own extent chains containing their contents

#### Traversal Algorithm
```
1. Start at root directory (from boot_info.root)
2. Read directory entry
3. Follow first extent pointer to get directory contents
4. For each extent in the chain (following next pointers):
   a. If extent type is DIR:
      - Parse as directory entry
      - If it's a subdirectory, recursively process its contents
      - If it's a file, extract the file data
   b. Move to next extent in chain
5. Continue until all extents processed
```

## efsdump Program Architecture

### Core Functions

#### Parsing Functions
- `read_partition()`: Reads partition boot info and unit structure
- `read_dirent()`: Reads directory entries with endianness conversion
- `read_stat()`: Reads file statistics
- `swap16()`, `swap32()`: Endianness conversion utilities

#### Traversal Functions
- `traverse_directory()`: Entry point for filesystem traversal
- `read_directory_contents()`: Reads contents of a directory using extent chains
- `find_file_extent()`: Locates file data extents

#### Extraction Functions
- `extract_file_data()`: Main file extraction dispatcher
- `extract_compressed_file_data()`: Handles LZO/UCL compressed files
- `extract_raw_file_data()`: Handles uncompressed files

#### Decompression Functions
- LZO decompression: `lzo1x_decompress_safe()`
- UCL decompression: `ucl_nrv2b_decompress_safe_8()`

### efsdump Program Flow

1. **Initialization**
   - Parse command line arguments
   - Initialize LZO and UCL libraries
   - Open filesystem image

2. **Partition Discovery**
   - Read all partitions from the image
   - Parse boot info and unit structures
   - Build logical unit mapping

3. **Directory Traversal**
   - Start from root directory of each partition
   - Use directory-centric approach to read contents
   - Follow extent chains to enumerate all files/directories

4. **File Extraction**
   - Detect compression by examining file headers
   - Decompress using appropriate algorithm (LZO/UCL)
   - Write extracted files to target directory

5. **Cleanup**
   - Free allocated memory
   - Close file handles

## efsdump Usage

### Basic Usage
```bash
# Extract all partitions to current directory
./efsdump filesystem.bin

# Extract to specific directory
./efsdump -d /tmp/extracted filesystem.bin

# List all partitions in image
./efsdump -l filesystem.bin

# Extract specific partition
./efsdump -p 0 filesystem.bin

# List contents without extracting
./efsdump -c filesystem.bin

# Verbose extraction
./efsdump -v filesystem.bin
```

### efsdump Command Line Options

| Option | Long Option | Description |
|--------|-------------|-------------|
| `-d DIR` | `--directory DIR` | Extract to directory DIR (default: current directory) |
| `-l` | `--list-partitions` | List all partitions in the image |
| `-p NUM` | `--partition NUM` | Extract only partition NUM (0-based, default: all) |
| `-c` | `--list-contents` | List contents without extracting files |
| `-v` | `--verbose` | Enable verbose output |
| `-h` | `--help` | Show help message |

### efsdump Examples

#### Explore an unknown image
```bash
# First, see what partitions are available
./efsdump -l unknown_image.bin

# List contents of first partition
./efsdump -p 0 -c unknown_image.bin

# Extract with verbose output to see what's happening
./efsdump -v -d extracted unknown_image.bin
```

#### Extract specific partition
```bash
# Extract only the application partition (usually partition 0)
./efsdump -p 0 -d app_files image.bin
```

#### Development/Debugging
```bash
# Verbose listing to understand filesystem structure
./efsdump -v -c image.bin

# Extract with full debugging output
./efsdump -v -d debug_output image.bin
```

## efsdump Output Formats

### List Partitions (`-l`)
```
Found 2 partition(s) in image.bin:
Partition 0:
  Boot Info: QSSL_F3S (Rev 3.0), 48 units
  Root: unit=1, index=3
Partition 1:
  Boot Info: QSSL_F3S (Rev 3.0), 29 units
  Root: unit=1, index=3
```

### List Contents (`-c`)
```
d ./bin
f ./bin/mkfilesystems.sh (1006 bytes)
f ./bin/puptool (14369 bytes)
d ./etc
f ./etc/passwd (373 bytes)
f ./etc/services (2048 bytes)
d ./etc/ntp
f ./etc/ntp/ntp.conf (256 bytes)
```

Format: `<type> <path> [(<size> bytes)]`
- `d`: directory
- `f`: file

### Verbose Output (`-v`)
Shows detailed information about:
- Partition boot info and unit structure
- Directory traversal process
- File extraction details
- Compression information
- Block-by-block decompression progress

---

# QNX IFS (Image File System) - ifsdump

## Overview

QNX IFS (Image File System) is the primary filesystem format used in QNX operating systems for creating bootable system images. Unlike EFS, IFS stores the entire filesystem as a single continuous image with an integrated directory structure, making it ideal for embedded systems that need fast boot times and efficient storage.

## QNX IFS Filesystem Structure

### High-Level Organization

The QNX IFS image is organized as a linear structure:

```
QNX IFS Image
├── Startup Header (optional)
│   ├── Signature (0x00ff7eeb)
│   ├── Boot configuration
│   ├── System parameters
│   └── Compression information
├── Compressed/Raw Image Data
│   ├── Startup code (if present)
│   └── Filesystem payload
└── Image Filesystem
    ├── Image Header ("imagefs" signature)
    ├── Directory Table
    ├── File Data Blocks
    └── Image Trailer (checksum)
```

### Key Data Structures

#### Startup Header Structure (`startup_header`)
Optional header found at the beginning of bootable images:
- `signature`: Magic number 0x00ff7eeb
- `version`: Header version information
- `flags1`, `flags2`: Control flags including compression and endianness
- `startup_size`: Size of startup code in bytes
- `stored_size`: Total size of stored image
- `imagefs_paddr`: Physical address of image filesystem
- `imagefs_size`: Size of image filesystem
- `paddr_bias`: Physical address bias for relocation

#### Image Header Structure (`image_header`)
Core filesystem header with "imagefs" signature:
- `signature[7]`: Magic signature "imagefs"
- `flags`: Filesystem flags (endianness, readonly, etc.)
- `image_size`: Total size of the image filesystem
- `hdr_dir_size`: Size of header + directory table
- `dir_offset`: Offset to directory table from header start
- `boot_ino[4]`: Inode numbers for boot files
- `script_ino`: Inode number for boot script
- `mountpoint`: Variable-length mount point string

#### Image Attribute Structure (`image_attr`)
Common attributes for all filesystem entries:
- `size`: Size of this directory entry structure
- `extattr_offset`: Offset to extended attributes
- `ino`: Inode number
- `mode`: File type and permissions (standard Unix mode bits)
- `gid`, `uid`: Group and user IDs
- `mtime`: Modification timestamp

#### File Entry Structure (`image_file`)
Regular file entries in the directory table:
- `attr`: Common attributes (image_attr)
- `offset`: Offset to file data from image start
- `size`: Size of file data in bytes
- `path`: Variable-length file path string

#### Directory Entry Structure (`image_dir`)
Directory entries in the directory table:
- `attr`: Common attributes (image_attr)
- `path`: Variable-length directory path string

#### Symlink Entry Structure (`image_symlink`)
Symbolic link entries:
- `attr`: Common attributes (image_attr)
- `sym_offset`: Offset within path string to symlink target
- `sym_size`: Size of symlink target string
- `path`: Variable-length path string containing both link path and target

#### Device Entry Structure (`image_device`)
Device file entries (character, block, FIFO, etc.):
- `attr`: Common attributes (image_attr)
- `dev`: Device number
- `rdev`: Raw device number for special files
- `path`: Variable-length device path string

### File Types and Mode Bits

IFS uses standard Unix file type indicators in the mode field:
- `S_IFREG` (0100000): Regular file
- `S_IFDIR` (0040000): Directory
- `S_IFLNK` (0120000): Symbolic link
- `S_IFCHR` (0020000): Character device
- `S_IFBLK` (0060000): Block device
- `S_IFIFO` (0010000): Named pipe (FIFO)
- `S_IFNAM` (0050000): Named special file

### Compression Support

IFS images can be compressed using multiple algorithms:

#### Compression Detection
Compression is indicated by flags in the startup header:
- `flags1 & STARTUP_HDR_FLAGS1_COMPRESS_MASK` determines compression type
- Supported types: None (0), zlib (4), LZO (8), UCL (12)

#### Compression Implementation
- **zlib**: Standard deflate compression using gzip-compatible format
- **LZO**: High-speed compression optimized for decompression performance
- **UCL**: Ultra-compact compression for minimal storage requirements

#### Decompression Process
1. Read startup header to determine compression type
2. Copy uncompressed startup code to temporary file
3. Decompress remaining image data using appropriate algorithm
4. Process the decompressed filesystem normally

### Directory Structure and Traversal

#### Linear Directory Table
Unlike EFS, IFS uses a linear directory table approach:

1. **Single Directory Table**: All entries stored sequentially in one table
2. **Variable-Length Entries**: Each entry has a size field for proper parsing
3. **Path-Based Organization**: Full paths stored with each entry
4. **Type-Based Processing**: Entry type determined by mode field

#### Traversal Algorithm
```
1. Locate image header using "imagefs" signature
2. Calculate directory table position (header + dir_offset)
3. Read directory entries sequentially:
   a. Read image_attr structure
   b. Read remaining entry data based on size field
   c. Process entry based on file type (mode & S_IFMT)
   d. Advance to next entry
4. Continue until all entries processed
```

## ifsdump Program Architecture

### Core Functions

#### Parsing Functions
- `find_pattern()`: Locates specific byte patterns in files
- `swap16()`, `swap32()`: Endianness conversion utilities
- `zero_ok()`: Validates zero fields in headers
- `check_name()`: Filters files based on command line patterns

#### Display Functions
- `display_shdr()`: Shows startup header information
- `display_ihdr()`: Shows image header information
- `display_attr()`: Shows file attribute details
- `display_file()`: Shows file entry information

#### Processing Functions
- `process_file()`: Handles regular file entries
- `process_dir()`: Handles directory entries
- `process_symlink()`: Handles symbolic link entries
- `process_device()`: Handles device file entries

#### Extraction Functions
- `extract_file()`: Extracts individual files to filesystem
- `create_directories()`: Creates directory structure recursively
- `copy_bytes()`: Efficiently copies file data

#### Compression Functions
- zlib decompression: `gzread()` using gzip streams
- LZO decompression: `lzo1x_decompress()` with block format
- UCL decompression: `ucl_nrv2b_decompress_8()` with block format

### ifsdump Program Flow

1. **Initialization**
   - Parse command line arguments and options
   - Open input image file or stdin
   - Initialize compression libraries if needed

2. **Header Discovery**
   - Search for startup header signature (0x00ff7eeb)
   - Validate zero fields and handle endianness
   - Display startup information if found

3. **Compression Handling**
   - Detect compression type from startup flags
   - Create temporary file for decompressed data
   - Decompress image using appropriate algorithm
   - Switch to processing decompressed image

4. **Image Processing**
   - Locate image header using "imagefs" signature
   - Parse image header and validate structure
   - Calculate directory table position

5. **Directory Traversal**
   - Read directory entries sequentially
   - Process each entry based on file type
   - Extract or display files as requested

6. **Cleanup and Verification**
   - Read and display checksums from image trailer
   - Validate startup trailer if present
   - Close files and free allocated memory

## ifsdump Usage

### Basic Usage
```bash
# List image contents (default behavior)
./ifsdump image.ifs

# Extract all files to current directory
./ifsdump -x image.ifs

# Extract to specific directory
./ifsdump -d /tmp/extracted -x image.ifs

# Extract specific file by name
./ifsdump -f filename image.ifs

# List contents only (explicit)
./ifsdump -l image.ifs

# Verbose extraction with debugging
./ifsdump -v -x image.ifs
```

### ifsdump Command Line Options

| Option | Long Option | Description |
|--------|-------------|-------------|
| `-d DIR` | `--directory DIR` | Extract to directory DIR (default: current directory) |
| `-f FILE` | `--file FILE` | Extract named file only |
| `-u FILE` | `--uncompress FILE` | Save uncompressed image to FILE |
| `-x` | `--extract` | Extract all files |
| `-l` | `--list` | List contents without extracting |
| `-b` | `--basename` | Extract to basenames of files |
| `-m` | `--md5` | Display MD5 checksums (placeholder) |
| `-z` | `--no-zero-check` | Disable zero field validation |
| `-v` | `--verbose` | Enable verbose output |
| `-h` | `--help` | Show help message |

### ifsdump Examples

#### Explore an unknown IFS image
```bash
# List all contents to understand structure
./ifsdump image.ifs

# Get detailed information with verbose output
./ifsdump -v image.ifs

# Extract everything to examine
./ifsdump -v -d extracted image.ifs
```

#### Extract specific files
```bash
# Extract a single file
./ifsdump -f bin/sh image.ifs

# Extract multiple specific files
./ifsdump -f libc.so -f ld.so image.ifs

# Extract all to directory with basenames only
./ifsdump -b -d flat_extract -x image.ifs
```

#### Work with compressed images
```bash
# Save decompressed image for faster repeated access
./ifsdump -u decompressed.ifs image.ifs.gz

# Process compressed image with verbose decompression info
./ifsdump -v -x compressed_image.ifs
```

#### Development and debugging
```bash
# Show detailed header and compression information
./ifsdump -v image.ifs

# Disable zero field checking for corrupted images
./ifsdump -z -v image.ifs

# Extract with full debugging output
./ifsdump -v -d debug_output -x image.ifs
```

## ifsdump Output Formats

### Default Listing
```
Image startup_size: 141624320 (0x8710400)
Image stored_size: 213133056 (0xcb42700)
Compressed size: 71508732 (0x44322fc)
     Offset       Size  Name
 0x00000000 0x00000001  *.boot
 0x00000001 0x00000100  Startup-header flags1=0xf flags2=0 paddr_bias=0
 0x00000101 0x00047008  startup.*
 0x00047109 0x00000059  Image-header mountpoint=/
 0x00047165 0x00000cf8  Image-directory
       ----       ----  Root-dirent
 0x0004785d 0x00001234  bin/sh
 0x00048a91 0x00004567  lib/libc.so
 0x0004cff8 0x00000123  etc/hosts
```

### Verbose Output (`-v`)
Shows detailed information about:
- Startup header fields and compression settings
- Image header details and mount information
- Directory entry attributes (permissions, ownership, timestamps)
- Decompression progress and block-by-block status
- File extraction details with full paths

### Checksums and Validation
```
Checksums: image=0x12345678 startup=0x87654321
```

---

# Compilation

## Prerequisites
- GCC compiler
- zlib1g-dev (zlib compression library) - **Required for ifsdump**
- liblzo2-dev (LZO compression library) - **Required for both tools**
- libucl-dev (UCL compression library) - **Required for both tools**

## Ubuntu/Debian Installation
```bash
sudo apt-get install build-essential zlib1g-dev liblzo2-dev libucl-dev
```

## Build
```bash
# Build both tools
make all

# Build individual tools
make efsdump
make ifsdump

# Build with debugging symbols
make debug

# Check for required dependencies
make check-deps

# Run test suite
make test
```

## Installation
```bash
# Install to system directories
sudo make install

# Install to custom prefix
make install DESTDIR=/usr/local
```

---

# Troubleshooting

## Common Issues

### "No valid partitions found" (efsdump)
- The image may not be a QNX EFS filesystem
- The image may be corrupted or truncated
- Try examining with a hex editor to verify QSSL_F3S signature

### "Unable to find startup header" (ifsdump)
- The image may not contain a QNX IFS with startup header
- Try searching for "imagefs" signature directly
- The image may be a raw IFS without startup wrapper

### "Failed to initialize LZO/UCL library"
- Ensure compression libraries are properly installed
- Check library versions are compatible
- Verify library packages include development headers

### "Decompression error -203" (UCL)
- This indicates UCL decompression failure
- The file may be corrupted or use a different compression variant
- Some files may use unsupported compression parameters

### "Compression detected but decompression failed"
- Check that all required compression libraries are installed
- Verify the compression type is supported (zlib, LZO, UCL)
- Try using `-z` flag to disable zero field validation

### Files extracted to wrong directories
- This was a known issue that has been fixed in the current version
- Ensure you're using the latest version with proper path handling

## Debug Information

Use verbose mode (`-v`) to get detailed information about:
- Header parsing and validation process
- Compression detection and decompression progress
- Directory structure and file attributes
- Extraction process and any errors encountered

---

# References

- [QNX Neutrino Documentation](https://www.qnx.com/developers/docs/)
- [QNX Image File System Documentation](https://www.qnx.com/developers/docs/7.0.0/index.html#com.qnx.doc.neutrino.utilities/topic/i/ifs.html)
- [Python QNX Mount Implementation](https://github.com/NetherlandsForensicInstitute/qnxmount) - Reference for EFS structures
- [zlib Library](https://zlib.net/)
- [LZO Compression Library](http://www.oberhumer.com/opensource/lzo/)
- [UCL Compression Library](http://www.oberhumer.com/opensource/ucl/)

---

# Files

- `efsdump.c` - QNX Compressed-EFS extractor source code
- `ifsdump.c` - QNX IFS extractor source code
- `README.md` - This documentation
- `Makefile` - Build configuration

---

# Changelog

### Version 2.0 (2025-06-11)
- Added comprehensive QNX IFS support with ifsdump tool
- Refactored ifsdump.c to match efsdump.c structure and style
- Added support for zlib, LZO, and UCL compression in IFS images
- Unified build system and documentation for both tools
- Added comprehensive test suite and dependency checking

### Version 1.0 (2025-05-26)
- Initial release with full QNX Compressed-EFS support
- Directory-centric filesystem traversal implementation
- Support for LZO and UCL compression
- Command-line interface with multiple extraction modes
- Proper directory hierarchy preservation
- Verbose debugging and listing capabilities

---

# License

MIT License - See source code for details.
