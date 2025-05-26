# QNX Compressed-EFS Filesystem Extractor

A tool to extract files from QNX compressed-EFS filesystem images. This tool supports the compressed variant of the QNX Embedded File System (EFS) that uses LZO and UCL compression methods commonly found in QNX embedded systems.

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

This approach ensures proper hierarchy preservation where.

## Program Architecture

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

### Program Flow

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

## Compilation

### Prerequisites
- GCC compiler
- liblzo2-dev (LZO compression library)
- libucl-dev (UCL compression library)

### Ubuntu/Debian
```bash
sudo apt-get install liblzo2-dev libucl-dev
```

### Build
```bash
cd efsdump
gcc -o efsdump efsdump.c -lm -llzo2 -lucl
```

## Usage

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

### Command Line Options

| Option | Long Option | Description |
|--------|-------------|-------------|
| `-d DIR` | `--directory DIR` | Extract to directory DIR (default: current directory) |
| `-l` | `--list-partitions` | List all partitions in the image |
| `-p NUM` | `--partition NUM` | Extract only partition NUM (0-based, default: all) |
| `-c` | `--list-contents` | List contents without extracting files |
| `-v` | `--verbose` | Enable verbose output |
| `-h` | `--help` | Show help message |

### Examples

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

## Output Formats

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

## Troubleshooting

### Common Issues

#### "No valid partitions found"
- The image may not be a QNX EFS filesystem
- The image may be corrupted or truncated
- Try examining with a hex editor to verify QSSL_F3S signature

#### "Failed to initialize LZO/UCL library"
- Ensure compression libraries are properly installed
- Check library versions are compatible

#### "Decompression error -203"
- This indicates UCL decompression failure
- The file may be corrupted or use a different compression variant
- Some files may use unsupported compression parameters

#### Files extracted to wrong directories
- This was a known issue that has been fixed in the current version
- Ensure you're using the latest version with proper directory-centric traversal

### Debug Information

Use verbose mode (`-v`) to get detailed information about:
- Which partitions are being processed
- Directory traversal path
- File compression details
- Extraction progress

## References

- [QNX Neutrino Documentation](https://www.qnx.com/developers/docs/)
- [Python QNX Mount Implementation](https://github.com/NetherlandsForensicInstitute/qnxmount) - Reference for EFS structures
- [LZO Compression Library](http://www.oberhumer.com/opensource/lzo/)
- [UCL Compression Library](http://www.oberhumer.com/opensource/ucl/)

## Files

- `efsdump.c` - Main source code
- `README.md` - This documentation
- `Makefile` - Build configuration

## Changelog

### Version 1.0 (2025-05-26)
- Initial release with full QNX Compressed-EFS support
- Directory-centric filesystem traversal implementation
- Support for LZO and UCL compression
- Command-line interface with multiple extraction modes
- Proper directory hierarchy preservation
- Verbose debugging and listing capabilities

## License

MIT License - See source code for details.
