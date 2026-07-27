/* Extract QNX Compressed-EFS filesystems
 *
 * A tool to extract files from QNX compressed-EFS filesystem images.
 * Supports LZO and UCL compression methods used in QNX embedded systems.
 *
 * Based on research of QNX EFS filesystem structures
 *
 * Author: Jason Tang
 * License: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <getopt.h>
#include <lzo/minilzo.h>
#include <ucl/ucl.h>

// Global options
typedef struct {
    char *target_dir;
    int list_partitions;
    int partition_index;
    int list_only;
    int verbose;
    int help;
} options_t;

static options_t g_options = {
    .target_dir = ".",
    .list_partitions = 0,
    .partition_index = -1,  // -1 means all partitions
    .list_only = 0,
    .verbose = 0,
    .help = 0
};

/*
 * Error policy:
 * - Partition boot-info failures are fatal for that partition.
 * - Logical-unit read failures are non-fatal and treated as empty units; erased
 *   or trailing units can be unreadable in valid dumps.
 * - Directory-entry parse failures skip that entry.
 * - File extraction/decompression failures fail that file.
 */

#define F3S_NAME_ALIGN(LENGTH)\
\
  (((LENGTH)+(sizeof(uint32_t)-1))&~(sizeof(uint32_t)-1))

// File compression constants
#define CMP_LZO     0
#define CMP_UCL     1
#define MAX_PATH_LEN 4096
#define WORKBUF_SIZE (512*1024)
#define EFS_BOOTINFO_OFFSET 40
#define EFS_NULL_LOGI 0xffff
#define EFS_NULL_INDEX 0xffff
#define EFS_COMPRESSED_MAGIC "iwlyfmbp"
#define EFS_COMPRESSED_MAGIC_LEN 8

#pragma pack(push, 1)

typedef struct {
    uint16_t logi_unit;
    uint16_t index;
} extptr_s;

typedef struct {
    uint16_t struct_size;
    char endian;
    uint8_t pad;
    uint16_t unit_pow2;
    uint16_t reserve;
    uint32_t erase_count;
    extptr_s boot;
} unit_info_s;

typedef struct {
    uint16_t struct_size;
    uint16_t logi;
    uint32_t age;
    uint32_t md5[4];
} unit_logi_s;

typedef struct {
    uint16_t struct_size;
    uint8_t rev_major;
    uint8_t rev_minor;
    char sig[8];
    uint16_t unit_index;
    uint16_t unit_total;
    uint16_t unit_spare;
    uint16_t align_pow2;
    extptr_s root;
} boot_info_s;

typedef struct {
    uint16_t struct_size;
    uint16_t mode;
    uint32_t uid;
    uint32_t gid;
    uint32_t mtime;
    uint32_t ctime;
} stat_s;

typedef struct {
    uint16_t struct_size;
    uint8_t moves;
    uint8_t namelen;
    extptr_s first;
    char *name;  // Dynamic allocation for variable length
    stat_s stat;
} dirent_s;

typedef enum {
    EXTSTAT_BAD = 0x0,
    EXTSTAT_DELETE = 0x1,
    EXTSTAT_ALLOC = 0x3,
    EXTSTAT_FREE = 0x7
} block_status_e;

typedef enum {
    EXTTYPE_XIP = 0x0,
    EXTTYPE_SYS = 0x1,
    EXTTYPE_DIR = 0x2,
    EXTTYPE_FILE = 0x3
} extent_type_e;

typedef struct {
    uint16_t padding1;
    uint8_t flags2;  // Contains exttype(2), BASIC(1), padding(5)
    uint8_t flags1; // Contains remaining status bits
} status_b;

typedef struct {
    status_b status[3];
    uint16_t pad[3];
    uint8_t reserve;
    uint8_t text_offset_hi;
    uint16_t text_offset_lo;
    uint16_t text_size;
    extptr_s next;
    extptr_s super;
} head_s;

typedef struct {
    char sig[8];
    uint32_t size;
    uint16_t blk_size;
    uint8_t cmptype;
    uint8_t flags;
} cmp_head_s;

typedef struct {
    unit_info_s ui;
    unit_logi_s ul;
    head_s *heads;  // Dynamic array of headers
    size_t head_count;
    dirent_s *dirents;  // Parallel array to heads for directory entries
} blocks_s;

typedef enum {
    EFS_BYTE_ORDER_BIG,
    EFS_BYTE_ORDER_LITTLE
} efs_byte_order_t;

typedef struct {
    boot_info_s bi;
    blocks_s *blocks;  // Array size determined by bi.unit_total
    long efs_offset;   // Offset where this EFS partition starts in the file
    efs_byte_order_t byte_order;
    uint16_t unit_pow2;
} part_s;

// Compression structures for file extraction
typedef struct deflate_filehdr_s {
    uint8_t      signature[8];
    uint32_t     usize;      // Uncompressed size of the file
    uint16_t     blksize;    // Size of compression blocks
    uint8_t      cmptype;    // Type of compression (CMP_LZO, ...)
    uint8_t      flags;
} deflate_filehdr_t;

typedef struct cmphdr_s {
    uint16_t  prev;       // Offset to previous hdr
    uint16_t  next;       // Offset to next hdr
    uint16_t  pusize;     // Size of prev uncompressed blk
    uint16_t  usize;      // Size of this uncompressed blk
} cmphdr_t;

// Directory path tracking
typedef struct {
    char path[MAX_PATH_LEN];
    size_t len;
} dir_state_t;

// File extent information for multi-extent files
typedef struct {
    long offset;      // Absolute file offset to data
    uint16_t size;    // Size of data at this extent
    uint16_t logi_unit; // Logical unit containing this extent
    uint16_t index;   // Index within the unit
} file_extent_t;

#pragma pack(pop)

typedef struct {
    FILE *fp;
    const file_extent_t *extents;
    int extent_count;
    int extent_index;
    size_t extent_offset;
    long saved_pos;
} extent_reader_t;

typedef struct {
    uint8_t *visited;
    size_t visited_count;
    unsigned depth;
} dir_walk_state_t;

// Forward declarations
void extract_filesystem(FILE *fp, part_s *partitions, size_t count);
void extract_partition(FILE *fp, part_s *part);
void extract_directory_entry(FILE *fp, part_s *part, dir_state_t *dir_state, dirent_s *dirent);
size_t read_partition(FILE *fp, part_s *part);
void free_partition(part_s *part);
head_s* find_file_extent(part_s *part, uint16_t logi_unit, uint16_t index);
void traverse_directory(FILE *fp, part_s *part, uint16_t logi_unit, uint16_t index);
int collect_file_extents(part_s *part, uint16_t logi_unit, uint16_t index, file_extent_t **extents, int *extent_count);
int extract_file_data(FILE *fp, part_s *part, uint16_t logi_unit, uint16_t index, const char *output_path);
int extract_compressed_file_data(extent_reader_t *reader, const char *output_path);
int extent_reader_init(extent_reader_t *reader, FILE *fp, const file_extent_t *extents, int extent_count);
void extent_reader_reset(extent_reader_t *reader);
void extent_reader_close(extent_reader_t *reader);
size_t extent_reader_read(extent_reader_t *reader, void *buffer, size_t size);
size_t extent_global_index(part_s *part, uint16_t logi_unit, uint16_t index);
int read_directory_contents(FILE *fp, part_s *part, dirent_s *dir_entry, const char *dir_path, dir_walk_state_t *state);
void init_dir_state(dir_state_t* state);
void push_dir(dir_state_t* state, const char* dirname);
void pop_dir(dir_state_t* state);
int create_directories(const char* path);

// CKSUM header handling functions
int is_cksum_file(FILE *fp);
long find_first_efs_partition(FILE *fp);
long find_next_efs_partition(FILE *fp, long start_offset);

// Print function declarations
void print_unit_info(const char* prefix, unit_info_s *ui);
void print_unit_logi(const char* prefix, unit_logi_s *ul);
void print_boot_info(const char* prefix, boot_info_s *bi);
void print_head(const char* prefix, head_s *hd);
void print_dirent(const char* prefix, dirent_s *dir);
void print_stat(const char* prefix, stat_s *st);
void print_extptr(const char* prefix, const char* label, extptr_s *ptr);
void print_status(const char* prefix, status_b *status);

uint16_t read_le16(uint16_t val) {
    uint8_t *bytes = (uint8_t *)&val;
    return (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
}

uint32_t read_le32(uint32_t val) {
    uint8_t *bytes = (uint8_t *)&val;
    return (uint32_t)bytes[0] |
           ((uint32_t)bytes[1] << 8) |
           ((uint32_t)bytes[2] << 16) |
           ((uint32_t)bytes[3] << 24);
}

uint16_t read_be16(uint16_t val) {
    uint8_t *bytes = (uint8_t *)&val;
    return ((uint16_t)bytes[0] << 8) | (uint16_t)bytes[1];
}

uint32_t read_be32(uint32_t val) {
    uint8_t *bytes = (uint8_t *)&val;
    return ((uint32_t)bytes[0] << 24) |
           ((uint32_t)bytes[1] << 16) |
           ((uint32_t)bytes[2] << 8) |
           (uint32_t)bytes[3];
}

uint16_t read_efs16(uint16_t val, efs_byte_order_t byte_order) {
    return byte_order == EFS_BYTE_ORDER_LITTLE ? read_le16(val) : read_be16(val);
}

uint32_t read_efs32(uint32_t val, efs_byte_order_t byte_order) {
    return byte_order == EFS_BYTE_ORDER_LITTLE ? read_le32(val) : read_be32(val);
}

// Directory management functions
void init_dir_state(dir_state_t* state) {
    strcpy(state->path, ".");
    state->len = 1;
}

void push_dir(dir_state_t* state, const char* dirname) {
    if (state->len + strlen(dirname) + 2 >= MAX_PATH_LEN) return;

    if (state->len > 1) {
        strcat(state->path, "/");
        state->len++;
    }

    strcat(state->path, dirname);
    state->len += strlen(dirname);
}

void pop_dir(dir_state_t* state) {
    char* last_slash = strrchr(state->path, '/');
    if (last_slash == NULL || last_slash == state->path) {
        strcpy(state->path, ".");
        state->len = 1;
    } else {
        *last_slash = '\0';
        state->len = last_slash - state->path;
    }
}

// Create directory recursively
int create_directories(const char* path) {
    char dir_path[MAX_PATH_LEN];
    strncpy(dir_path, path, sizeof(dir_path) - 1);
    dir_path[sizeof(dir_path) - 1] = '\0';

    char* last_slash = strrchr(dir_path, '/');
    if (last_slash) {
        *last_slash = '\0';

        // Create parent directories first
        char* p = dir_path;
        while (*p) {
            if (*p == '/') {
                *p = '\0';
                if (strlen(dir_path) > 0 && mkdir(dir_path, 0755) != 0 && errno != EEXIST) {
                    fprintf(stderr, "Failed to create directory %s: %s\n", dir_path, strerror(errno));
                    return -1;
                }
                *p = '/';
            }
            p++;
        }
        if (strlen(dir_path) > 0 && mkdir(dir_path, 0755) != 0 && errno != EEXIST) {
            fprintf(stderr, "Failed to create directory %s: %s\n", dir_path, strerror(errno));
            return -1;
        }
    }
    return 0;
}

// CKSUM header handling functions
int is_cksum_file(FILE *fp) {
    char header[16];
    long current_pos = ftell(fp);

    // Read the first 16 bytes to check for CKSUM signature
    fseek(fp, 0, SEEK_SET);
    if (fread(header, 1, 16, fp) != 16) {
        fseek(fp, current_pos, SEEK_SET);
        return 0;
    }

    // Restore file position
    fseek(fp, current_pos, SEEK_SET);

    // Check for "CHKSUM:" signature (with or without space)
    return (memcmp(header, "CHKSUM:", 7) == 0);
}

long find_first_efs_partition(FILE *fp) {
    char buffer[4096];
    long current_pos = ftell(fp);

    if (g_options.verbose) {
        printf("Scanning for EFS partitions in CKSUM file...\n");
    }

    // Get file size to know when to stop scanning
    fseek(fp, 0, SEEK_END);
    long file_size_tmp = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    if (file_size_tmp < 0) {
        return -1;
    }
    size_t file_size = (size_t)file_size_tmp;

    // Search for QSSL_F3S signature throughout the file
    size_t offset = 0;
    while (offset < file_size - 8) {
        size_t bytes_to_read = (offset + sizeof(buffer) > file_size) ?
                              (file_size - offset) : sizeof(buffer);

        if (fseek(fp, offset, SEEK_SET) != 0) {
            break;
        }

        size_t bytes_read = fread(buffer, 1, bytes_to_read, fp);
        if (bytes_read < 8) {
            break;
        }

        // Search for QSSL_F3S in this buffer
        for (size_t i = 0; i <= bytes_read - 8; i++) {
            if (memcmp(buffer + i, "QSSL_F3S", 8) == 0) {
                size_t signature_offset = offset + i;
                // EFS partition starts 0x2C bytes before the QSSL_F3S signature
                long partition_start = (signature_offset >= 0x2C) ? (long)(signature_offset - 0x2C) : -1;

                if (partition_start < 0) {
                    continue;
                }

                if (g_options.verbose) {
                    printf("Found EFS partition at offset 0x%lx (signature at 0x%zx)\n",
                           partition_start, signature_offset);
                }

                fseek(fp, current_pos, SEEK_SET);
                return partition_start;
            }
        }

        // Move forward, but overlap by 7 bytes to ensure we don't miss signatures across buffer boundaries
        offset += bytes_read - 7;
    }

    if (g_options.verbose) {
        printf("No EFS partition found in file\n");
    }

    fseek(fp, current_pos, SEEK_SET);
    return -1;  // No EFS partition found
}

long find_next_efs_partition(FILE *fp, long start_offset) {
    char buffer[4096];
    long current_pos = ftell(fp);

    if (g_options.verbose) {
        printf("Scanning for next EFS partition starting from offset 0x%lx...\n", start_offset);
    }

    // Get file size to know when to stop scanning
    fseek(fp, 0, SEEK_END);
    long file_size_tmp = ftell(fp);

    if (file_size_tmp < 0) {
        return -1;
    }
    size_t file_size = (size_t)file_size_tmp;

    // Search for QSSL_F3S signature starting from the given offset
    size_t offset = (size_t)start_offset;
    while (offset < file_size - 8) {
        size_t bytes_to_read = (offset + sizeof(buffer) > file_size) ?
                              (file_size - offset) : sizeof(buffer);

        if (fseek(fp, offset, SEEK_SET) != 0) {
            break;
        }

        size_t bytes_read = fread(buffer, 1, bytes_to_read, fp);
        if (bytes_read < 8) {
            break;
        }

        // Search for QSSL_F3S in this buffer
        for (size_t i = 0; i <= bytes_read - 8; i++) {
            if (memcmp(buffer + i, "QSSL_F3S", 8) == 0) {
                size_t signature_offset = offset + i;
                // EFS partition starts 0x2C bytes before the QSSL_F3S signature
                long partition_start = (signature_offset >= 0x2C) ? (long)(signature_offset - 0x2C) : -1;

                if (partition_start < 0) {
                    continue;
                }

                if (g_options.verbose) {
                    printf("Found next EFS partition at offset 0x%lx (signature at 0x%zx)\n",
                           partition_start, signature_offset);
                }

                fseek(fp, current_pos, SEEK_SET);
                return partition_start;
            }
        }

        // Move forward, but overlap by 7 bytes to ensure we don't miss signatures across buffer boundaries
        offset += bytes_read - 7;
    }

    if (g_options.verbose) {
        printf("No additional EFS partitions found\n");
    }

    fseek(fp, current_pos, SEEK_SET);
    return -1;  // No additional EFS partition found
}

// Find a file extent by logical unit and index
head_s* find_file_extent(part_s *part, uint16_t logi_unit, uint16_t index) {
    // Check if logical unit is valid
    if (logi_unit == 0 || logi_unit == EFS_NULL_LOGI || logi_unit > part->bi.unit_total) {
        return NULL;
    }

    // Get the blocks for this logical unit (logi_unit is 1-based)
    blocks_s *blocks = &part->blocks[logi_unit - 1];

    // Check if index is valid
    if (index >= blocks->head_count) {
        return NULL;
    }

    head_s *head = &blocks->heads[index];
    extent_type_e exttype = (head->status[0].flags2) & 0x03;

    // Only return file extents
    if (exttype == EXTTYPE_FILE) {
        return head;
    }

    return NULL;
}

int extent_reader_init(extent_reader_t *reader, FILE *fp, const file_extent_t *extents, int extent_count) {
    if (!reader || !fp || !extents || extent_count <= 0) {
        return -1;
    }

    reader->fp = fp;
    reader->extents = extents;
    reader->extent_count = extent_count;
    reader->extent_index = 0;
    reader->extent_offset = 0;
    reader->saved_pos = ftell(fp);
    return reader->saved_pos < 0 ? -1 : 0;
}

void extent_reader_reset(extent_reader_t *reader) {
    reader->extent_index = 0;
    reader->extent_offset = 0;
}

void extent_reader_close(extent_reader_t *reader) {
    if (reader && reader->fp && reader->saved_pos >= 0) {
        fseek(reader->fp, reader->saved_pos, SEEK_SET);
    }
}

size_t extent_reader_read(extent_reader_t *reader, void *buffer, size_t size) {
    uint8_t *out = buffer;
    size_t total_read = 0;

    while (total_read < size && reader->extent_index < reader->extent_count) {
        const file_extent_t *extent = &reader->extents[reader->extent_index];

        if (reader->extent_offset >= extent->size) {
            reader->extent_index++;
            reader->extent_offset = 0;
            continue;
        }

        size_t available = extent->size - reader->extent_offset;
        size_t to_read = size - total_read;
        if (to_read > available) {
            to_read = available;
        }

        if (fseek(reader->fp, extent->offset + (long)reader->extent_offset, SEEK_SET) != 0) {
            break;
        }

        size_t bytes_read = fread(out + total_read, 1, to_read, reader->fp);
        total_read += bytes_read;
        reader->extent_offset += bytes_read;

        if (bytes_read != to_read) {
            break;
        }
    }

    return total_read;
}

size_t extent_global_index(part_s *part, uint16_t logi_unit, uint16_t index) {
    size_t slot = 0;

    if (logi_unit == 0 || logi_unit == EFS_NULL_LOGI || logi_unit > part->bi.unit_total) {
        return (size_t)-1;
    }

    for (uint16_t i = 1; i < logi_unit; i++) {
        slot += part->blocks[i - 1].head_count;
    }

    if (index >= part->blocks[logi_unit - 1].head_count) {
        return (size_t)-1;
    }

    return slot + index;
}

int mark_visited_extent(part_s *part, dir_walk_state_t *state, uint16_t logi_unit, uint16_t index, const char *context) {
    size_t slot = extent_global_index(part, logi_unit, index);

    if (!state || !state->visited || slot == (size_t)-1 || slot >= state->visited_count) {
        return 0;
    }

    if (state->visited[slot]) {
        if (g_options.verbose) {
            printf("      Warning: detected directory cycle at unit=%u, index=%u (%s)\n",
                   logi_unit, index, context ? context : "unknown");
        }
        return -1;
    }

    state->visited[slot] = 1;
    return 0;
}

// Collect all extents for a file by following the extent chain
int collect_file_extents(part_s *part, uint16_t logi_unit, uint16_t index, file_extent_t **extents, int *extent_count) {
    *extents = NULL;
    *extent_count = 0;
    int extent_capacity = 0;

    uint16_t current_logi = logi_unit;
    uint16_t current_index = index;
    size_t max_extents = 0;
    for (int i = 0; i < part->bi.unit_total; i++) {
        max_extents += part->blocks[i].head_count;
    }
    uint8_t *visited = max_extents > 0 ? calloc(max_extents, 1) : NULL;

    // Follow the extent chain
    while (current_logi != 0 && current_logi != EFS_NULL_LOGI) {
        if (current_logi > part->bi.unit_total) {
            break;
        }

        blocks_s *blocks = &part->blocks[current_logi - 1];
        if (current_index >= blocks->head_count) {
            break;
        }

        head_s *head = &blocks->heads[current_index];
        extent_type_e exttype = (head->status[0].flags2) & 0x03;

        if (exttype != EXTTYPE_FILE) {
            break;  // Not a file extent
        }

        size_t visited_slot = extent_global_index(part, current_logi, current_index);
        if (visited && visited_slot != (size_t)-1 && visited_slot < max_extents) {
            if (visited[visited_slot]) {
                if (g_options.verbose) {
                    printf("      Warning: detected extent chain cycle at unit=%u, index=%u\n",
                           current_logi, current_index);
                }
                free(visited);
                return *extent_count;
            }
            visited[visited_slot] = 1;
        }

        // Calculate the actual data offset within the block
        // Each block is unit_pow2 size, data comes before the head_s tables
        long block_base = part->efs_offset + (current_logi - 1) * (1LL << part->unit_pow2);
        long data_offset = block_base + (long)(head->text_offset_hi) * 0x40000 + (long)(head->text_offset_lo) * 4;

        // Reallocate extent array
        if (*extent_count == extent_capacity) {
            int new_capacity = extent_capacity == 0 ? 8 : extent_capacity * 2;
            file_extent_t *new_extents = realloc(*extents, new_capacity * sizeof(file_extent_t));
            if (!new_extents) {
                free(visited);
                free(*extents);
                *extents = NULL;
                *extent_count = 0;
                return -1;
            }
            *extents = new_extents;
            extent_capacity = new_capacity;
        }

        // Store extent information
        (*extents)[*extent_count].offset = data_offset;
        (*extents)[*extent_count].size = head->text_size;
        (*extents)[*extent_count].logi_unit = current_logi;
        (*extents)[*extent_count].index = current_index;
        (*extent_count)++;

        if (g_options.verbose) {
            printf("      Extent %d: unit=%u, index=%u, offset=0x%lx, size=%u\n",
                   *extent_count, current_logi, current_index, data_offset, head->text_size);
        }

        // Move to next extent
        if (head->next.logi_unit == 0 || head->next.logi_unit == EFS_NULL_LOGI) {
            break;
        }
        current_logi = head->next.logi_unit;
        current_index = head->next.index;

        // Safety check to prevent infinite loops on malformed chains
        if (max_extents > 0 && (size_t)*extent_count > max_extents) {
            if (g_options.verbose) {
                printf("      Warning: Hit safety limit of %zu extents\n", max_extents);
            }
            break;
        }
    }

    free(visited);
    return *extent_count;
}

// Traverse directory structure by following the extent chain
// Read directory contents by following extent chain of a directory entry
int read_directory_contents(FILE *fp, part_s *part, dirent_s *dir_entry, const char *dir_path, dir_walk_state_t *state) {
    if (dir_entry->first.logi_unit == 0 || dir_entry->first.logi_unit == EFS_NULL_LOGI) {
        return 0;  // No contents
    }

    if (g_options.verbose) {
        printf("    Reading contents of directory: %s\n", dir_path);
    }

    // Follow the extent chain to get all directory contents
    uint16_t logi_unit = dir_entry->first.logi_unit;
    uint16_t index = dir_entry->first.index;

    while (logi_unit != 0 && logi_unit != EFS_NULL_LOGI) {
        if (logi_unit > part->bi.unit_total) {
            break;
        }
        if (mark_visited_extent(part, state, logi_unit, index, dir_path) != 0) {
            return -1;
        }

        blocks_s *blocks = &part->blocks[logi_unit - 1];
        if (index >= blocks->head_count) {
            break;
        }

        head_s *head = &blocks->heads[index];
        extent_type_e exttype = (head->status[0].flags2) & 0x03;

        if (exttype == EXTTYPE_DIR) {
            dirent_s *content_entry = &blocks->dirents[index];
            if (content_entry->name == NULL) {
                if (g_options.verbose) {
                    printf("      Skipping invalid directory entry at unit=%u, index=%u\n", logi_unit, index);
                }
                goto next_extent;
            }

            if (g_options.verbose) {
                printf("      Content entry: %s (mode=0x%x)\n", content_entry->name, content_entry->stat.mode);
            }

            if ((content_entry->stat.mode & 0xf000) == 0x4000) {  // Directory
                // Skip special directories
                if (strcmp(content_entry->name, ".") == 0 || strcmp(content_entry->name, "..") == 0 || strcmp(content_entry->name, "/") == 0) {
                    // Skip these
                } else {
                    // This is a subdirectory
                    const char *clean_name = content_entry->name;
                    if (clean_name[0] == '.') {
                        clean_name++;
                    } else if (clean_name[0] == '/') {
                        clean_name++;
                    }

                    char subdir_path[MAX_PATH_LEN];
                    snprintf(subdir_path, sizeof(subdir_path), "%s/%s", dir_path, clean_name);

                    if (g_options.list_only) {
                        printf("d %s\n", subdir_path);
                    } else {
                        if (mkdir(subdir_path, 0755) != 0 && errno != EEXIST) {
                            fprintf(stderr, "Failed to create directory %s: %s\n", subdir_path, strerror(errno));
                        } else if (g_options.verbose) {
                            printf("        Created subdirectory: %s\n", subdir_path);
                        }
                    }

                    // Recursively read this subdirectory's contents
                    state->depth++;
                    if (state->depth > state->visited_count ||
                        read_directory_contents(fp, part, content_entry, subdir_path, state) != 0) {
                        state->depth--;
                        return -1;
                    }
                    state->depth--;
                }
            } else if ((content_entry->stat.mode & 0xf000) == 0x8000) {  // Regular file
                // This is a file in the current directory
                char filepath[MAX_PATH_LEN];
                int ret = snprintf(filepath, sizeof(filepath), "%s/%s", dir_path, content_entry->name);
                if (ret >= (int)sizeof(filepath)) {
                    fprintf(stderr, "Warning: Path too long, truncated: %s/%s\n", dir_path, content_entry->name);
                }

                if (g_options.list_only) {
                    // Just list the file
                    head_s *file_head = find_file_extent(part, content_entry->first.logi_unit, content_entry->first.index);
                    if (file_head && file_head->text_size > 0) {
                        printf("f %s (%u bytes)\n", filepath, file_head->text_size);
                    } else {
                        printf("f %s (empty)\n", filepath);
                    }
                } else {
                    // Extract the file
                    if (g_options.verbose) {
                        printf("        File: %s -> extent unit=%u, index=%u\n",
                               filepath, content_entry->first.logi_unit, content_entry->first.index);
                    }

                    if (extract_file_data(fp, part, content_entry->first.logi_unit, content_entry->first.index, filepath) == 0) {
                        if (g_options.verbose) {
                            printf("          Successfully extracted: %s\n", filepath);
                        }
                    } else {
                        printf("          Failed to extract: %s\n", filepath);
                    }
                }
            }
        }

        // Move to next extent in the chain
next_extent:
        if (head->next.logi_unit == 0 || head->next.logi_unit == EFS_NULL_LOGI) {
            break;
        }
        logi_unit = head->next.logi_unit;
        index = head->next.index;
    }

    return 0;
}

void traverse_directory(FILE *fp, part_s *part, uint16_t logi_unit, uint16_t index) {
    if (logi_unit == 0 || logi_unit > part->bi.unit_total || logi_unit == EFS_NULL_LOGI) {
        return;
    }

    // Get the blocks for this logical unit
    blocks_s *blocks = &part->blocks[logi_unit - 1];

    if (index >= blocks->head_count) {
        return;
    }

    head_s *head = &blocks->heads[index];
    extent_type_e exttype = (head->status[0].flags2) & 0x03;

    if (g_options.verbose) {
        printf("    Traversing: unit=%u, index=%u, type=%s\n",
               logi_unit, index,
               exttype == EXTTYPE_DIR ? "DIR" :
               exttype == EXTTYPE_FILE ? "FILE" :
               exttype == EXTTYPE_SYS ? "SYS" : "XIP");
    }

    if (exttype == EXTTYPE_DIR) {
        // This is a directory entry - read its contents
        dirent_s *dirent = &blocks->dirents[index];
        if (dirent->name == NULL) {
            if (g_options.verbose) {
                printf("      Skipping invalid directory entry at unit=%u, index=%u\n", logi_unit, index);
            }
            return;
        }

        if (g_options.verbose) {
            printf("      Directory entry: %s\n", dirent->name);
        }

        size_t total_extents = 0;
        for (int i = 0; i < part->bi.unit_total; i++) {
            total_extents += part->blocks[i].head_count;
        }
        dir_walk_state_t state = {
            .visited = total_extents > 0 ? calloc(total_extents, 1) : NULL,
            .visited_count = total_extents,
            .depth = 0
        };

        // Process any directory entry, not just root
        const char *dir_path = (strcmp(dirent->name, "/") == 0) ? "." : dirent->name + 1; // Skip leading '/'
        if (read_directory_contents(fp, part, dirent, dir_path, &state) != 0 && g_options.verbose) {
            printf("      Warning: stopped directory traversal at unit=%u, index=%u\n", logi_unit, index);
        }
        free(state.visited);
    }
}

// File extraction function - properly follows extent chains and respects block structure
int extract_file_data(FILE *fp, part_s *part, uint16_t logi_unit, uint16_t index, const char *output_path) {
    if (g_options.verbose) {
        printf("      Extracting file: %s from unit=%u, index=%u\n", output_path, logi_unit, index);
    }

    // Collect all extents for this file
    file_extent_t *extents = NULL;
    int extent_count = 0;

    if (collect_file_extents(part, logi_unit, index, &extents, &extent_count) < 0) {
        fprintf(stderr, "Failed to collect extents for %s\n", output_path);
        return -1;
    }

    if (extent_count == 0) {
        // Empty file
        create_directories(output_path);
        int fd = open(output_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd >= 0) {
            close(fd);
            if (g_options.verbose) {
                printf("        Created empty file: %s\n", output_path);
            }
            return 0;
        } else {
            fprintf(stderr, "Failed to create empty file %s: %s\n", output_path, strerror(errno));
            return -1;
        }
    }

    extent_reader_t reader;
    if (extent_reader_init(&reader, fp, extents, extent_count) != 0) {
        fprintf(stderr, "Failed to initialize extent reader for %s\n", output_path);
        free(extents);
        return -1;
    }

    uint8_t magic[EFS_COMPRESSED_MAGIC_LEN];
    size_t magic_read = extent_reader_read(&reader, magic, sizeof(magic));
    extent_reader_reset(&reader);

    int result = 0;
    if (magic_read == sizeof(magic) && memcmp(magic, EFS_COMPRESSED_MAGIC, sizeof(magic)) == 0) {
        if (g_options.verbose) {
            printf("        Found compression signature in %s\n", output_path);
        }
        result = extract_compressed_file_data(&reader, output_path);
    } else {
        uint8_t buffer[16384];
        size_t total_written = 0;

        if (g_options.verbose) {
            printf("        Extracting raw file: %s\n", output_path);
        }

        create_directories(output_path);
        int fd = open(output_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd < 0) {
            fprintf(stderr, "Failed to create file %s: %s\n", output_path, strerror(errno));
            result = -1;
        } else {
            size_t bytes_read;
            while ((bytes_read = extent_reader_read(&reader, buffer, sizeof(buffer))) > 0) {
                ssize_t written = write(fd, buffer, bytes_read);
                if (written < 0 || (size_t)written != bytes_read) {
                    fprintf(stderr, "Failed to write data to %s: %s\n", output_path, strerror(errno));
                    result = -1;
                    break;
                }
                total_written += bytes_read;
            }
            close(fd);

            if (g_options.verbose && result == 0) {
                printf("        Successfully extracted %zu bytes to %s\n", total_written, output_path);
            }
        }
    }

    extent_reader_close(&reader);
    free(extents);
    return result;
}

int extract_compressed_file_data(extent_reader_t *reader, const char *output_path) {
    // Compressed file headers and block headers are little-endian.
    deflate_filehdr_t defhdr;
    if (extent_reader_read(reader, &defhdr, sizeof(deflate_filehdr_t)) != sizeof(deflate_filehdr_t)) {
        fprintf(stderr, "Failed to read compression header for %s\n", output_path);
        return -1;
    }
    defhdr.usize = read_le32(defhdr.usize);
    defhdr.blksize = read_le16(defhdr.blksize);

    if (memcmp(defhdr.signature, EFS_COMPRESSED_MAGIC, EFS_COMPRESSED_MAGIC_LEN) != 0 ||
        (defhdr.cmptype != CMP_LZO && defhdr.cmptype != CMP_UCL) ||
        defhdr.blksize < 16) {
        fprintf(stderr, "Invalid compression header for %s (size=%u, blksize=%u, compression=%u)\n",
                output_path, defhdr.usize, defhdr.blksize, defhdr.cmptype);
        return -1;
    }

    if (g_options.verbose) {
        printf("Extracting compressed file: %s (size: %u, compression: %s, blksize: %u)\n",
               output_path, defhdr.usize, defhdr.cmptype == CMP_LZO ? "LZO" : "UCL", defhdr.blksize);
    }

    // Create output file
    create_directories(output_path);
    int outfd = open(output_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (outfd < 0) {
        fprintf(stderr, "Failed to create file %s: %s\n", output_path, strerror(errno));
        return -1;
    }


    // Allocate work buffer
    size_t overhead = defhdr.blksize / 8 + 256;
    size_t buf_len = defhdr.blksize + overhead;
    unsigned char* workbuf = malloc(buf_len * 2);
    if (!workbuf) {
        fprintf(stderr, "Failed to allocate work buffer for %s\n", output_path);
        close(outfd);
        return -1;
    }

    // Decompress blocks
    size_t total_written = 0;
    int result = 0;
    for (int block_num = 0; total_written < defhdr.usize; block_num++) {
        cmphdr_t blockhdr;
        if (extent_reader_read(reader, &blockhdr, sizeof(cmphdr_t)) != sizeof(cmphdr_t)) {
            fprintf(stderr, "Failed to read block header %d for %s\n", block_num, output_path);
            result = -1;
            break;
        }
        blockhdr.prev = read_le16(blockhdr.prev);
        blockhdr.next = read_le16(blockhdr.next);
        blockhdr.pusize = read_le16(blockhdr.pusize);
        blockhdr.usize = read_le16(blockhdr.usize);

        if (blockhdr.next == 0) break;
        if (blockhdr.next < sizeof(cmphdr_t)) {
            fprintf(stderr, "Invalid compressed data block %d for %s (next=%u)\n",
                    block_num, output_path, blockhdr.next);
            result = -1;
            break;
        }

        uint16_t in_len = blockhdr.next - sizeof(cmphdr_t);
        uint16_t expected_out_len = blockhdr.usize;  // Use block header's usize
        if (in_len > buf_len || expected_out_len > defhdr.blksize) {
            fprintf(stderr, "Invalid compressed data block %d for %s (in_len=%u, usize=%u)\n",
                    block_num, output_path, in_len, expected_out_len);
            result = -1;
            break;
        }

        if (g_options.verbose) {
            printf("  Block %d: next=%u, usize=%u, in_len=%u, expected_out=%u\n",
                   block_num, blockhdr.next, blockhdr.usize, in_len, expected_out_len);
        }

        // Read compressed data
        unsigned char* in = workbuf + buf_len - in_len;
        unsigned char* out = workbuf;
        if (extent_reader_read(reader, in, in_len) != in_len) {
            fprintf(stderr, "Failed to read compressed data block %d for %s\n", block_num, output_path);
            result = -1;
            break;
        }

        // Decompress this block
        int error = 0;
        uint16_t actual_out_len = 0;
        if (defhdr.cmptype == CMP_LZO) {
            lzo_uint lzo_out_len = defhdr.blksize;
            error = lzo1x_decompress_safe(in, in_len, out, &lzo_out_len, NULL);
            actual_out_len = lzo_out_len;
            if (error != LZO_E_OK) {
                fprintf(stderr, "LZO decompression error %d for %s\n", error, output_path);
                result = -1;
                break;
            }
        } else if (defhdr.cmptype == CMP_UCL) {
            ucl_uint ucl_out_len = defhdr.blksize;
            error = ucl_nrv2b_decompress_safe_8(in, in_len, out, &ucl_out_len, NULL);
            actual_out_len = ucl_out_len;
            if (error != UCL_E_OK) {
                fprintf(stderr, "UCL decompression error %d for %s\n", error, output_path);
                result = -1;
                break;
            }
        } else {
            fprintf(stderr, "Unknown compression type %d for %s\n", defhdr.cmptype, output_path);
            result = -1;
            break;
        }

        // Write only up to the declared uncompressed file size.
        size_t remaining = defhdr.usize - total_written;
        size_t out_len = actual_out_len < remaining ? actual_out_len : remaining;

        // Write decompressed data
        ssize_t written = write(outfd, out, out_len);
        if (written < 0 || (size_t)written != out_len) {
            fprintf(stderr, "Failed to write to file %s: %s\n", output_path, strerror(errno));
            result = -1;
            break;
        }
        total_written += out_len;
    }
    if (result == 0 && total_written != defhdr.usize) {
        fprintf(stderr, "Incomplete compressed file %s: expected %u bytes, wrote %zu bytes\n",
                output_path, defhdr.usize, total_written);
        result = -1;
    }

    free(workbuf);
    close(outfd);

    if (g_options.verbose) {
        printf("  Extracted %zu bytes to %s\n", total_written, output_path);
    }
    return result;
}

// Functions to read structures with endianness conversion
int read_stat(FILE *fp, stat_s *st, efs_byte_order_t byte_order) {
    if (fread(st, sizeof(stat_s), 1, fp) != 1) {
        return -1;
    }
    st->struct_size = read_efs16(st->struct_size, byte_order);
    st->mode = read_efs16(st->mode, byte_order);
    st->uid = read_efs32(st->uid, byte_order);
    st->gid = read_efs32(st->gid, byte_order);
    st->mtime = read_efs32(st->mtime, byte_order);
    st->ctime = read_efs32(st->ctime, byte_order);
    return 0;
}

int read_dirent(FILE *fp, dirent_s *dir, long base_offset, uint8_t hi, uint16_t lo,
        uint16_t text_size, efs_byte_order_t byte_order) {
    // Calculate absolute offset
    long offset = base_offset + (long)(hi) * 0x40000 + (long)(lo) * 4;
    long current = ftell(fp);
    const size_t fixed_size = sizeof(uint16_t) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(extptr_s);

    memset(dir, 0, sizeof(*dir));

    if (fseek(fp, offset, SEEK_SET) != 0) {
        fseek(fp, current, SEEK_SET);
        return -1;
    }

    // Read fixed portion
    if (fread(&dir->struct_size, sizeof(uint16_t), 1, fp) != 1) {
        fseek(fp, current, SEEK_SET);
        return -1;
    }
    dir->struct_size = read_efs16(dir->struct_size, byte_order);
    if (fread(&dir->moves, sizeof(uint8_t), 1, fp) != 1 ||
        fread(&dir->namelen, sizeof(uint8_t), 1, fp) != 1 ||
        fread(&dir->first, sizeof(extptr_s), 1, fp) != 1) {
        fseek(fp, current, SEEK_SET);
        return -1;
    }
    dir->first.logi_unit = read_efs16(dir->first.logi_unit, byte_order);
    dir->first.index = read_efs16(dir->first.index, byte_order);

    // Allocate and read name
    size_t name_size = F3S_NAME_ALIGN(dir->namelen);
    size_t required_size = dir->struct_size + name_size + sizeof(stat_s);
    if (dir->struct_size < fixed_size || required_size > text_size) {
        fseek(fp, current, SEEK_SET);
        return -1;
    }

    dir->name = malloc(name_size + 1);
    if (!dir->name) {
        fseek(fp, current, SEEK_SET);
        return -1;
    }
    if (fread(dir->name, name_size, 1, fp) != 1) {
        free(dir->name);
        dir->name = NULL;
        fseek(fp, current, SEEK_SET);
        return -1;
    }
    dir->name[dir->namelen] = '\0';

    // Read stat structure
    if (read_stat(fp, &dir->stat, byte_order) != 0) {
        free(dir->name);
        dir->name = NULL;
        fseek(fp, current, SEEK_SET);
        return -1;
    }

    // Restore position
    fseek(fp, current, SEEK_SET);
    return 0;
}

int read_unit_info(FILE *fp, unit_info_s *ui, efs_byte_order_t byte_order) {
    if (fread(ui, sizeof(unit_info_s), 1, fp) != 1) {
        return -1;
    }
    ui->struct_size = read_efs16(ui->struct_size, byte_order);
    ui->unit_pow2 = read_efs16(ui->unit_pow2, byte_order);
    ui->reserve = read_efs16(ui->reserve, byte_order);
    ui->erase_count = read_efs32(ui->erase_count, byte_order);
    ui->boot.logi_unit = read_efs16(ui->boot.logi_unit, byte_order);
    ui->boot.index = read_efs16(ui->boot.index, byte_order);
    return 0;
}

int read_unit_logi(FILE *fp, unit_logi_s *ul, efs_byte_order_t byte_order) {
    if (fread(ul, sizeof(unit_logi_s), 1, fp) != 1) {
        return -1;
    }
    ul->struct_size = read_efs16(ul->struct_size, byte_order);
    ul->logi = read_efs16(ul->logi, byte_order);
    ul->age = read_efs32(ul->age, byte_order);
    for(int i = 0; i < 4; i++) {
        ul->md5[i] = read_efs32(ul->md5[i], byte_order);
    }
    return 0;
}

int read_boot_info(FILE *fp, boot_info_s *bi, efs_byte_order_t byte_order) {
    if (fread(bi, sizeof(boot_info_s), 1, fp) != 1) {
        return -1;
    }
    bi->struct_size = read_efs16(bi->struct_size, byte_order);
    bi->unit_index = read_efs16(bi->unit_index, byte_order);
    bi->unit_total = read_efs16(bi->unit_total, byte_order);
    bi->unit_spare = read_efs16(bi->unit_spare, byte_order);
    bi->align_pow2 = read_efs16(bi->align_pow2, byte_order);
    bi->root.logi_unit = read_efs16(bi->root.logi_unit, byte_order);
    bi->root.index = read_efs16(bi->root.index, byte_order);
    return 0;
}

int read_head(FILE *fp, head_s *hd, efs_byte_order_t byte_order) {
    if (fread(hd, sizeof(head_s), 1, fp) != 1) {
        return -1;
    }
    if (byte_order == EFS_BYTE_ORDER_LITTLE) {
        /*
         * status_b describes QNX's big-endian 32-bit status word as
         * padding16, flags2, flags1. Little-endian F3S stores the same word as
         * flags1, flags2, padding16, so it needs a field-order conversion in
         * addition to the scalar conversions below.
         */
        for (int i = 0; i < 3; i++) {
            uint8_t raw[sizeof(status_b)];
            memcpy(raw, &hd->status[i], sizeof(raw));
            hd->status[i].flags1 = raw[0];
            hd->status[i].flags2 = raw[1];
            hd->status[i].padding1 = (uint16_t)raw[2] | ((uint16_t)raw[3] << 8);
        }
    }
    for(int i = 0; i < 3; i++) {
        hd->pad[i] = read_efs16(hd->pad[i], byte_order);
    }
    hd->text_offset_lo = read_efs16(hd->text_offset_lo, byte_order);
    hd->text_size = read_efs16(hd->text_size, byte_order);
    hd->next.logi_unit = read_efs16(hd->next.logi_unit, byte_order);
    hd->next.index = read_efs16(hd->next.index, byte_order);
    hd->super.logi_unit = read_efs16(hd->super.logi_unit, byte_order);
    hd->super.index = read_efs16(hd->super.index, byte_order);
    return 0;
}

int read_blocks(FILE *fp, blocks_s *blocks, size_t unit_size, efs_byte_order_t byte_order) {
    long block_top_pos = ftell(fp) - 40;

    if (unit_size < EFS_BOOTINFO_OFFSET) {
        return -1;
    }

    // Skip padding
    if (fseek(fp, (long)(unit_size - EFS_BOOTINFO_OFFSET), SEEK_CUR) != 0) {
        return -1;
    }

    if (read_unit_info(fp, &blocks->ui, byte_order) != 0 ||
        read_unit_logi(fp, &blocks->ul, byte_order) != 0) {
        return -1;
    }

    if (g_options.verbose) {
        print_unit_info("      ", &blocks->ui);
        print_unit_logi("      ", &blocks->ul);
    }

    long pos = ftell(fp);
    if (fseek(fp, -40-32, SEEK_CUR) != 0) {
        return -1;
    }

    // Initialize arrays
    blocks->heads = NULL;
    blocks->dirents = NULL;
    blocks->head_count = 0;

    while(1) {
        blocks->head_count++;
        head_s *new_heads = realloc(blocks->heads, blocks->head_count * sizeof(head_s));
        if (!new_heads) {
            blocks->head_count--;
            return -1;
        }
        blocks->heads = new_heads;

        dirent_s *new_dirents = realloc(blocks->dirents, blocks->head_count * sizeof(dirent_s));
        if (!new_dirents) {
            blocks->head_count--;
            return -1;
        }
        blocks->dirents = new_dirents;
        memset(&blocks->dirents[blocks->head_count - 1], 0, sizeof(dirent_s));

        // Read header
        if (read_head(fp, &blocks->heads[blocks->head_count - 1], byte_order) != 0) {
            return -1;
        }

        if (g_options.verbose) {
            printf("      Header %zu:\n", blocks->head_count);
            print_head("        ", &blocks->heads[blocks->head_count - 1]);
        }

        // Read corresponding dirent
        extent_type_e exttype = (blocks->heads[blocks->head_count - 1].status[0].flags2) & 0x03;
        if (exttype == EXTTYPE_DIR) {
            if (read_dirent(fp, &blocks->dirents[blocks->head_count - 1], block_top_pos,
                    blocks->heads[blocks->head_count - 1].text_offset_hi,
                    blocks->heads[blocks->head_count - 1].text_offset_lo,
                    blocks->heads[blocks->head_count - 1].text_size, byte_order) != 0) {
                blocks->dirents[blocks->head_count - 1].name = NULL;
                if (g_options.verbose) {
                    printf("        Warning: failed to read directory entry\n");
                }
            } else if (g_options.verbose) {
                print_dirent("        ", &blocks->dirents[blocks->head_count - 1]);
            }
        } else {
            // Initialize name to NULL for non-directory entries
            blocks->dirents[blocks->head_count - 1].name = NULL;
        }
        if (fseek(fp, -64, SEEK_CUR) != 0) {
            return -1;
        }

        // Check LAST flag in first status
        if((blocks->heads[blocks->head_count - 1].status[0].flags1 >> 7) & 0x1) {
            break;
        }

        // Safety limit to prevent infinite loops
        if (blocks->head_count > 1000) {
            if (g_options.verbose) {
                printf("      Warning: Hit safety limit of 1000 headers, breaking\n");
            }
            break;
        }
    }
    fseek(fp, pos, SEEK_SET);
    return 0;
}

size_t read_partition(FILE *fp, part_s *part) {
    memset(part, 0, sizeof(*part));

    long partition_start = ftell(fp);
    unit_info_s first_unit;
    if (fread(&first_unit, sizeof(first_unit), 1, fp) != 1 ||
        fseek(fp, partition_start, SEEK_SET) != 0) {
        return 0;
    }
    if (first_unit.endian == 'L') {
        part->byte_order = EFS_BYTE_ORDER_LITTLE;
    } else if (first_unit.endian == 'B') {
        part->byte_order = EFS_BYTE_ORDER_BIG;
    } else {
        fprintf(stderr, "Invalid F3S byte-order marker 0x%02x\n",
                (unsigned char)first_unit.endian);
        return 0;
    }
    part->unit_pow2 = read_efs16(first_unit.unit_pow2, part->byte_order);
    if (part->unit_pow2 >= sizeof(size_t) * 8 ||
        ((size_t)1 << part->unit_pow2) < EFS_BOOTINFO_OFFSET) {
        fprintf(stderr, "Invalid F3S unit size power %u\n", part->unit_pow2);
        return 0;
    }
    size_t unit_size = (size_t)1 << part->unit_pow2;

    // Skip to boot info location
    if (fseek(fp, EFS_BOOTINFO_OFFSET, SEEK_CUR) != 0) {
        return 0;
    }
//    read_unit_info(fp, &blocks->ui);
//    read_unit_logi(fp, &blocks->ul);
    if (read_boot_info(fp, &part->bi, part->byte_order) != 0) {
        return 0;
    }
    if (fseek(fp, -24, SEEK_CUR) != 0) {
        return 0;
    }

    if (part->bi.unit_total == 0 || part->bi.unit_total > 4096 || memcmp(part->bi.sig, "QSSL_F3S", 8) != 0) {
        fprintf(stderr, "Invalid partition boot info\n");
        return 0;
    }

    if (g_options.verbose) {
        printf("  Partition Boot Info (%s-endian):\n",
               part->byte_order == EFS_BYTE_ORDER_LITTLE ? "little" : "big");
        print_boot_info("    ", &part->bi);
    }

    // Allocate and read blocks
    part->blocks = calloc(part->bi.unit_total, sizeof(blocks_s));
    if (!part->blocks) {
        return 0;
    }
    for(int i = 0; i < part->bi.unit_total; i++) {
        if (g_options.verbose) {
            printf("    Logical Unit %d:\n", i + 1);
        }
        if (read_blocks(fp, &part->blocks[i], unit_size, part->byte_order) != 0) {
            if (g_options.verbose) {
                printf("    Warning: failed to read logical unit %d, treating it as empty\n", i + 1);
            }
            for (size_t j = 0; j < part->blocks[i].head_count; j++) {
                free(part->blocks[i].dirents[j].name);
            }
            free(part->blocks[i].heads);
            free(part->blocks[i].dirents);
            part->blocks[i].heads = NULL;
            part->blocks[i].dirents = NULL;
            part->blocks[i].head_count = 0;
            clearerr(fp);
        }
    }
    return part->bi.unit_total * unit_size;
}

void free_partition(part_s *part) {
    if (!part->blocks) {
        return;
    }
    for(int i = 0; i < part->bi.unit_total; i++) {
        for(size_t j = 0; j < part->blocks[i].head_count; j++) {
            // Only free name if it was allocated (non-NULL)
            if (part->blocks[i].dirents[j].name != NULL) {
                free(part->blocks[i].dirents[j].name);
            }
        }
        free(part->blocks[i].heads);
        free(part->blocks[i].dirents);
    }
    free(part->blocks);
}

void print_extptr(const char* prefix, const char* label, extptr_s *ptr) {
    printf("%s%s Extended Pointer:\n", prefix, label);
    printf("%s  Logical Unit: %u\n", prefix, ptr->logi_unit);
    printf("%s  Index: %u\n", prefix, ptr->index);
}

void print_status(const char* prefix, status_b *status) {
    printf("%sStatus:\n", prefix);

    // Extract fields from packed bits
    extent_type_e exttype = (status->flags2) & 0x03;
    uint8_t basic = (status->flags2 >> 3) & 0x01;
    uint8_t write = (status->flags1 >> 0) & 0x01;
    uint8_t next = (status->flags1 >> 1) & 0x01;
    uint8_t super = (status->flags1 >> 2) & 0x01;
    uint8_t split = (status->flags1 >> 3) & 0x01;
    block_status_e blkstat = (status->flags1 >> 4) & 0x07;
    uint8_t last = (status->flags1 >> 7) & 0x01;

    printf("%s  Extent Type: %s\n", prefix,
           exttype == EXTTYPE_XIP ? "XIP" :
           exttype == EXTTYPE_SYS ? "SYS" :
           exttype == EXTTYPE_DIR ? "DIR" :
           exttype == EXTTYPE_FILE ? "FILE" : "Unknown");

    printf("%s  Extent Status: %s\n", prefix,
           blkstat == EXTSTAT_BAD ? "BAD" :
           blkstat == EXTSTAT_DELETE ? "DELETE" :
           blkstat == EXTSTAT_ALLOC ? "ALLOC" :
           blkstat == EXTSTAT_FREE ? "FREE" : "Unknown");

    printf("%s  Flags: %s%s%s%s%s%s\n", prefix,
           basic ? "BASIC " : "",
           write ? "WRITE " : "",
           next ? "NEXT " : "",
           super ? "SUPER " : "",
           split ? "SPLIT " : "",
           last ? "LAST" : "");
}

void print_unit_info(const char* prefix, unit_info_s *ui) {
    printf("%sUnit Info:\n", prefix);
    printf("%s  Structure Size: %u\n", prefix, ui->struct_size);
    printf("%s  Endian: %c\n", prefix, ui->endian);
    printf("%s  Unit Power of 2: %u\n", prefix, ui->unit_pow2);
    printf("%s  Reserve: %u\n", prefix, ui->reserve);
    printf("%s  Erase Count: %u\n", prefix, ui->erase_count);
    print_extptr(prefix, "Boot", &ui->boot);
}

void print_unit_logi(const char* prefix, unit_logi_s *ul) {
    printf("%sUnit Logical:\n", prefix);
    printf("%s  Structure Size: %u\n", prefix, ul->struct_size);
    printf("%s  Logical Unit: %u\n", prefix, ul->logi);
    printf("%s  Age: %u\n", prefix, ul->age);
    printf("%s  MD5: %08x%08x%08x%08x\n", prefix,
           ul->md5[0], ul->md5[1], ul->md5[2], ul->md5[3]);
}

void print_boot_info(const char* prefix, boot_info_s *bi) {
    printf("%sBoot Info:\n", prefix);
    printf("%s  Structure Size: %u\n", prefix, bi->struct_size);
    printf("%s  Revision: %u.%u\n", prefix, bi->rev_major, bi->rev_minor);
    printf("%s  Signature: %.8s\n", prefix, bi->sig);
    printf("%s  Unit Index: %u\n", prefix, bi->unit_index);
    printf("%s  Total Units: %u\n", prefix, bi->unit_total);
    printf("%s  Spare Units: %u\n", prefix, bi->unit_spare);
    printf("%s  Alignment Power of 2: %u\n", prefix, bi->align_pow2);
    print_extptr(prefix, "Root", &bi->root);
}

void print_head(const char* prefix, head_s *hd) {
    printf("%sHeader:\n", prefix);
    char new_prefix[256];
    snprintf(new_prefix, sizeof(new_prefix), "%s  ", prefix);

    for(int i = 0; i < 3; i++) {
        printf("%sStatus %d:\n", prefix, i);
        print_status(new_prefix, &hd->status[i]);
        break; // because these are all duplicates
    }

    printf("%s  Reserve: %u\n", prefix, hd->reserve);
    printf("%s  Text Offset: 0x%02x%04x\n", prefix,
           hd->text_offset_hi, hd->text_offset_lo);
    printf("%s  Text Size: %u\n", prefix, hd->text_size);
    print_extptr(prefix, "Next", &hd->next);
    print_extptr(prefix, "Super", &hd->super);
}

// Mode string helper
void modestr(uint16_t mode, char* str) {
    // Handle file type (upper byte)
    switch (mode & 0xf000) {
        case 0x2000: str[0] = 'c'; break;  // character device
        case 0x4000: str[0] = 'd'; break;  // directory
        case 0x6000: str[0] = 'b'; break;  // block device
        case 0x1000: str[0] = 'p'; break;  // FIFO
        case 0xa000: str[0] = 'l'; break;  // symbolic link
        case 0xc000: str[0] = 's'; break;  // socket
        case 0x8000: str[0] = '-'; break;  // regular file
        default: str[0] = '?'; break;
    }

    // Handle permissions (lower byte)
    // Owner
    str[1] = (mode & 0x0100) ? 'r' : '-';
    str[2] = (mode & 0x0080) ? 'w' : '-';
    str[3] = (mode & 0x0040) ? 'x' : '-';
    // Group
    str[4] = (mode & 0x0020) ? 'r' : '-';
    str[5] = (mode & 0x0010) ? 'w' : '-';
    str[6] = (mode & 0x0008) ? 'x' : '-';
    // Others
    str[7] = (mode & 0x0004) ? 'r' : '-';
    str[8] = (mode & 0x0002) ? 'w' : '-';
    str[9] = (mode & 0x0001) ? 'x' : '-';

    str[10] = '\0';
}

void print_stat(const char* prefix, stat_s *st) {
    char mode_str[16];
    modestr(st->mode, mode_str);
    printf("%sStat Info:\n", prefix);
    printf("%s  Structure Size: %u\n", prefix, st->struct_size);
    printf("%s  Mode: (%04o) %s\n", prefix, st->mode, mode_str);
    printf("%s  UID: %u\n", prefix, st->uid);
    printf("%s  GID: %u\n", prefix, st->gid);
    printf("%s  Modified Time: %u\n", prefix, st->mtime);
    printf("%s  Created Time: %u\n", prefix, st->ctime);
}

void print_dirent(const char* prefix, dirent_s *dir) {
    printf("%sDirectory Entry:\n", prefix);
    printf("%s  Structure Size: %u\n", prefix, dir->struct_size);
    printf("%s  Moves: %u\n", prefix, dir->moves);
    printf("%s  Name Length: %u\n", prefix, dir->namelen);
    printf("%s  Name: %s\n", prefix, dir->name);
    print_extptr(prefix, "First", &dir->first);

    print_stat(prefix, &dir->stat);
}

void print_blocks(const char* prefix, blocks_s *blocks) {
    printf("%sLogical Units:\n", prefix);
    char new_prefix[256];
    snprintf(new_prefix, sizeof(new_prefix), "%s  ", prefix);

    print_unit_info(new_prefix, &blocks->ui);
    print_unit_logi(new_prefix, &blocks->ul);

    printf("%s  Headers and Directory Entries (%zu):\n", prefix, blocks->head_count);
    char header_prefix[256];
    snprintf(header_prefix, sizeof(header_prefix), "%s    ", prefix);

    for(size_t i = 0; i < blocks->head_count; i++) {
        printf("%s  Header %zu:    ----------------------------------------\n", prefix, i+1);
        print_head(header_prefix, &blocks->heads[i]);
        extent_type_e exttype = (blocks->heads[i].status[0].flags2) & 0x03;
        if (exttype == EXTTYPE_DIR) {
            print_dirent(header_prefix, &blocks->dirents[i]);
        }
    }
}

void print_partition(const char* prefix, part_s *part) {
    printf("%sPartition:\n", prefix);
    char new_prefix[256];
    snprintf(new_prefix, sizeof(new_prefix), "%s  ", prefix);

    print_boot_info(new_prefix, &part->bi);

    for(int i = 0; i < part->bi.unit_total; i++) {
        printf("%sLogical Unit %d:    ========================================\n", new_prefix, i+1);
        char block_prefix[512];  // Increase buffer size to accommodate prefix + "  "
        snprintf(block_prefix, sizeof(block_prefix), "%s  ", new_prefix);
        print_blocks(block_prefix, &part->blocks[i]);
    }
}

// Extract filesystem by building directory tree and extracting files
void extract_filesystem(FILE *fp, part_s *partitions, size_t count) {
    printf("Extracting QNX EFS Filesystem:\n");
    for (size_t i = 0; i < count; i++) {
        printf("Extracting partition %zu:\n", i + 1);
        extract_partition(fp, &partitions[i]);
    }
}

// Extract partition by following the filesystem tree structure starting from root
void extract_partition(FILE *fp, part_s *part) {
    if (g_options.verbose) {
        printf("  Boot Info: %.8s (Rev %u.%u), %u units\n",
               part->bi.sig, part->bi.rev_major, part->bi.rev_minor, part->bi.unit_total);

        // Start traversal from the root directory specified in boot info
        printf("  Starting from root: unit=%u, index=%u\n",
               part->bi.root.logi_unit, part->bi.root.index);
    }

    // Traverse the filesystem tree starting from root
    if (g_options.verbose) {
        printf("  Starting extraction traversal...\n");
    }
    traverse_directory(fp, part, part->bi.root.logi_unit, part->bi.root.index);
    if (g_options.verbose) {
        printf("  Extraction traversal completed\n");
    }
}

// Process a directory entry and extract files by following extent pointers
void extract_directory_entry(FILE *fp, part_s *part, dir_state_t *dir_state, dirent_s *dirent) {
    char mode_str[16];
    modestr(dirent->stat.mode, mode_str);

    printf("    Directory entry: %s (%s)\n", dirent->name, mode_str);

    if ((dirent->stat.mode & 0xf000) == 0x4000) {  // Directory
        if (strcmp(dirent->name, "/") == 0) {
            // Root directory - reset to root
            strcpy(dir_state->path, ".");
            dir_state->len = 1;
            if (mkdir(dir_state->path, 0755) != 0 && errno != EEXIST) {
                if (errno != EEXIST) {
                    fprintf(stderr, "Failed to create directory %s: %s\n",
                           dir_state->path, strerror(errno));
                }
            }
            printf("      Root directory: %s\n", dir_state->path);
        } else if (strcmp(dirent->name, ".") == 0) {
            // Current directory - just create it, don't change path
            if (mkdir(dir_state->path, 0755) != 0 && errno != EEXIST) {
                if (errno != EEXIST) {
                    fprintf(stderr, "Failed to create directory %s: %s\n",
                           dir_state->path, strerror(errno));
                }
            }
            printf("      Current directory: %s\n", dir_state->path);
        } else if (strcmp(dirent->name, "..") == 0) {
            // Parent directory - go up
            pop_dir(dir_state);
            printf("      Moved up to: %s\n", dir_state->path);
        } else {
            // Regular directory - this is a new subdirectory
            // Reset to root and then add this directory
            strcpy(dir_state->path, ".");
            dir_state->len = 1;
            push_dir(dir_state, dirent->name);
            if (mkdir(dir_state->path, 0755) != 0 && errno != EEXIST) {
                fprintf(stderr, "Failed to create directory %s: %s\n",
                       dir_state->path, strerror(errno));
            } else {
                printf("      Created directory: %s\n", dir_state->path);
            }
        }
    } else if ((dirent->stat.mode & 0xf000) == 0x8000) {  // Regular file
        // This directory entry refers to a file - find the corresponding file extent
        char filepath[MAX_PATH_LEN];
        int ret = snprintf(filepath, sizeof(filepath), "%s/%s", dir_state->path, dirent->name);
        if (ret >= (int)sizeof(filepath)) {
            fprintf(stderr, "Warning: Path too long, truncated: %s/%s\n", dir_state->path, dirent->name);
        }

        printf("      File entry: %s (extent: unit=%u, index=%u)\n",
               filepath, dirent->first.logi_unit, dirent->first.index);

        // Extract the file using the new implementation
        if (extract_file_data(fp, part, dirent->first.logi_unit, dirent->first.index, filepath) == 0) {
            printf("        Successfully extracted: %s\n", filepath);
        } else {
            fprintf(stderr, "        Failed to extract: %s\n", filepath);
        }
    }
}

// Print usage information
void print_usage(const char *progname) {
    printf("QNX Compressed-EFS Filesystem Extractor\n");
    printf("Usage: %s [OPTIONS] <filesystem_image>\n\n", progname);
    printf("Options:\n");
    printf("  -d, --directory DIR    Extract to directory DIR (default: current directory)\n");
    printf("  -l, --list-partitions  List all partitions in the image\n");
    printf("  -p, --partition NUM    Extract only partition NUM (0-based, default: all)\n");
    printf("  -c, --list-contents    List contents without extracting files\n");
    printf("  -v, --verbose          Enable verbose output\n");
    printf("  -h, --help             Show this help message\n\n");
    printf("Examples:\n");
    printf("  %s image.bin                    # Extract all partitions to current directory\n", progname);
    printf("  %s -d /tmp/extract image.bin    # Extract to /tmp/extract\n", progname);
    printf("  %s -l image.bin                 # List all partitions\n", progname);
    printf("  %s -p 0 -c image.bin            # List contents of partition 0\n", progname);
    printf("  %s -v -d output image.bin       # Verbose extraction to output directory\n", progname);
}

// Parse command line arguments
int parse_arguments(int argc, char *argv[]) {
    static struct option long_options[] = {
        {"directory",       required_argument, 0, 'd'},
        {"list-partitions", no_argument,       0, 'l'},
        {"partition",       required_argument, 0, 'p'},
        {"list-contents",   no_argument,       0, 'c'},
        {"verbose",         no_argument,       0, 'v'},
        {"help",            no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };

    int option_index = 0;
    int c;

    while ((c = getopt_long(argc, argv, "d:lp:cvh", long_options, &option_index)) != -1) {
        switch (c) {
            case 'd':
                g_options.target_dir = optarg;
                break;
            case 'l':
                g_options.list_partitions = 1;
                break;
            case 'p':
                g_options.partition_index = atoi(optarg);
                break;
            case 'c':
                g_options.list_only = 1;
                break;
            case 'v':
                g_options.verbose = 1;
                break;
            case 'h':
                g_options.help = 1;
                break;
            case '?':
                return -1;
            default:
                return -1;
        }
    }

    // Check if we have the required image argument
    if (optind >= argc && !g_options.help) {
        fprintf(stderr, "Error: Missing filesystem image argument\n\n");
        print_usage(argv[0]);
        return -1;
    }

    return optind;  // Return index of the image file argument
}

int main(int argc, char *argv[]) {
    // Parse command line arguments
    int image_arg_index = parse_arguments(argc, argv);

    if (image_arg_index < 0) {
        return 1;
    }

    if (g_options.help) {
        print_usage(argv[0]);
        return 0;
    }

    const char *image_file = argv[image_arg_index];

    // Initialize compression libraries
    if (lzo_init() != LZO_E_OK) {
        fprintf(stderr, "Error: Failed to initialize LZO library\n");
        return 1;
    }

    if (ucl_init() != UCL_E_OK) {
        fprintf(stderr, "Error: Failed to initialize UCL library\n");
        return 1;
    }

    FILE *fp = fopen(image_file, "rb");
    if (!fp) {
        perror("Failed to open file");
        return 1;
    }

    // Check if this is a CKSUM file and adjust file pointer if needed
    long efs_offset = 0;
    if (is_cksum_file(fp)) {
        if (g_options.verbose) {
            printf("Detected CKSUM file format\n");
        }

        efs_offset = find_first_efs_partition(fp);
        if (efs_offset < 0) {
            fprintf(stderr, "Error: No EFS partitions found in CKSUM file\n");
            fclose(fp);
            return 1;
        }

        // Seek to the first EFS partition
        fseek(fp, efs_offset, SEEK_SET);

        if (g_options.verbose) {
            printf("Positioned at EFS partition offset 0x%lx\n", efs_offset);
        }
    } else {
        if (g_options.verbose) {
            printf("Standard EFS file format detected\n");
        }
    }

    // Read partitions
    size_t next_part = 0;
    part_s partitions[2];
    int partition_count = 0;

    // Read first partition
    if (g_options.verbose) {
        printf("Reading partition 0...\n");
    }
    next_part = read_partition(fp, &partitions[0]);
    partitions[0].efs_offset = efs_offset;  // Store the EFS offset
    if (next_part > 0) {
        partition_count++;

        // Try to find second partition by searching for next QSSL_F3S signature
        if (g_options.verbose) {
            printf("Reading partition 1...\n");
        }

        // Search for the next EFS partition starting after the first one
        long search_start = efs_offset + next_part;
        long second_efs_offset = find_next_efs_partition(fp, search_start);
        if (second_efs_offset >= 0) {
            fseek(fp, second_efs_offset, SEEK_SET);
            size_t next_part2 = read_partition(fp, &partitions[1]);
            partitions[1].efs_offset = second_efs_offset;  // Store the second EFS offset
            if (next_part2 > 0) {
                partition_count++;
            }
        }
    }

    if (partition_count == 0) {
        fprintf(stderr, "Error: No valid partitions found in image\n");
        fclose(fp);
        return 1;
    }

    // Handle list partitions option
    if (g_options.list_partitions) {
        printf("Found %d partition(s) in %s:\n", partition_count, image_file);
        for (int i = 0; i < partition_count; i++) {
            printf("Partition %d:\n", i);
            printf("  Boot Info: %.8s (Rev %u.%u), %u units\n",
                   partitions[i].bi.sig, partitions[i].bi.rev_major,
                   partitions[i].bi.rev_minor, partitions[i].bi.unit_total);
            printf("  Root: unit=%u, index=%u\n",
                   partitions[i].bi.root.logi_unit, partitions[i].bi.root.index);
        }

        // Clean up and exit
        for (int i = 0; i < partition_count; i++) {
            free_partition(&partitions[i]);
        }
        fclose(fp);
        return 0;
    }

    // Change to target directory if specified
    if (strcmp(g_options.target_dir, ".") != 0) {
        if (mkdir(g_options.target_dir, 0755) != 0 && errno != EEXIST) {
            perror("Failed to create target directory");
            fclose(fp);
            return 1;
        }
        if (chdir(g_options.target_dir) != 0) {
            perror("Failed to change to target directory");
            fclose(fp);
            return 1;
        }
        if (g_options.verbose) {
            printf("Changed to target directory: %s\n", g_options.target_dir);
        }
    }

    // Extract or list filesystem contents
    if (g_options.partition_index >= 0) {
        // Extract specific partition
        if (g_options.partition_index >= partition_count) {
            fprintf(stderr, "Error: Partition %d not found (only %d partitions available)\n",
                    g_options.partition_index, partition_count);
            for (int i = 0; i < partition_count; i++) {
                free_partition(&partitions[i]);
            }
            fclose(fp);
            return 1;
        }

        if (g_options.verbose || g_options.list_only) {
            printf("%s partition %d:\n", g_options.list_only ? "Listing" : "Extracting",
                   g_options.partition_index);
        }
        extract_partition(fp, &partitions[g_options.partition_index]);
    } else {
        // Extract all partitions
        if (g_options.verbose || g_options.list_only) {
            printf("%s QNX EFS Filesystem:\n", g_options.list_only ? "Listing" : "Extracting");
        }
        extract_filesystem(fp, partitions, partition_count);
    }

    // Clean up
    for (int i = 0; i < partition_count; i++) {
        free_partition(&partitions[i]);
    }
    fclose(fp);

    return 0;
}
