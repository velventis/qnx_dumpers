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
#include <arpa/inet.h>
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


#define F3S_NAME_ALIGN(LENGTH)\
\
  (((LENGTH)+(sizeof(uint32_t)-1))&~(sizeof(uint32_t)-1))

// File compression constants
#define CMP_LZO     0
#define CMP_UCL     1
#define MAX_PATH_LEN 4096
#define WORKBUF_SIZE (512*1024)

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

typedef struct {
    boot_info_s bi;
    blocks_s *blocks;  // Array size determined by bi.unit_total
    long efs_offset;   // Offset where this EFS partition starts in the file
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

// Forward declarations
void extract_filesystem(FILE *fp, part_s *partitions, size_t count);
void extract_partition(FILE *fp, part_s *part);
void extract_directory_entry(FILE *fp, part_s *part, dir_state_t *dir_state, dirent_s *dirent);
head_s* find_file_extent(part_s *part, uint16_t logi_unit, uint16_t index);
void traverse_directory(FILE *fp, part_s *part, uint16_t logi_unit, uint16_t index);
int collect_file_extents(part_s *part, uint16_t logi_unit, uint16_t index, file_extent_t **extents, int *extent_count);
int extract_contiguous_file_data(FILE *fp, file_extent_t *extents, int extent_count, uint8_t **data_buffer, size_t *total_size);
int extract_file_data(FILE *fp, part_s *part, uint16_t logi_unit, uint16_t index, const char *output_path);
int extract_compressed_file_data(FILE *fp, long file_offset, const char *output_path);
int extract_raw_file_data(FILE *fp, long file_offset, uint16_t file_size, const char *output_path);
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

// Function to swap endianness if needed
uint16_t swap16(uint16_t val) {
    return ntohs(val);
}

uint32_t swap32(uint32_t val) {
    return ntohl(val);
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
    if (logi_unit == 0 || logi_unit > part->bi.unit_total) {
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

// Collect all extents for a file by following the extent chain
int collect_file_extents(part_s *part, uint16_t logi_unit, uint16_t index, file_extent_t **extents, int *extent_count) {
    *extents = NULL;
    *extent_count = 0;
    
    uint16_t current_logi = logi_unit;
    uint16_t current_index = index;
    
    // Follow the extent chain
    while (current_logi != 0 && current_logi != 65535) {
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
        
        // Calculate the actual data offset within the block
        // Each block is unit_pow2 size, data comes before the head_s tables
        long block_base = part->efs_offset + (current_logi - 1) * (1LL << blocks->ui.unit_pow2);
        long data_offset = block_base + (long)(head->text_offset_hi) * 0x40000 + (long)(head->text_offset_lo) * 4;
        
        // Reallocate extent array
        *extents = realloc(*extents, (*extent_count + 1) * sizeof(file_extent_t));
        if (!*extents) {
            *extent_count = 0;
            return -1;
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
        if (head->next.logi_unit == 0 || head->next.logi_unit == 65535) {
            break;
        }
        current_logi = head->next.logi_unit;
        current_index = head->next.index;
        
        // Safety check to prevent infinite loops
        if (*extent_count > 100) {
            if (g_options.verbose) {
                printf("      Warning: Hit safety limit of 100 extents\n");
            }
            break;
        }
    }
    
    return *extent_count;
}

// Extract contiguous file data from multiple extents
int extract_contiguous_file_data(FILE *fp, file_extent_t *extents, int extent_count, uint8_t **data_buffer, size_t *total_size) {
    *data_buffer = NULL;
    *total_size = 0;
    
    if (extent_count == 0) {
        return 0;  // Empty file
    }
    
    // Calculate total size needed
    size_t total_data_size = 0;
    for (int i = 0; i < extent_count; i++) {
        total_data_size += extents[i].size;
    }
    
    if (total_data_size == 0) {
        return 0;  // Empty file
    }
    
    // Allocate buffer for all data
    *data_buffer = malloc(total_data_size);
    if (!*data_buffer) {
        return -1;
    }
    
    // Read data from each extent
    size_t buffer_pos = 0;
    long current_pos = ftell(fp);
    
    for (int i = 0; i < extent_count; i++) {
        if (g_options.verbose) {
            printf("        Reading extent %d: offset=0x%lx, size=%u\n", 
                   i + 1, extents[i].offset, extents[i].size);
        }
        
        // Seek to extent data
        if (fseek(fp, extents[i].offset, SEEK_SET) != 0) {
            fprintf(stderr, "Failed to seek to extent %d at offset 0x%lx\n", i, extents[i].offset);
            free(*data_buffer);
            *data_buffer = NULL;
            fseek(fp, current_pos, SEEK_SET);
            return -1;
        }
        
        // Read extent data
        size_t bytes_read = fread(*data_buffer + buffer_pos, 1, extents[i].size, fp);
        if (bytes_read != extents[i].size) {
            fprintf(stderr, "Failed to read extent %d: expected %u bytes, got %zu bytes\n", 
                   i, extents[i].size, bytes_read);
            free(*data_buffer);
            *data_buffer = NULL;
            fseek(fp, current_pos, SEEK_SET);
            return -1;
        }
        
        buffer_pos += extents[i].size;
    }
    
    *total_size = total_data_size;
    fseek(fp, current_pos, SEEK_SET);
    
    if (g_options.verbose) {
        printf("        Successfully collected %zu bytes from %d extents\n", *total_size, extent_count);
    }
    
    return 0;
}

// Traverse directory structure by following the extent chain
// Read directory contents by following extent chain of a directory entry
void read_directory_contents(FILE *fp, part_s *part, dirent_s *dir_entry, const char *dir_path) {
    if (dir_entry->first.logi_unit == 0 || dir_entry->first.logi_unit == 65535) {
        return;  // No contents
    }
    
    if (g_options.verbose) {
        printf("    Reading contents of directory: %s\n", dir_path);
    }
    
    // Follow the extent chain to get all directory contents
    uint16_t logi_unit = dir_entry->first.logi_unit;
    uint16_t index = dir_entry->first.index;
    
    while (logi_unit != 0 && logi_unit != 65535) {
        if (logi_unit > part->bi.unit_total) {
            break;
        }
        
        blocks_s *blocks = &part->blocks[logi_unit - 1];
        if (index >= blocks->head_count) {
            break;
        }
        
        head_s *head = &blocks->heads[index];
        extent_type_e exttype = (head->status[0].flags2) & 0x03;
        
        if (exttype == EXTTYPE_DIR) {
            dirent_s *content_entry = &blocks->dirents[index];
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
                    read_directory_contents(fp, part, content_entry, subdir_path);
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
        if (head->next.logi_unit == 0 || head->next.logi_unit == 65535) {
            break;
        }
        logi_unit = head->next.logi_unit;
        index = head->next.index;
    }
}

void traverse_directory(FILE *fp, part_s *part, uint16_t logi_unit, uint16_t index) {
    if (logi_unit == 0 || logi_unit > part->bi.unit_total || logi_unit == 65535) {
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
        
        if (g_options.verbose) {
            printf("      Directory entry: %s\n", dirent->name);
        }
        
        // Process any directory entry, not just root
        if (dirent->name != NULL) {
            // Determine the directory path
            const char *dir_path = (strcmp(dirent->name, "/") == 0) ? "." : dirent->name + 1; // Skip leading '/'
            read_directory_contents(fp, part, dirent, dir_path);
        }
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
    
    // Extract contiguous file data from all extents
    uint8_t *file_data = NULL;
    size_t total_size = 0;
    
    if (extract_contiguous_file_data(fp, extents, extent_count, &file_data, &total_size) < 0) {
        fprintf(stderr, "Failed to extract file data for %s\n", output_path);
        free(extents);
        return -1;
    }
    
    free(extents);  // Done with extent array
    
    if (total_size == 0) {
        // Empty file
        create_directories(output_path);
        int fd = open(output_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd >= 0) {
            close(fd);
            if (g_options.verbose) {
                printf("        Created empty file: %s\n", output_path);
            }
        }
        return 0;
    }
    
    // Check if this is compressed data
    if (total_size >= 8 && memcmp(file_data, "iwlyfmbp", 8) == 0) {
        // This is compressed data - write to temp file and decompress
        if (g_options.verbose) {
            printf("        Found compression signature in %s\n", output_path);
        }
        
        // Create temporary file for compressed data
        char temp_path[MAX_PATH_LEN];
        snprintf(temp_path, sizeof(temp_path), "%s.tmp", output_path);
        
        create_directories(temp_path);
        int temp_fd = open(temp_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (temp_fd < 0) {
            fprintf(stderr, "Failed to create temp file %s: %s\n", temp_path, strerror(errno));
            free(file_data);
            return -1;
        }
        
        // Write compressed data to temp file
        if (write(temp_fd, file_data, total_size) != (ssize_t)total_size) {
            fprintf(stderr, "Failed to write compressed data to %s: %s\n", temp_path, strerror(errno));
            close(temp_fd);
            unlink(temp_path);
            free(file_data);
            return -1;
        }
        close(temp_fd);
        free(file_data);
        
        // Open temp file and decompress
        FILE *temp_fp = fopen(temp_path, "rb");
        if (!temp_fp) {
            fprintf(stderr, "Failed to reopen temp file %s: %s\n", temp_path, strerror(errno));
            unlink(temp_path);
            return -1;
        }
        
        int result = extract_compressed_file_data(temp_fp, 0, output_path);
        fclose(temp_fp);
        unlink(temp_path);  // Clean up temp file
        
        return result;
    } else {
        // Raw file data - write directly
        if (g_options.verbose) {
            printf("        Extracting raw file: %s (size: %zu bytes)\n", output_path, total_size);
        }
        
        create_directories(output_path);
        int fd = open(output_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd < 0) {
            fprintf(stderr, "Failed to create file %s: %s\n", output_path, strerror(errno));
            free(file_data);
            return -1;
        }
        
        if (write(fd, file_data, total_size) != (ssize_t)total_size) {
            fprintf(stderr, "Failed to write data to %s: %s\n", output_path, strerror(errno));
            close(fd);
            free(file_data);
            return -1;
        }
        
        close(fd);
        free(file_data);
        
        if (g_options.verbose) {
            printf("        Successfully extracted %zu bytes to %s\n", total_size, output_path);
        }
        return 0;
    }
}

// Extract raw (uncompressed) file data
int extract_raw_file_data(FILE *fp, long file_offset, uint16_t file_size, const char *output_path) {
    long current_pos = ftell(fp);
    
    if (fseek(fp, file_offset, SEEK_SET) != 0) {
        fprintf(stderr, "Failed to seek to raw file data at offset 0x%lx\n", file_offset);
        return -1;
    }
    
    // Create output file
    create_directories(output_path);
    int outfd = open(output_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (outfd < 0) {
        fprintf(stderr, "Failed to create raw file %s: %s\n", output_path, strerror(errno));
        fseek(fp, current_pos, SEEK_SET);
        return -1;
    }
    
    // Read and write data in chunks
    uint8_t buffer[4096];
    uint16_t remaining = file_size;
    int total_written = 0;
    
    while (remaining > 0) {
        size_t to_read = (remaining > sizeof(buffer)) ? sizeof(buffer) : remaining;
        size_t bytes_read = fread(buffer, 1, to_read, fp);
        
        if (bytes_read == 0) {
            if (feof(fp)) {
                printf("  Warning: EOF reached before expected size for %s\n", output_path);
                break;
            } else {
                fprintf(stderr, "Failed to read raw data for %s: %s\n", output_path, strerror(errno));
                close(outfd);
                fseek(fp, current_pos, SEEK_SET);
                return -1;
            }
        }
        
        ssize_t written = write(outfd, buffer, bytes_read);
        if (written < 0 || (size_t)written != bytes_read) {
            fprintf(stderr, "Failed to write raw data to %s: %s\n", output_path, strerror(errno));
            close(outfd);
            fseek(fp, current_pos, SEEK_SET);
            return -1;
        }
        
        remaining -= bytes_read;
        total_written += bytes_read;
    }
    
    close(outfd);
    fseek(fp, current_pos, SEEK_SET);
    
    if (g_options.verbose) {
        printf("  Extracted %d raw bytes to %s\n", total_written, output_path);
    }
    return 0;
}

// Extract compressed file data (original logic)
int extract_compressed_file_data(FILE *fp, long file_offset, const char *output_path) {
    long current_pos = ftell(fp);
    fseek(fp, file_offset, SEEK_SET);
    
    // Read compression header - keep both original and converted versions
    deflate_filehdr_t defhdr_orig, defhdr;
    if (fread(&defhdr_orig, sizeof(deflate_filehdr_t), 1, fp) != 1) {
        fprintf(stderr, "Failed to read compression header for %s\n", output_path);
        fseek(fp, current_pos, SEEK_SET);
        return -1;
    }
    
    // Make a copy and convert endianness for validation/display
    defhdr = defhdr_orig;
    defhdr.usize = swap32(defhdr.usize);
    defhdr.blksize = swap16(defhdr.blksize);
    
    // Accept block sizes as small as 16 bytes - this is valid for some files
    // Since blksize is uint16_t (max 65535), we only need to check the minimum
    if (defhdr.blksize < 16) {
        fprintf(stderr, "Invalid compression header for %s (blksize=%d)\n", 
               output_path, defhdr.blksize);
        fseek(fp, current_pos, SEEK_SET);
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
        fseek(fp, current_pos, SEEK_SET);
        return -1;
    }
    
    
    // Allocate work buffer using ORIGINAL header values (like efsdump2)
    size_t overhead = defhdr_orig.blksize / 8 + 256;
    size_t buf_len = defhdr_orig.blksize + overhead;
    unsigned char* workbuf = malloc(buf_len * 2);
    if (!workbuf) {
        fprintf(stderr, "Failed to allocate work buffer for %s\n", output_path);
        close(outfd);
        fseek(fp, current_pos, SEEK_SET);
        return -1;
    }
    
    // Decompress blocks
    int total_written = 0;
    for (int block_num = 0; ; block_num++) {
        cmphdr_t blockhdr;
        if (fread(&blockhdr, sizeof(cmphdr_t), 1, fp) != 1) {
            fprintf(stderr, "Failed to read block header %d for %s\n", block_num, output_path);
            break;
        }
        
        // NOTE: Do NOT apply endianness conversion to block headers - they're already in correct format
        if (blockhdr.next == 0) break;
        
        uint16_t in_len = blockhdr.next - sizeof(cmphdr_t);
        uint16_t expected_out_len = blockhdr.usize;  // Use block header's usize
        
        if (g_options.verbose) {
            printf("  Block %d: next=%u, usize=%u, in_len=%u, expected_out=%u\n", 
                   block_num, blockhdr.next, blockhdr.usize, in_len, expected_out_len);
        }
        
        // Read compressed data
        unsigned char* in = workbuf + buf_len - in_len;
        unsigned char* out = workbuf;
        if (fread(in, in_len, 1, fp) != 1) {
            fprintf(stderr, "Failed to read compressed data block %d for %s\n", block_num, output_path);
            break;
        }
        
        // Decompress using ORIGINAL header values (like efsdump2)
        int error = 0;
        uint16_t actual_out_len = 0;
        if (defhdr_orig.cmptype == CMP_LZO) {
            lzo_uint lzo_out_len = defhdr_orig.blksize;  // Buffer size from original header
            error = lzo1x_decompress_safe(in, in_len, out, &lzo_out_len, NULL);
            actual_out_len = lzo_out_len;
            if (error != LZO_E_OK) {
                fprintf(stderr, "LZO decompression error %d for %s\n", error, output_path);
                break;
            }
        } else if (defhdr_orig.cmptype == CMP_UCL) {
            ucl_uint ucl_out_len = defhdr_orig.blksize;  // Buffer size from original header
            error = ucl_nrv2b_decompress_safe_8(in, in_len, out, &ucl_out_len, NULL);
            actual_out_len = ucl_out_len;
            if (error != UCL_E_OK) {
                fprintf(stderr, "UCL decompression error %d for %s\n", error, output_path);
                break;
            }
        } else {
            fprintf(stderr, "Unknown compression type %d for %s\n", defhdr_orig.cmptype, output_path);
            break;
        }
        
        // Write the actual decompressed amount (should match expected)
        uint16_t out_len = actual_out_len;
        
        // Write decompressed data
        ssize_t written = write(outfd, out, out_len);
        if (written < 0 || (size_t)written != out_len) {
            fprintf(stderr, "Failed to write to file %s: %s\n", output_path, strerror(errno));
            break;
        }
        total_written += out_len;
    }
    
    free(workbuf);
    close(outfd);
    fseek(fp, current_pos, SEEK_SET);
    
    if (g_options.verbose) {
        printf("  Extracted %d bytes to %s\n", total_written, output_path);
    }
    return 0;
}

// Functions to read structures with endianness conversion
void read_stat(FILE *fp, stat_s *st) {
    fread(st, sizeof(stat_s), 1, fp);
    st->struct_size = swap16(st->struct_size);
    st->mode = swap16(st->mode);
    st->uid = swap32(st->uid);
    st->gid = swap32(st->gid);
    st->mtime = swap32(st->mtime);
    st->ctime = swap32(st->ctime);
}

void read_dirent(FILE *fp, dirent_s *dir, long base_offset, uint8_t hi, uint16_t lo) {
    // Calculate absolute offset
    long offset = base_offset + (long)(hi) * 0x40000 + (long)(lo) * 4;
    long current = ftell(fp);
    
    fseek(fp, offset, SEEK_SET);
    
    // Read fixed portion
    fread(&dir->struct_size, sizeof(uint16_t), 1, fp);
    dir->struct_size = swap16(dir->struct_size);
    fread(&dir->moves, sizeof(uint8_t), 1, fp);
    fread(&dir->namelen, sizeof(uint8_t), 1, fp);
    fread(&dir->first, sizeof(extptr_s), 1, fp);
    dir->first.logi_unit = swap16(dir->first.logi_unit);
    dir->first.index = swap16(dir->first.index);
    
    // Allocate and read name
    size_t name_size = F3S_NAME_ALIGN(dir->namelen);
    dir->name = malloc(name_size);
    fread(dir->name, name_size, 1, fp);
    dir->name[dir->namelen] = '\0';
    
    // Read stat structure
    read_stat(fp, &dir->stat);
    
    // Restore position
    fseek(fp, current, SEEK_SET);
}

void read_unit_info(FILE *fp, unit_info_s *ui) {
    fread(ui, sizeof(unit_info_s), 1, fp);
    ui->struct_size = swap16(ui->struct_size);
    ui->unit_pow2 = swap16(ui->unit_pow2);
    ui->reserve = swap16(ui->reserve);
    ui->erase_count = swap32(ui->erase_count);
    ui->boot.logi_unit = swap16(ui->boot.logi_unit);
    ui->boot.index = swap16(ui->boot.index);
}

void read_unit_logi(FILE *fp, unit_logi_s *ul) {
    fread(ul, sizeof(unit_logi_s), 1, fp);
    ul->struct_size = swap16(ul->struct_size);
    ul->logi = swap16(ul->logi);
    ul->age = swap32(ul->age);
    for(int i = 0; i < 4; i++) {
        ul->md5[i] = swap32(ul->md5[i]);
    }
}

void read_boot_info(FILE *fp, boot_info_s *bi) {
    fread(bi, sizeof(boot_info_s), 1, fp);
    bi->struct_size = swap16(bi->struct_size);
    bi->unit_index = swap16(bi->unit_index);
    bi->unit_total = swap16(bi->unit_total);
    bi->unit_spare = swap16(bi->unit_spare);
    bi->align_pow2 = swap16(bi->align_pow2);
    bi->root.logi_unit = swap16(bi->root.logi_unit);
    bi->root.index = swap16(bi->root.index);
}

void read_head(FILE *fp, head_s *hd) {
    fread(hd, sizeof(head_s), 1, fp);
    for(int i = 0; i < 3; i++) {
        hd->pad[i] = swap16(hd->pad[i]);
    }
    hd->text_offset_hi = swap16(hd->text_offset_hi);
    hd->text_offset_lo = swap16(hd->text_offset_lo);
    hd->text_size = swap16(hd->text_size);
    hd->next.logi_unit = swap16(hd->next.logi_unit);
    hd->next.index = swap16(hd->next.index);
    hd->super.logi_unit = swap16(hd->super.logi_unit);
    hd->super.index = swap16(hd->super.index);
}

void read_blocks(FILE *fp, blocks_s *blocks) {
    long block_top_pos = ftell(fp) - 40;

    // Skip padding
    fseek(fp, 0x20000 - 40, SEEK_CUR);

    read_unit_info(fp, &blocks->ui);
    read_unit_logi(fp, &blocks->ul);
    
    if (g_options.verbose) {
        print_unit_info("      ", &blocks->ui);
        print_unit_logi("      ", &blocks->ul);
    }
    
    long pos = ftell(fp);
    fseek(fp, -40-32, SEEK_CUR);
    
    // Initialize arrays
    blocks->heads = NULL;
    blocks->dirents = NULL;
    blocks->head_count = 0;
    
    while(1) {
        blocks->head_count++;
        blocks->heads = realloc(blocks->heads, blocks->head_count * sizeof(head_s));
        blocks->dirents = realloc(blocks->dirents, blocks->head_count * sizeof(dirent_s));
        
        // Read header
        read_head(fp, &blocks->heads[blocks->head_count - 1]);
        
        if (g_options.verbose) {
            printf("      Header %zu:\n", blocks->head_count);
            print_head("        ", &blocks->heads[blocks->head_count - 1]);
        }
        
        // Read corresponding dirent
        extent_type_e exttype = (blocks->heads[blocks->head_count - 1].status[0].flags2) & 0x03;
        if (exttype == EXTTYPE_DIR) {
            read_dirent(fp, &blocks->dirents[blocks->head_count - 1], block_top_pos,
                    blocks->heads[blocks->head_count - 1].text_offset_hi,
                    blocks->heads[blocks->head_count - 1].text_offset_lo);
            
            if (g_options.verbose) {
                print_dirent("        ", &blocks->dirents[blocks->head_count - 1]);
            }
        } else {
            // Initialize name to NULL for non-directory entries
            blocks->dirents[blocks->head_count - 1].name = NULL;
        }
        fseek(fp, -64, SEEK_CUR);
        
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
}

size_t read_partition(FILE *fp, part_s *part) {
    // Skip to boot info location
    fseek(fp, 40, SEEK_CUR);
//    read_unit_info(fp, &blocks->ui);
//    read_unit_logi(fp, &blocks->ul);
    read_boot_info(fp, &part->bi);
    fseek(fp, -24, SEEK_CUR);
    
    if (g_options.verbose) {
        printf("  Partition Boot Info:\n");
        print_boot_info("    ", &part->bi);
    }
    
    // Allocate and read blocks
    part->blocks = malloc(part->bi.unit_total * sizeof(blocks_s));
    for(int i = 0; i < part->bi.unit_total; i++) {
        if (g_options.verbose) {
            printf("    Logical Unit %d:\n", i + 1);
        }
        read_blocks(fp, &part->blocks[i]);
    }
    return part->bi.unit_total * (1 << part->blocks[0].ui.unit_pow2);
}

void free_partition(part_s *part) {
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
