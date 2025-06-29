/* Extract QNX IFS (Image File System) files
 *
 * A tool to extract files from QNX IFS format images.
 * Supports zlib, LZO, and UCL compression methods used in QNX embedded systems.
 *
 * Based on research of QNX IFS filesystem structures
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
#include <stdarg.h>
#include <libgen.h>
#include <utime.h>
#include <stdbool.h>
#include <limits.h>

#include <zlib.h>
#include <lzo/lzo1x.h>
#include <ucl/ucl.h>

#ifndef PATH_MAX
#define PATH_MAX 4096
#endif

// Global options structure like efsdump.c
typedef struct {
    char *target_dir;
    char *extract_file;
    char *ucompress_file;
    int extract_all;
    int list_only;
    int basename_only;
    int verbose;
    int show_md5;
    int zero_check_enabled;
    int help;
} options_t;

static options_t g_options = {
    .target_dir = ".",
    .extract_file = NULL,
    .ucompress_file = NULL,
    .extract_all = 0,
    .list_only = 0,
    .basename_only = 0,
    .verbose = 0,
    .show_md5 = 0,
    .zero_check_enabled = 1,
    .help = 0
};

// Constants and definitions from sys/ headers
#define STARTUP_HDR_SIGNATURE           0x00ff7eeb
#define STARTUP_HDR_FLAGS1_VIRTUAL      0x01
#define STARTUP_HDR_FLAGS1_BIGENDIAN    0x02
#define STARTUP_HDR_FLAGS1_COMPRESS_MASK    0x1c
#define STARTUP_HDR_FLAGS1_COMPRESS_SHIFT   0x02
#define STARTUP_HDR_FLAGS1_COMPRESS_NONE    0x00
#define STARTUP_HDR_FLAGS1_COMPRESS_ZLIB    0x04
#define STARTUP_HDR_FLAGS1_COMPRESS_LZO     0x08
#define STARTUP_HDR_FLAGS1_COMPRESS_UCL     0x0c

#define IMAGE_SIGNATURE     "imagefs"
#define IMAGE_FLAGS_BIGENDIAN   0x01
#define IMAGE_FLAGS_READONLY    0x02
#define IMAGE_FLAGS_INO_BITS    0x04

// File type constants
#ifndef S_IFMT
#define S_IFMT      0170000
#endif
#ifndef S_IFREG
#define S_IFREG     0100000
#endif
#ifndef S_IFDIR
#define S_IFDIR     0040000
#endif
#ifndef S_IFCHR
#define S_IFCHR     0020000
#endif
#ifndef S_IFBLK
#define S_IFBLK     0060000
#endif
#ifndef S_IFIFO
#define S_IFIFO     0010000
#endif
#ifndef S_IFLNK
#define S_IFLNK     0120000
#endif
#ifndef S_IFSOCK
#define S_IFSOCK    0140000
#endif
#ifndef S_IFNAM
#define S_IFNAM     0050000
#endif

// Endian conversion macros
#define ENDIAN_RET32(x) ((((x) >> 24) & 0xff) | \
                         (((x) >> 8) & 0xff00) | \
                         (((x) & 0xff00) << 8) | \
                         (((x) & 0xff) << 24))

#define ENDIAN_RET16(x) ((((x) >> 8) & 0xff) | \
                         (((x) & 0xff) << 8))

#define CROSSENDIAN(big) (big)

#pragma pack(push, 1)

// QNX startup header structure
struct startup_header {
    uint32_t signature;
    uint16_t version;
    uint8_t flags1;
    uint8_t flags2;
    uint16_t header_size;
    uint16_t machine;
    uint32_t startup_vaddr;
    uint32_t paddr_bias;
    uint32_t image_paddr;
    uint32_t ram_paddr;
    uint32_t ram_size;
    uint32_t startup_size;
    uint32_t stored_size;
    uint32_t imagefs_paddr;
    uint32_t imagefs_size;
    uint16_t preboot_size;
    uint16_t zero0;
    uint32_t zero[3];
    uint32_t info[48];
};

// QNX image header structure  
struct image_header {
    char signature[7];
    uint8_t flags;
    uint32_t image_size;
    uint32_t hdr_dir_size;
    uint32_t dir_offset;
    uint32_t boot_ino[4];
    uint32_t script_ino;
    uint32_t chain_paddr;
    uint32_t spare[10];
    uint32_t mountflags;
    char mountpoint[1];  // Variable length
};

// Image directory entry attributes
struct image_attr {
    uint16_t size;
    uint16_t extattr_offset;
    uint32_t ino;
    uint32_t mode;
    uint32_t gid;
    uint32_t uid;
    uint32_t mtime;
};

// Image file entry
struct image_file {
    struct image_attr attr;
    uint32_t offset;
    uint32_t size;
    char path[1];  // Variable length
};

// Image directory entry
struct image_dir {
    struct image_attr attr;
    char path[1];  // Variable length
};

// Image symlink entry
struct image_symlink {
    struct image_attr attr;
    uint16_t sym_offset;
    uint16_t sym_size;
    char path[1];  // Variable length
};

// Image device entry
struct image_device {
    struct image_attr attr;
    uint32_t dev;
    uint32_t rdev;
    char path[1];  // Variable length
};

// Image trailer
struct image_trailer {
    uint32_t cksum;
};

// Startup trailer
struct startup_trailer {
    uint32_t cksum;
};

#pragma pack(pop)

// Extract file tracking
struct extract_file {
    struct extract_file *next;
    char *path;
};

// Global state
static char *progname;
static char **check_files = NULL;
static int files_to_extract = 0;
static int files_left_to_extract = 0;
static int processing_done = 0;
static struct extract_file *extract_files = NULL;

// Function declarations
static void print_usage(const char *progname);
static int parse_arguments(int argc, char *argv[]);
static void error(int level, const char *fmt, ...) __attribute__((format(printf,2,3)));
static FILE* process(const char *file, FILE *fp);
static int find_pattern(FILE *fp, const unsigned char *p, int len, int count);
static int check_name(const char *name);
static int copy_bytes(FILE *from, FILE *to, int nbytes);
static int create_directories(const char *path);
static int zero_ok(struct startup_header *shdr);

static void display_shdr(int spos, struct startup_header *hdr);
static void display_ihdr(FILE *fp, int ipos, struct image_header *hdr);
static void display_attr(struct image_attr *attr);
static void process_file(FILE *fp, int ipos, struct image_file *ent);
static void process_dir(struct image_dir *ent);
static void process_symlink(struct image_symlink *ent);
static void process_device(struct image_device *ent);
static void extract_file(FILE *fp, int ipos, struct image_file *ent);
static void display_file(int ipos, struct image_file *ent);

// Endianness conversion functions like efsdump.c
uint16_t swap16(uint16_t val) {
    return ntohs(val);
}

uint32_t swap32(uint32_t val) {
    return ntohl(val);
}

// Error handling function
static void error(int level, const char *fmt, ...) {
    va_list ap;
    fprintf(stderr, "%s: ", progname);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
    if (level == 0) {
        exit(EXIT_FAILURE);
    }
}

// Print usage information like efsdump.c style
static void print_usage(const char *progname) {
    printf("QNX IFS (Image File System) Extractor\n");
    printf("Usage: %s [OPTIONS] <image_file> [files]\n\n", progname);
    printf("Options:\n");
    printf("  -d, --directory DIR    Extract to directory DIR (default: current directory)\n");
    printf("  -f, --file FILE        Extract named file\n");
    printf("  -u, --uncompress FILE  Save uncompressed image to FILE\n");
    printf("  -x, --extract          Extract all files\n");
    printf("  -l, --list             List contents without extracting\n");
    printf("  -b, --basename         Extract to basenames of files\n");
    printf("  -m, --md5              Display MD5 checksums\n");
    printf("  -z, --no-zero-check    Disable zero field validation\n");
    printf("  -v, --verbose          Enable verbose output\n");
    printf("  -h, --help             Show this help message\n\n");
    printf("Examples:\n");
    printf("  %s image.ifs                    # List image contents\n", progname);
    printf("  %s -x image.ifs                # Extract all files\n", progname);
    printf("  %s -d /tmp/extract -x image.ifs # Extract to directory\n", progname);
    printf("  %s -f filename image.ifs       # Extract specific file\n", progname);
}

// Parse command line arguments
static int parse_arguments(int argc, char *argv[]) {
    static struct option long_options[] = {
        {"directory",       required_argument, 0, 'd'},
        {"file",            required_argument, 0, 'f'},
        {"uncompress",      required_argument, 0, 'u'},
        {"extract",         no_argument,       0, 'x'},
        {"list",            no_argument,       0, 'l'},
        {"basename",        no_argument,       0, 'b'},
        {"md5",             no_argument,       0, 'm'},
        {"no-zero-check",   no_argument,       0, 'z'},
        {"verbose",         no_argument,       0, 'v'},
        {"help",            no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };
    
    int option_index = 0;
    int c;
    
    while ((c = getopt_long(argc, argv, "d:f:u:xlbmzvh", long_options, &option_index)) != -1) {
        switch (c) {
            case 'd':
                g_options.target_dir = optarg;
                g_options.extract_all = 1;
                break;
            case 'f': {
                struct extract_file *ef = malloc(sizeof(*ef));
                if (!ef) {
                    perror("Unable to allocate memory for file extraction record");
                    break;
                }
                ef->path = strdup(optarg);
                if (!ef->path) {
                    perror("Unable to record path");
                    free(ef);
                    break;
                }
                ef->next = extract_files;
                extract_files = ef;
                files_to_extract++;
                files_left_to_extract++;
                g_options.extract_all = 1;
                break;
            }
            case 'u':
                g_options.ucompress_file = optarg;
                break;
            case 'x':
                g_options.extract_all = 1;
                break;
            case 'l':
                g_options.list_only = 1;
                break;
            case 'b':
                g_options.basename_only = 1;
                break;
            case 'm':
                g_options.show_md5 = 1;
                break;
            case 'z':
                g_options.zero_check_enabled = 0;
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
        fprintf(stderr, "Error: Missing image file argument\n\n");
        print_usage(argv[0]);
        return -1;
    }
    
    return optind;  // Return index of the image file argument
}

// Find pattern in file like efsdump.c utility functions
static int find_pattern(FILE *fp, const unsigned char *p, int len, int count) {
    int c;
    int i = 0;
    int n = 0;

    while(i < len && (c = fgetc(fp)) != EOF) {
        n++;
        if(c == p[i]) {
            i++;
        } else {
            if(count != -1 && n >= count) {
                break;
            }
            i = (c == p[0]) ? 1 : 0;
        }
    }

    if(i < len) {
        return -1;
    }
    return (int)(ftell(fp) - i);
}

// Check if name matches filter list provided on command line
static int check_name(const char *name) {
    if(!check_files) {
        return 0; 
    }
    name = basename((char *)name);
    for(char **p = check_files; *p; p++) {
        if(strcmp(name, *p) == 0) {
            return 0;
        }
    }
    return -1;
}

// Copy bytes between files
static int copy_bytes(FILE *from, FILE *to, int nbytes) {
    char buff[4096];
    int remain = nbytes;
    while(remain > 0) {
        int chunk = (remain < (int)sizeof(buff)) ? remain : (int)sizeof(buff);
        if(fread(buff, 1, chunk, from) != (size_t)chunk) {
            return -1;
        }
        if(fwrite(buff, 1, chunk, to) != (size_t)chunk) {
            return -1;
        }
        remain -= chunk;
    }
    return 0;
}

// Create directories recursively like efsdump.c
static int create_directories(const char *path) {
    char dir_path[PATH_MAX];
    strncpy(dir_path, path, sizeof(dir_path) - 1);
    dir_path[sizeof(dir_path) - 1] = '\0';
    
    char *last_slash = strrchr(dir_path, '/');
    if (last_slash) {
        *last_slash = '\0';
        
        // Create parent directories first
        char *p = dir_path;
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

// Check if zero fields are indeed zero
static int zero_ok(struct startup_header *shdr) {
    return (shdr->zero[0] == 0 && shdr->zero[1] == 0 && shdr->zero[2] == 0);
}

// Display functions
static void display_shdr(int spos, struct startup_header *hdr) {
    if(check_name("Startup-header") != 0) {
        return;
    }
    printf(" 0x%08x 0x%08x  Startup-header flags1=%#x flags2=%#x paddr_bias=%#x\n",
           spos, hdr->header_size, hdr->flags1, hdr->flags2, hdr->paddr_bias);
    if(g_options.verbose) {
        printf("                        preboot_size=%#x\n", hdr->preboot_size);
        printf("                        image_paddr=%#x stored_size=%#x\n", hdr->image_paddr, hdr->stored_size);
        printf("                        startup_size=%#x imagefs_size=%#x\n", hdr->startup_size, hdr->imagefs_size);
        printf("                        ram_paddr=%#x ram_size=%#x\n", hdr->ram_paddr, hdr->ram_size);
        printf("                        startup_vaddr=%#x\n", hdr->startup_vaddr);
    }
}

static void display_ihdr(FILE *fp, int ipos, struct image_header *hdr) {
    if(check_name("Image-header") != 0) {
        return;
    }
    printf(" 0x%08x 0x%08lx  Image-header", ipos, (unsigned long)sizeof(*hdr));
    if(hdr->mountpoint[0]) {
        char buff[512];
        size_t toread = hdr->dir_offset - offsetof(struct image_header, mountpoint);
        if(toread > sizeof(buff)) {
            toread = sizeof(buff)-1;
        }
        fseek(fp, ipos + offsetof(struct image_header, mountpoint), SEEK_SET);
        if(fread(buff, 1, toread, fp) == toread) {
            buff[toread-1] = '\0';
            printf(" mountpoint=%s", buff);
        }
    }
    printf("\n");
    if(g_options.verbose) {
        printf("                       flags=%#x mountflags=%#x\n", hdr->flags, hdr->mountflags);
    }
}

static void display_attr(struct image_attr *attr) {
    printf("                        gid=%d uid=%d mode=%#o ino=%d mtime=%x\n",
           attr->gid, attr->uid, attr->mode & ~S_IFMT, attr->ino, attr->mtime);
}

static void process_dir(struct image_dir *ent) {
    const char *name = ent->path[0] ? ent->path : "Root-dirent";
    if(check_name(name) != 0) {
        return;
    }
    if (!g_options.list_only) {
        printf("       ----       ----  %s\n", name);
    }
    if(g_options.verbose) {
        display_attr(&ent->attr);
    }
}

static void process_symlink(struct image_symlink *ent) {
    if(check_name(ent->path) != 0) {
        return;
    }
    if (!g_options.list_only) {
        printf("       ---- 0x%08x  %s -> %s\n", ent->sym_size, ent->path, &ent->path[ent->sym_offset]);
    }
    if(g_options.verbose) {
        display_attr(&ent->attr);
    }
}

static void process_device(struct image_device *ent) {
    if(check_name(ent->path) != 0) {
        return;
    }
    if (!g_options.list_only) {
        printf("       ----       ----  %s dev=%d rdev=%d\n", ent->path, ent->dev, ent->rdev);
    }
    if(g_options.verbose) {
        display_attr(&ent->attr);
    }
}

static void extract_file(FILE *fp, int ipos, struct image_file *ent) {
    if(check_name(ent->path) != 0) {
        return;
    }

    const char *name = g_options.basename_only ? basename(ent->path) : ent->path;

    if (extract_files) {
        struct extract_file *ef;
        for (ef = extract_files; ef != NULL; ef = ef->next) {
            if (!strcmp(ef->path, name)) {
                break;
            }
        }
        if(!ef) {
            return;
        }
        if(--files_left_to_extract == 0) {
            processing_done = 1;
        }
    }

    // Ensure directory structure is created
    create_directories(name);

    FILE *dst = fopen(name, "wb");
    if(!dst) {
        error(1, "Unable to open %s: %s", name, strerror(errno));
        return;
    }

    fseek(fp, ipos + ent->offset, SEEK_SET);

    if(g_options.verbose) {
        printf("%s ... size %d\n", name, ent->size);
    }

    if(copy_bytes(fp, dst, ent->size) == -1) {
        fclose(dst);
        unlink(name);
        error(1, "Unable to create file %s: %s", name, strerror(errno));
        return;
    }

    fchmod(fileno(dst), ent->attr.mode & 07777);
    fchown(fileno(dst), ent->attr.uid, ent->attr.gid);
    fclose(dst);

    struct utimbuf buff;
    buff.actime = buff.modtime = ent->attr.mtime;
    utime(name, &buff);

    if(g_options.verbose) {
        printf("Extracted %s\n", name);
    }
}

static void display_file(int ipos, struct image_file *ent) {
    if(check_name(ent->path) != 0) {
        return;
    }
    printf(" 0x%08x 0x%08x  %s", ipos + ent->offset, ent->size, ent->path);
    if (g_options.show_md5) {
        printf(" MD5 Not available");
    }
    printf("\n");
    if(g_options.verbose) {
        display_attr(&ent->attr);
    }
}

static void process_file(FILE *fp, int ipos, struct image_file *ent) {
    if(g_options.extract_all) {
        extract_file(fp, ipos, ent);
    }
    if(!g_options.extract_all || g_options.verbose) {
        display_file(ipos, ent);
    }
}

// Main processing function
static FILE* process(const char *file, FILE *fp) {
    struct startup_header shdr = {0};
    shdr.signature = STARTUP_HDR_SIGNATURE;
    struct image_header ihdr;
    int spos = -1;
    int ipos;
    int dpos;
    static char buf[0x10000];
    static char out_buf[0x10000];

    // Initialize image header with signature
    strcpy(ihdr.signature, IMAGE_SIGNATURE);

    // Attempt to find IMAGE_SIGNATURE directly
    ipos = find_pattern(fp, (unsigned char*)ihdr.signature, sizeof(ihdr.signature), 0);
    if(ipos == -1) {
        rewind(fp);

        // Find startup signature and verify
        while(1) {
            spos = find_pattern(fp, (unsigned char *)&shdr.signature, sizeof(shdr.signature), -1);
            if(spos == -1) {
                // Try endian swapped signature
                uint32_t swapped = ENDIAN_RET32(shdr.signature);
                shdr.signature = swapped;
                rewind(fp);
                spos = find_pattern(fp, (unsigned char *)&shdr.signature, sizeof(shdr.signature), -1);
                if(spos == -1) {
                    error(1, "Unable to find startup header in %s", file);
                    return fp;
                }
            }

            if(fread((char *)&shdr + sizeof(shdr.signature), sizeof(shdr)-sizeof(shdr.signature), 1, fp) != 1) {
                error(1, "Unable to read image %s", file);
                return fp;
            }

            // Check zero fields
            if(!zero_ok(&shdr)) {
                if(g_options.zero_check_enabled) {
                    continue; // try next match
                } else {
                    if(g_options.verbose) {
                        printf("Warning: Non-zero data in zero fields ignored\n");
                    }
                }
            }
            break;
        }

        if (!g_options.list_only) {
            printf("Image startup_size: %d (0x%x)\n", shdr.startup_size, shdr.startup_size);
            printf("Image stored_size: %d (0x%x)\n", shdr.stored_size, shdr.stored_size);
            printf("Compressed size: %lu (0x%lx)\n",
                   (unsigned long)(shdr.stored_size - shdr.startup_size - sizeof(struct image_trailer)),
                   (unsigned long)(shdr.stored_size - shdr.startup_size - sizeof(struct image_trailer)));
        }

        // Handle compression if needed
        if((shdr.flags1 & STARTUP_HDR_FLAGS1_COMPRESS_MASK) != 0) {
            if(g_options.verbose) {
                printf("DEBUG: Compression detected, flags1=0x%x, mask=0x%x\n", 
                       shdr.flags1, shdr.flags1 & STARTUP_HDR_FLAGS1_COMPRESS_MASK);
            }
            FILE *fp2;
            int n;

            // Create file to hold uncompressed image
            if(g_options.ucompress_file) {
                fp2 = fopen(g_options.ucompress_file, "w+b");
            } else {
                fp2 = tmpfile();
            }

            if(!fp2) {
                error(1, "Unable to create file for uncompressed image");
                return fp;
            }

            // Copy non-compressed part
            rewind(fp);
            if(CROSSENDIAN(shdr.flags1 & STARTUP_HDR_FLAGS1_BIGENDIAN)) {
                for(n = 0; n < spos + (int)ENDIAN_RET32(shdr.startup_size); ++n) {
                    int c = getc(fp);
                    if(c == EOF) break;
                    putc(c, fp2);
                }
            } else {
                for(n = 0; n < spos + (int)shdr.startup_size; ++n) {
                    int c = getc(fp);
                    if(c == EOF) break;
                    putc(c, fp2);
                }
            }
            fflush(fp2);

            // Decompress compressed part
            if(g_options.verbose) {
                printf("DEBUG: Starting decompression, type=0x%x\n", shdr.flags1 & STARTUP_HDR_FLAGS1_COMPRESS_MASK);
            }
            switch(shdr.flags1 & STARTUP_HDR_FLAGS1_COMPRESS_MASK) {
            case STARTUP_HDR_FLAGS1_COMPRESS_ZLIB: {
                int fd = fileno(fp);
                off_t curpos = ftell(fp);
                lseek(fd, curpos, SEEK_SET);
                gzFile zin = gzdopen(fd, "rb");
                if(!zin) {
                    error(1, "Unable to open zlib stream");
                    fclose(fp2);
                    return fp;
                }
                while((n = gzread(zin, buf, sizeof(buf))) > 0) {
                    fwrite(buf, 1, n, fp2);
                }
                // Can't gzclose(zin) safely since it closes fd
                break;
            }
            case STARTUP_HDR_FLAGS1_COMPRESS_LZO: {
                if(lzo_init() != LZO_E_OK) {
                    error(1, "LZO init failure");
                    fclose(fp2);
                    return fp;
                }
                for(;;) {
                    int c1 = getc(fp);
                    int c2 = getc(fp);
                    if(c1 == EOF || c2 == EOF) break;
                    unsigned len = ((unsigned)c1 << 8) | c2;
                    if(len == 0) break;
                    if(fread(buf, 1, len, fp) != len) {
                        error(1, "LZO read failure");
                        fclose(fp2);
                        return fp;
                    }
                    lzo_uint out_len;
                    int status = lzo1x_decompress((unsigned char*)buf, len, (unsigned char*)out_buf, &out_len, NULL);
                    if(status != LZO_E_OK) {
                        error(1, "LZO decompression failure");
                        fclose(fp2);
                        return fp;
                    }
                    fwrite(out_buf, 1, out_len, fp2);
                }
                break;
            }
            case STARTUP_HDR_FLAGS1_COMPRESS_UCL: {
                for(;;) {
                    int c1 = getc(fp);
                    int c2 = getc(fp);
                    if(c1 == EOF || c2 == EOF) break;
                    unsigned len = ((unsigned)c1 << 8) | c2;
                    if(len == 0) break;
                    if(fread(buf, 1, len, fp) != len) {
                        error(1, "UCL read failure");
                        fclose(fp2);
                        return fp;
                    }
                    ucl_uint out_len;
                    int status = ucl_nrv2b_decompress_8((unsigned char*)buf, len, (unsigned char*)out_buf, &out_len, NULL);
                    if(status != UCL_E_OK) {
                        error(1, "UCL decompression failure");
                        fclose(fp2);
                        return fp;
                    }
                    fwrite(out_buf, 1, out_len, fp2);
                }
                break;
            }
            default:
                error(1, "Unsupported compression type.");
                fclose(fp2);
                return fp;
            }

            fclose(fp);
            fp = fp2;
            rewind(fp2);
            if(g_options.verbose) {
                printf("DEBUG: Decompression complete, switched to decompressed file\n");
            }
        }

        if(CROSSENDIAN(shdr.flags1 & STARTUP_HDR_FLAGS1_BIGENDIAN)) {
            // Convert fields from big-endian
            shdr.version      = ENDIAN_RET16(shdr.version);
            shdr.machine      = ENDIAN_RET16(shdr.machine);
            shdr.header_size  = ENDIAN_RET16(shdr.header_size);
            shdr.preboot_size = ENDIAN_RET16(shdr.preboot_size);
            shdr.startup_vaddr= ENDIAN_RET32(shdr.startup_vaddr);
            shdr.paddr_bias   = ENDIAN_RET32(shdr.paddr_bias);
            shdr.image_paddr  = ENDIAN_RET32(shdr.image_paddr);
            shdr.ram_paddr    = ENDIAN_RET32(shdr.ram_paddr);
            shdr.ram_size     = ENDIAN_RET32(shdr.ram_size);
            shdr.startup_size = ENDIAN_RET32(shdr.startup_size);
            shdr.stored_size  = ENDIAN_RET32(shdr.stored_size);
            shdr.imagefs_paddr= ENDIAN_RET32(shdr.imagefs_paddr);
            shdr.imagefs_size = ENDIAN_RET32(shdr.imagefs_size);
            shdr.zero0        = ENDIAN_RET32(shdr.zero0);
            shdr.zero[0]      = ENDIAN_RET32(shdr.zero[0]);
            shdr.zero[1]      = ENDIAN_RET32(shdr.zero[1]);
            shdr.zero[2]      = ENDIAN_RET32(shdr.zero[2]);
        }

        if (!g_options.list_only && g_options.verbose) {
            printf("signature:     0x%08x\n", shdr.signature);
            printf("version:       0x%08x\n", shdr.version);
            printf("flags1:        0x%08x\n", shdr.flags1);
            printf("flags2:        0x%08x\n", shdr.flags2);
            printf("header_size:   0x%08x\n", shdr.header_size);
            printf("machine:       0x%08x\n", shdr.machine);
            printf("startup_vaddr: 0x%08x\n", shdr.startup_vaddr);
            printf("paddr_bias:    0x%08x\n", shdr.paddr_bias);
            printf("image_paddr:   0x%08x\n", shdr.image_paddr);
            printf("ram_paddr:     0x%08x\n", shdr.ram_paddr);
            printf("ram_size:      0x%08x\n", shdr.ram_size);
            printf("startup_size:  0x%08x\n", shdr.startup_size);
            printf("stored_size:   0x%08x\n", shdr.stored_size);
            printf("imagefs_paddr: 0x%08x\n", shdr.imagefs_paddr);
            printf("imagefs_size:  0x%08x\n", shdr.imagefs_size);
            printf("preboot_size:  0x%08x\n", shdr.preboot_size);
            printf("zero0:         0x%08x\n", shdr.zero0);
        }

        if((shdr.flags1 & STARTUP_HDR_FLAGS1_COMPRESS_MASK) != 0) {
            // After decompression, search from the startup end for the image header
            fseek(fp, shdr.startup_size, SEEK_SET);
            ipos = find_pattern(fp, (unsigned char*)ihdr.signature, sizeof(ihdr.signature), -1);
        } else {
            fseek(fp, spos + shdr.startup_size, SEEK_SET);
            ipos = find_pattern(fp, (unsigned char*)ihdr.signature, sizeof(ihdr.signature), -1);
        }
        if(ipos == -1) {
            error(1, "Unable to find image header in %s", file);
            return fp;
        }
    }

    // Read image header
    fseek(fp, ipos, SEEK_SET);
    if(g_options.verbose) {
        printf("DEBUG: Reading image header from position 0x%x\n", ipos);
    }
    if(fread((char *)&ihdr, sizeof(ihdr), 1, fp) != 1) {
        error(1, "Unable to read image %s", file);
        return fp;
    }
    if(g_options.verbose) {
        printf("DEBUG: Read ihdr, first few bytes: %02x %02x %02x %02x\n", 
               *(unsigned char*)&ihdr.flags,
               *(unsigned char*)&ihdr.image_size,
               *((unsigned char*)&ihdr.image_size + 1),
               *((unsigned char*)&ihdr.image_size + 2));
    }

    if(CROSSENDIAN(ihdr.flags & IMAGE_FLAGS_BIGENDIAN)) {
        // Convert image header fields
        uint32_t *p = (uint32_t *)&ihdr.image_size;
        for(; (unsigned char*)p < (unsigned char*)&ihdr + offsetof(struct image_header, mountpoint); p++) {
            *p = ENDIAN_RET32(*p);
        }
    }

    dpos = ipos + ihdr.dir_offset;

    if(g_options.verbose) {
        printf("DEBUG: ipos=0x%x, ihdr.dir_offset=0x%x, dpos=0x%x\n", ipos, ihdr.dir_offset, dpos);
        printf("DEBUG: ihdr.image_size=0x%x, ihdr.hdr_dir_size=0x%x\n", ihdr.image_size, ihdr.hdr_dir_size);
        printf("DEBUG: ihdr.flags=0x%x\n", ihdr.flags);
    }

    if(!g_options.list_only) {
        printf("     Offset       Size  Name\n");
        if(spos != -1) {
            if(spos > 0 && check_name("*.boot") == 0) {
                printf(" 0x%08x 0x%08x  *.boot\n", 0, spos);
            }
            display_shdr(spos, &shdr);
            if(check_name("startup.*") == 0) {
                printf(" 0x%08lx 0x%08lx  startup.*\n", (unsigned long)(spos + sizeof(shdr)),
                       (unsigned long)(shdr.startup_size - sizeof(shdr)));
            }
        }
        display_ihdr(fp, ipos, &ihdr);
        if(check_name("Image-directory") == 0) {
            printf(" 0x%08x 0x%08x  Image-directory\n", dpos, ihdr.hdr_dir_size - ihdr.dir_offset);
        }
    }

    while(!processing_done) {
        char buff[1024];
        union image_dirent {
            struct image_attr attr;
            struct image_file file;
            struct image_dir dir;
            struct image_symlink symlink;
            struct image_device device;
        } *dir = (union image_dirent *)buff;

        fseek(fp, dpos, SEEK_SET);
        if(fread(&dir->attr, sizeof(dir->attr), 1, fp) != 1) {
            error(1, "Early end reading directory");
            break;
        }

        if(CROSSENDIAN(ihdr.flags & IMAGE_FLAGS_BIGENDIAN)) {
            dir->attr.size = ENDIAN_RET16(dir->attr.size);
            dir->attr.extattr_offset = ENDIAN_RET16(dir->attr.extattr_offset);

            uint32_t *p = &dir->attr.ino;
            for(; (unsigned char*)p < (unsigned char*)(&dir->attr + 1); p++) {
                *p = ENDIAN_RET32(*p);
            }
        }

        if(dir->attr.size < sizeof(dir->attr)) {
            if(dir->attr.size != 0) {
                error(1, "Invalid dir entry");
            }
            break;
        }
        if(dir->attr.size > sizeof(dir->attr)) {
            size_t extra = dir->attr.size - sizeof(dir->attr);
            if(extra > sizeof(buff)-sizeof(dir->attr)) {
                error(1, "Directory entry too large");
                break;
            }
            if(fread((char*)dir + sizeof(dir->attr), extra, 1, fp) != 1) {
                error(1, "Error reading directory");
                break;
            }
        }
        dpos += dir->attr.size;

        switch(dir->attr.mode & S_IFMT) {
        case S_IFREG:
            if(CROSSENDIAN(ihdr.flags & IMAGE_FLAGS_BIGENDIAN)) {
                dir->file.offset = ENDIAN_RET32(dir->file.offset);
                dir->file.size = ENDIAN_RET32(dir->file.size);
            }
            process_file(fp, ipos, &dir->file);
            break;
        case S_IFDIR:
            process_dir(&dir->dir);
            break;
        case S_IFLNK:
            if(CROSSENDIAN(ihdr.flags & IMAGE_FLAGS_BIGENDIAN)) {
                dir->symlink.sym_offset = ENDIAN_RET16(dir->symlink.sym_offset);
                dir->symlink.sym_size = ENDIAN_RET16(dir->symlink.sym_size);
            }
            process_symlink(&dir->symlink);
            break;
        case S_IFCHR:
        case S_IFBLK:
        case S_IFIFO:
        case S_IFNAM:
            if(CROSSENDIAN(ihdr.flags & IMAGE_FLAGS_BIGENDIAN)) {
                dir->device.dev = ENDIAN_RET32(dir->device.dev);
                dir->device.rdev = ENDIAN_RET32(dir->device.rdev);
            }
            process_device(&dir->device);
            break;
        default:
            error(1, "Unknown file type in IFS directory");
            break;
        }
    }

    if(!g_options.list_only) {
        struct image_trailer itlr;
        struct startup_trailer stlr;

        fseek(fp, ipos + ihdr.image_size - sizeof(itlr), SEEK_SET);
        if(fread(&itlr, sizeof(itlr), 1, fp) != 1) {
            error(1, "Early end reading image trailer");
            return fp;
        }

        printf("Checksums: image=%#x", CROSSENDIAN(shdr.flags1 & STARTUP_HDR_FLAGS1_BIGENDIAN) ? ENDIAN_RET32(itlr.cksum) : itlr.cksum);

        if(spos != -1) {
            fseek(fp, spos + shdr.startup_size - sizeof(stlr), SEEK_SET);
            if(fread(&stlr, sizeof(stlr), 1, fp) != 1) {
                printf("\n");
                error(1, "Early end reading startup trailer");
                return fp;
            }
            printf(" startup=%#lx", (unsigned long)(CROSSENDIAN(shdr.flags1 & STARTUP_HDR_FLAGS1_BIGENDIAN) ? ENDIAN_RET32(stlr.cksum) : stlr.cksum));
        }
        printf("\n");
    }
    return fp;
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
    
    progname = basename(argv[0]);
    const char *image_file = argv[image_arg_index];
    
    if(image_arg_index < argc - 1) {
        check_files = &argv[image_arg_index + 1];
    }

    // Open input file
    FILE *fp = NULL;
    if(strcmp(image_file, "-") != 0) {
        fp = fopen(image_file, "rb");
        if(!fp) {
            error(0, "Unable to open file %s - %s", image_file, strerror(errno));
        }
    } else if(isatty(fileno(stdin))) {
        error(0, "Must have an image file (cannot read from tty)");
    } else {
        fp = stdin;
        image_file = "-- stdin --";
    }

    // Change to target directory if specified
    if (strcmp(g_options.target_dir, ".") != 0) {
        if (create_directories(g_options.target_dir) != 0) {
            error(0, "Cannot create directory %s to extract %s", g_options.target_dir, image_file);
        }
        if (chdir(g_options.target_dir) != 0) {
            error(0, "Cannot cd to directory %s to extract %s", g_options.target_dir, image_file);
        }
        if (g_options.verbose) {
            printf("Changed to target directory: %s\n", g_options.target_dir);
        }
    }

    // Process the image
    FILE *result_fp = process(image_file, fp);

    if (result_fp && result_fp != stdin) {
        fclose(result_fp);
    }

    return 0;
}
