/*
 * Decompress QNX compressed-ELF files.
 *
 * These files start with the magic "iwlyfmbp" followed by a little-endian
 * deflate header and a stream of little-endian compressed block headers.
 */

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <lzo/minilzo.h>
#include <ucl/ucl.h>

#define CMP_LZO 0
#define CMP_UCL 1
#define QNX_MAGIC "iwlyfmbp"
#define QNX_MAGIC_LEN 8

#pragma pack(push, 1)
typedef struct {
    uint8_t signature[8];
    uint32_t usize;
    uint16_t blksize;
    uint8_t cmptype;
    uint8_t flags;
} qnx_deflate_filehdr_t;

typedef struct {
    uint16_t prev;
    uint16_t next;
    uint16_t pusize;
    uint16_t usize;
} qnx_cmphdr_t;
#pragma pack(pop)

typedef struct {
    const char *input_path;
    const char *output_path;
    int verbose;
} options_t;

static uint16_t read_le16(uint16_t val) {
    uint8_t *bytes = (uint8_t *)&val;
    return (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
}

static uint32_t read_le32(uint32_t val) {
    uint8_t *bytes = (uint8_t *)&val;
    return (uint32_t)bytes[0] |
           ((uint32_t)bytes[1] << 8) |
           ((uint32_t)bytes[2] << 16) |
           ((uint32_t)bytes[3] << 24);
}

static void print_usage(const char *progname) {
    printf("QNX compressed-ELF decompressor\n");
    printf("Usage: %s [OPTIONS] <input> [output]\n\n", progname);
    printf("Options:\n");
    printf("  -o, --output FILE   Write decompressed ELF to FILE\n");
    printf("  -v, --verbose       Print block-level details\n");
    printf("  -h, --help          Show this help message\n\n");
    printf("Examples:\n");
    printf("  %s test_elf/app app.elf\n", progname);
    printf("  %s -o ntpd.elf test_elf/ntpd\n", progname);
}

static int parse_arguments(int argc, char **argv, options_t *options) {
    static struct option long_options[] = {
        {"output", required_argument, 0, 'o'},
        {"verbose", no_argument, 0, 'v'},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}
    };

    memset(options, 0, sizeof(*options));

    int c;
    while ((c = getopt_long(argc, argv, "o:vh", long_options, NULL)) != -1) {
        switch (c) {
            case 'o':
                options->output_path = optarg;
                break;
            case 'v':
                options->verbose = 1;
                break;
            case 'h':
                print_usage(argv[0]);
                return 1;
            default:
                return -1;
        }
    }

    if (optind >= argc) {
        fprintf(stderr, "Error: missing input file\n\n");
        print_usage(argv[0]);
        return -1;
    }

    options->input_path = argv[optind++];
    if (optind < argc) {
        if (options->output_path) {
            fprintf(stderr, "Error: output specified both with -o and positional argument\n");
            return -1;
        }
        options->output_path = argv[optind++];
    }

    if (optind < argc) {
        fprintf(stderr, "Error: too many arguments\n");
        return -1;
    }

    if (!options->output_path) {
        fprintf(stderr, "Error: missing output file\n\n");
        print_usage(argv[0]);
        return -1;
    }

    return 0;
}

static int read_exact(FILE *fp, void *buffer, size_t size, const char *what) {
    if (fread(buffer, 1, size, fp) != size) {
        fprintf(stderr, "Failed to read %s\n", what);
        return -1;
    }
    return 0;
}

static int write_exact(int fd, const void *buffer, size_t size, const char *path) {
    const uint8_t *pos = buffer;
    size_t remaining = size;

    while (remaining > 0) {
        ssize_t written = write(fd, pos, remaining);
        if (written < 0) {
            fprintf(stderr, "Failed to write %s: %s\n", path, strerror(errno));
            return -1;
        }
        pos += written;
        remaining -= written;
    }

    return 0;
}

static int decompress_qnx_elf(FILE *in, int outfd, const char *output_path, int verbose) {
    qnx_deflate_filehdr_t defhdr;
    if (read_exact(in, &defhdr, sizeof(defhdr), "compression header") != 0) {
        return -1;
    }

    defhdr.usize = read_le32(defhdr.usize);
    defhdr.blksize = read_le16(defhdr.blksize);

    if (memcmp(defhdr.signature, QNX_MAGIC, QNX_MAGIC_LEN) != 0 ||
        (defhdr.cmptype != CMP_LZO && defhdr.cmptype != CMP_UCL) ||
        defhdr.blksize < 16) {
        fprintf(stderr, "Invalid QNX compressed-ELF header (size=%u, blksize=%u, compression=%u)\n",
                defhdr.usize, defhdr.blksize, defhdr.cmptype);
        return -1;
    }

    if (verbose) {
        printf("Decompressing: size=%u, blksize=%u, compression=%s\n",
               defhdr.usize, defhdr.blksize, defhdr.cmptype == CMP_LZO ? "LZO" : "UCL");
    }

    size_t overhead = defhdr.blksize / 8 + 256;
    size_t buf_len = defhdr.blksize + overhead;
    uint8_t *workbuf = malloc(buf_len * 2);
    if (!workbuf) {
        fprintf(stderr, "Failed to allocate work buffer\n");
        return -1;
    }

    size_t total_written = 0;
    int result = 0;

    for (int block_num = 0; total_written < defhdr.usize; block_num++) {
        qnx_cmphdr_t blockhdr;
        if (read_exact(in, &blockhdr, sizeof(blockhdr), "block header") != 0) {
            result = -1;
            break;
        }

        blockhdr.prev = read_le16(blockhdr.prev);
        blockhdr.next = read_le16(blockhdr.next);
        blockhdr.pusize = read_le16(blockhdr.pusize);
        blockhdr.usize = read_le16(blockhdr.usize);

        if (blockhdr.next == 0) {
            break;
        }
        if (blockhdr.next < sizeof(qnx_cmphdr_t)) {
            fprintf(stderr, "Invalid block %d: next=%u\n", block_num, blockhdr.next);
            result = -1;
            break;
        }

        uint16_t in_len = blockhdr.next - sizeof(qnx_cmphdr_t);
        if (in_len > buf_len || blockhdr.usize > defhdr.blksize) {
            fprintf(stderr, "Invalid block %d: in_len=%u, usize=%u\n",
                    block_num, in_len, blockhdr.usize);
            result = -1;
            break;
        }

        uint8_t *in_buf = workbuf + buf_len - in_len;
        uint8_t *out_buf = workbuf;
        if (read_exact(in, in_buf, in_len, "compressed block data") != 0) {
            result = -1;
            break;
        }

        uint16_t actual_out_len = 0;
        if (defhdr.cmptype == CMP_LZO) {
            lzo_uint out_len = defhdr.blksize;
            int error = lzo1x_decompress_safe(in_buf, in_len, out_buf, &out_len, NULL);
            if (error != LZO_E_OK) {
                fprintf(stderr, "LZO decompression error %d at block %d\n", error, block_num);
                result = -1;
                break;
            }
            actual_out_len = out_len;
        } else {
            ucl_uint out_len = defhdr.blksize;
            int error = ucl_nrv2b_decompress_safe_8(in_buf, in_len, out_buf, &out_len, NULL);
            if (error != UCL_E_OK) {
                fprintf(stderr, "UCL decompression error %d at block %d\n", error, block_num);
                result = -1;
                break;
            }
            actual_out_len = out_len;
        }

        size_t remaining = defhdr.usize - total_written;
        size_t to_write = actual_out_len < remaining ? actual_out_len : remaining;
        if (write_exact(outfd, out_buf, to_write, output_path) != 0) {
            result = -1;
            break;
        }
        total_written += to_write;

        if (verbose) {
            printf("  Block %d: in=%u out=%u total=%zu/%u\n",
                   block_num, in_len, actual_out_len, total_written, defhdr.usize);
        }
    }

    if (result == 0 && total_written != defhdr.usize) {
        fprintf(stderr, "Incomplete output: expected %u bytes, wrote %zu bytes\n",
                defhdr.usize, total_written);
        result = -1;
    }

    free(workbuf);
    return result;
}

int main(int argc, char **argv) {
    options_t options;
    int parse_result = parse_arguments(argc, argv, &options);
    if (parse_result > 0) {
        return 0;
    }
    if (parse_result < 0) {
        return 1;
    }

    if (lzo_init() != LZO_E_OK) {
        fprintf(stderr, "Failed to initialize LZO\n");
        return 1;
    }
    if (ucl_init() != UCL_E_OK) {
        fprintf(stderr, "Failed to initialize UCL\n");
        return 1;
    }

    FILE *in = fopen(options.input_path, "rb");
    if (!in) {
        fprintf(stderr, "Failed to open %s: %s\n", options.input_path, strerror(errno));
        return 1;
    }

    int outfd = open(options.output_path, O_WRONLY | O_CREAT | O_TRUNC, 0755);
    if (outfd < 0) {
        fprintf(stderr, "Failed to create %s: %s\n", options.output_path, strerror(errno));
        fclose(in);
        return 1;
    }

    int result = decompress_qnx_elf(in, outfd, options.output_path, options.verbose);

    close(outfd);
    fclose(in);

    return result == 0 ? 0 : 1;
}
