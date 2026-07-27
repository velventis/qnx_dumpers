/* Create QNX "iwlyfmbp" deflate files using UCL NRV2B compression. */

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ucl/ucl.h>
#include <unistd.h>

#define CMP_UCL 1
#define DEFAULT_BLOCK_SIZE 8192
#define QNX_DEFLATE_MAGIC "iwlyfmbp"

static void usage(const char *argv0)
{
    printf("Usage: %s [-o OUTPUT] [-b BLOCKSIZE] INPUT\n", argv0);
    printf("\n");
    printf("Create a QNX compressed file stream compatible with efsdump.\n");
    printf("Defaults: block size %u, output stdout.\n", DEFAULT_BLOCK_SIZE);
}

static void put_le16(uint8_t *p, uint16_t value)
{
    p[0] = value & 0xff;
    p[1] = (value >> 8) & 0xff;
}

static void put_le32(uint8_t *p, uint32_t value)
{
    p[0] = value & 0xff;
    p[1] = (value >> 8) & 0xff;
    p[2] = (value >> 16) & 0xff;
    p[3] = (value >> 24) & 0xff;
}

static int write_all(int fd, const void *buf, size_t len)
{
    const uint8_t *p = buf;

    while (len > 0) {
        ssize_t written = write(fd, p, len);
        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        if (written == 0) {
            errno = EIO;
            return -1;
        }
        p += written;
        len -= (size_t)written;
    }

    return 0;
}

static int read_all_file(const char *path, uint8_t **data, size_t *len)
{
    FILE *fp = fopen(path, "rb");
    long size;
    uint8_t *buf;

    if (!fp) {
        return -1;
    }
    if (fseek(fp, 0, SEEK_END) != 0) {
        fclose(fp);
        return -1;
    }
    size = ftell(fp);
    if (size < 0) {
        fclose(fp);
        return -1;
    }
    if (fseek(fp, 0, SEEK_SET) != 0) {
        fclose(fp);
        return -1;
    }

    buf = malloc((size_t)size ? (size_t)size : 1);
    if (!buf) {
        fclose(fp);
        errno = ENOMEM;
        return -1;
    }
    if (size > 0 && fread(buf, 1, (size_t)size, fp) != (size_t)size) {
        int had_error = ferror(fp);

        free(buf);
        fclose(fp);
        errno = had_error ? errno : EIO;
        return -1;
    }
    if (fclose(fp) != 0) {
        free(buf);
        return -1;
    }

    *data = buf;
    *len = (size_t)size;
    return 0;
}

static int deflate_file(const uint8_t *input, size_t input_len,
                        uint16_t block_size, int outfd)
{
    uint8_t header[16];
    size_t offset = 0;
    uint16_t prev_next = 0;
    uint16_t prev_usize = 0;

    if (input_len > UINT32_MAX) {
        fprintf(stderr, "Input too large for QNX deflate header\n");
        return -1;
    }

    memcpy(header, QNX_DEFLATE_MAGIC, 8);
    put_le32(header + 8, (uint32_t)input_len);
    put_le16(header + 12, block_size);
    header[14] = CMP_UCL;
    header[15] = 0;
    if (write_all(outfd, header, sizeof(header)) != 0) {
        perror("write header");
        return -1;
    }

    while (offset < input_len || (input_len == 0 && offset == 0)) {
        size_t chunk_len = input_len - offset;
        size_t work_len;
        uint8_t *compressed;
        ucl_uint compressed_len;
        uint8_t block_header[8];
        uint16_t next;
        int rc;

        if (chunk_len > block_size) {
            chunk_len = block_size;
        }

        work_len = chunk_len + chunk_len / 8 + 256;
        compressed = malloc(work_len ? work_len : 1);
        if (!compressed) {
            perror("malloc");
            return -1;
        }

        compressed_len = (ucl_uint)work_len;
        rc = ucl_nrv2b_99_compress(input + offset, (ucl_uint)chunk_len,
                                   compressed, &compressed_len, NULL, 10,
                                   NULL, NULL);
        if (rc != UCL_E_OK) {
            fprintf(stderr, "UCL compression failed: %d\n", rc);
            free(compressed);
            return -1;
        }
        if (compressed_len + sizeof(block_header) > UINT16_MAX ||
            chunk_len > UINT16_MAX) {
            fprintf(stderr, "Compressed block too large\n");
            free(compressed);
            return -1;
        }

        next = (uint16_t)(compressed_len + sizeof(block_header));
        put_le16(block_header + 0, prev_next);
        put_le16(block_header + 2, next);
        put_le16(block_header + 4, prev_usize);
        put_le16(block_header + 6, (uint16_t)chunk_len);
        if (write_all(outfd, block_header, sizeof(block_header)) != 0 ||
            write_all(outfd, compressed, compressed_len) != 0) {
            perror("write block");
            free(compressed);
            return -1;
        }

        free(compressed);
        offset += chunk_len;
        prev_next = next;
        prev_usize = (uint16_t)chunk_len;
        if (input_len == 0) {
            break;
        }
    }

    return 0;
}

int main(int argc, char **argv)
{
    const char *output_path = NULL;
    const char *input_path;
    uint8_t *input = NULL;
    size_t input_len = 0;
    uint16_t block_size = DEFAULT_BLOCK_SIZE;
    int outfd = STDOUT_FILENO;
    int opt;
    int rc;

    static const struct option long_options[] = {
        {"output", required_argument, NULL, 'o'},
        {"block-size", required_argument, NULL, 'b'},
        {"help", no_argument, NULL, 'h'},
        {NULL, 0, NULL, 0},
    };

    while ((opt = getopt_long(argc, argv, "o:b:h", long_options, NULL)) != -1) {
        switch (opt) {
        case 'o':
            output_path = optarg;
            break;
        case 'b': {
            char *end = NULL;
            unsigned long value = strtoul(optarg, &end, 0);
            if (!end || *end || value < 16 || value > UINT16_MAX) {
                fprintf(stderr, "Invalid block size: %s\n", optarg);
                return 1;
            }
            block_size = (uint16_t)value;
            break;
        }
        case 'h':
            usage(argv[0]);
            return 0;
        default:
            usage(argv[0]);
            return 1;
        }
    }

    if (optind + 1 != argc) {
        usage(argv[0]);
        return 1;
    }
    input_path = argv[optind];

    if (ucl_init() != UCL_E_OK) {
        fprintf(stderr, "ucl_init failed\n");
        return 1;
    }
    if (read_all_file(input_path, &input, &input_len) != 0) {
        fprintf(stderr, "Failed to read %s: %s\n", input_path, strerror(errno));
        return 1;
    }

    if (output_path) {
        outfd = open(output_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (outfd < 0) {
            fprintf(stderr, "Failed to open %s: %s\n", output_path,
                    strerror(errno));
            free(input);
            return 1;
        }
    }

    rc = deflate_file(input, input_len, block_size, outfd);
    if (output_path && close(outfd) != 0) {
        fprintf(stderr, "Failed to close %s: %s\n", output_path,
                strerror(errno));
        rc = -1;
    }

    free(input);
    return rc == 0 ? 0 : 1;
}
