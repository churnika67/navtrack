/* Fixed-capacity byte ring buffer, as used for a UART receive queue. */
#ifndef NAVTRACK_RING_BUFFER_H
#define NAVTRACK_RING_BUFFER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t *storage;
    size_t capacity;
    size_t head;  /* next write index */
    size_t tail;  /* next read index */
    size_t count;
    size_t dropped; /* bytes rejected because the buffer was full */
} ring_buffer_t;

/* Caller owns storage; nothing is allocated. */
void rb_init(ring_buffer_t *rb, uint8_t *storage, size_t capacity);
int rb_push(ring_buffer_t *rb, uint8_t byte);   /* 1 on success, 0 if full */
int rb_pop(ring_buffer_t *rb, uint8_t *byte);   /* 1 on success, 0 if empty */
size_t rb_write(ring_buffer_t *rb, const uint8_t *data, size_t len);
size_t rb_count(const ring_buffer_t *rb);
int rb_is_full(const ring_buffer_t *rb);

#define RB_NO_LINE (-1)
#define RB_LINE_TOO_LONG (-2)

/*
 * Copies one '\n'-terminated line (without CR/LF) into out and removes it.
 * Returns the line length (0 for an empty line), RB_NO_LINE if no complete
 * line is buffered, or RB_LINE_TOO_LONG if the line does not fit in out
 * (the line is discarded).
 */
int rb_read_line(ring_buffer_t *rb, char *out, size_t out_size);

#ifdef __cplusplus
}
#endif

#endif
