#include "ring_buffer.h"

void rb_init(ring_buffer_t *rb, uint8_t *storage, size_t capacity) {
    rb->storage = storage;
    rb->capacity = capacity;
    rb->head = rb->tail = rb->count = rb->dropped = 0;
}

int rb_push(ring_buffer_t *rb, uint8_t byte) {
    if (rb->count == rb->capacity) {
        rb->dropped++;
        return 0;
    }
    rb->storage[rb->head] = byte;
    rb->head = (rb->head + 1) % rb->capacity;
    rb->count++;
    return 1;
}

int rb_pop(ring_buffer_t *rb, uint8_t *byte) {
    if (rb->count == 0) return 0;
    *byte = rb->storage[rb->tail];
    rb->tail = (rb->tail + 1) % rb->capacity;
    rb->count--;
    return 1;
}

size_t rb_write(ring_buffer_t *rb, const uint8_t *data, size_t len) {
    size_t written = 0;
    for (size_t i = 0; i < len; ++i) written += (size_t)rb_push(rb, data[i]);
    return written;
}

size_t rb_count(const ring_buffer_t *rb) { return rb->count; }

int rb_is_full(const ring_buffer_t *rb) { return rb->count == rb->capacity; }

int rb_read_line(ring_buffer_t *rb, char *out, size_t out_size) {
    /* Look for '\n' without consuming anything. */
    size_t line_len = 0;
    int found = 0;
    for (size_t i = 0; i < rb->count; ++i) {
        if (rb->storage[(rb->tail + i) % rb->capacity] == '\n') {
            line_len = i;
            found = 1;
            break;
        }
    }
    if (!found) {
        /* A full buffer with no newline can never complete; drop it so the stream resyncs. */
        if (rb_is_full(rb)) {
            rb->dropped += rb->count;
            rb->tail = rb->head;
            rb->count = 0;
        }
        return RB_NO_LINE;
    }

    int fits = line_len < out_size;
    size_t written = 0;
    uint8_t byte;
    for (size_t i = 0; i <= line_len; ++i) {
        rb_pop(rb, &byte);
        if (fits && byte != '\n' && byte != '\r') out[written++] = (char)byte;
    }
    if (!fits) return RB_LINE_TOO_LONG;
    out[written] = '\0';
    return (int)written;
}
