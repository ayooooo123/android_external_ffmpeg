/*
Copyright (c) 2018 Raspberry Pi (Trading) Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Authors: John Cox
*/

#include "config.h"
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include "rpi_sand_fns.h"
#include "avassert.h"
#include "cpu.h"
#include "frame.h"
#if ARCH_ARM && HAVE_NEON
#include "libavutil/arm/cpu.h"
#include "libavutil/arm/rpi_sand_neon.h"
#define HAVE_SAND_ASM 1
#elif ARCH_AARCH64 && HAVE_NEON
#include "libavutil/aarch64/cpu.h"
#include "libavutil/aarch64/rpi_sand_neon.h"
#define HAVE_SAND_ASM 1
#else
#define HAVE_SAND_ASM 0
#endif

#define PW 1
#include "rpi_sand_fn_pw.c"
#undef PW

#define PW 2
#include "rpi_sand_fn_pw.c"
#undef PW

#if 1
// Simple round
static void cpy16_to_8(uint8_t * dst, const uint8_t * _src, unsigned int n, const unsigned int shr)
{
    const unsigned int rnd = (1 << shr) >> 1;
    const uint16_t * src = (const uint16_t *)_src;

    for (; n != 0; --n) {
        *dst++ = (*src++ + rnd) >> shr;
    }
}
#else
// Dithered variation
static void cpy16_to_8(uint8_t * dst, const uint8_t * _src, unsigned int n, const unsigned int shr)
{
    unsigned int rnd = (1 << shr) >> 1;
    const unsigned int mask = ((1 << shr) - 1);
    const uint16_t * src = (const uint16_t *)_src;

    for (; n != 0; --n) {
        rnd = *src++ + (rnd & mask);
        *dst++ = rnd >> shr;
    }
}
#endif

// Fetches a single patch - offscreen fixup not done here
// w <= stride1
// unclipped
// _x & _w in pixels, strides in bytes
void av_rpi_sand30_to_planar_y16(uint8_t * dst, const unsigned int dst_stride,
                             const uint8_t * src,
                             unsigned int stride1, unsigned int stride2,
                             unsigned int _x, unsigned int y,
                             unsigned int _w, unsigned int h)
{
    const unsigned int x0 = (_x / 3) * 4; // Byte offset of the word
    const unsigned int xskip0 = _x - (x0 >> 2) * 3;
    const unsigned int x1 = ((_x + _w) / 3) * 4;
    const unsigned int xrem1 = _x + _w - (x1 >> 2) * 3;
    const unsigned int mask = stride1 - 1;
    const uint8_t * p0 = src + (x0 & mask) + y * stride1 + (x0 & ~mask) * stride2;
    const unsigned int slice_inc = ((stride2 - 1) * stride1) >> 2;  // RHS of a stripe to LHS of next in words

#if HAVE_SAND_ASM
    if (_x == 0 && have_neon(av_get_cpu_flags())) {
        ff_rpi_sand30_lines_to_planar_y16(dst, dst_stride, src, stride1, stride2, _x, y, _w, h);
        return;
    }
#endif

    if (x0 == x1) {
        // *******************
        // Partial single word xfer
        return;
    }

    for (unsigned int i = 0; i != h; ++i, dst += dst_stride, p0 += stride1)
    {
        unsigned int x = x0;
        const uint32_t * p = (const uint32_t *)p0;
        uint16_t * d = (uint16_t *)dst;

        if (xskip0 != 0) {
            const uint32_t p3 = *p++;

            if (xskip0 == 1)
                *d++ = (p3 >> 10) & 0x3ff;
            *d++ = (p3 >> 20) & 0x3ff;

            if (((x += 4) & mask) == 0)
                p += slice_inc;
        }

        while (x != x1) {
            const uint32_t p3 = *p++;
            *d++ = p3 & 0x3ff;
            *d++ = (p3 >> 10) & 0x3ff;
            *d++ = (p3 >> 20) & 0x3ff;

            if (((x += 4) & mask) == 0)
                p += slice_inc;
        }

        if (xrem1 != 0) {
            const uint32_t p3 = *p;

            *d++ = p3 & 0x3ff;
            if (xrem1 == 2)
                *d++ = (p3 >> 10) & 0x3ff;
        }
    }
}


void av_rpi_sand30_to_planar_c16(uint8_t * dst_u, const unsigned int dst_stride_u,
                             uint8_t * dst_v, const unsigned int dst_stride_v,
                             const uint8_t * src,
                             unsigned int stride1, unsigned int stride2,
                             unsigned int _x, unsigned int y,
                             unsigned int _w, unsigned int h)
{
    const unsigned int x0 = (_x / 3) * 8; // Byte offset of the word
    const unsigned int xskip0 = _x - (x0 >> 3) * 3;
    const unsigned int x1 = ((_x + _w) / 3) * 8;
    const unsigned int xrem1 = _x + _w - (x1 >> 3) * 3;
    const unsigned int mask = stride1 - 1;
    const uint8_t * p0 = src + (x0 & mask) + y * stride1 + (x0 & ~mask) * stride2;
    const unsigned int slice_inc = ((stride2 - 1) * stride1) >> 2;  // RHS of a stripe to LHS of next in words

#if HAVE_SAND_ASM
    if (_x == 0 && have_neon(av_get_cpu_flags())) {
        ff_rpi_sand30_lines_to_planar_c16(dst_u, dst_stride_u, dst_v, dst_stride_v,
                                       src, stride1, stride2, _x, y, _w, h);
        return;
    }
#endif

    if (x0 == x1) {
        // *******************
        // Partial single word xfer
        return;
    }

    for (unsigned int i = 0; i != h; ++i, dst_u += dst_stride_u, dst_v += dst_stride_v, p0 += stride1)
    {
        unsigned int x = x0;
        const uint32_t * p = (const uint32_t *)p0;
        uint16_t * du = (uint16_t *)dst_u;
        uint16_t * dv = (uint16_t *)dst_v;

        if (xskip0 != 0) {
            const uint32_t p3a = *p++;
            const uint32_t p3b = *p++;

            if (xskip0 == 1)
            {
                *du++ = (p3a >> 20) & 0x3ff;
                *dv++ = (p3b >>  0) & 0x3ff;
            }
            *du++ = (p3b >> 10) & 0x3ff;
            *dv++ = (p3b >> 20) & 0x3ff;

            if (((x += 8) & mask) == 0)
                p += slice_inc;
        }

        while (x != x1) {
            const uint32_t p3a = *p++;
            const uint32_t p3b = *p++;

            *du++ = p3a & 0x3ff;
            *dv++ = (p3a >> 10) & 0x3ff;
            *du++ = (p3a >> 20) & 0x3ff;
            *dv++ = p3b & 0x3ff;
            *du++ = (p3b >> 10) & 0x3ff;
            *dv++ = (p3b >> 20) & 0x3ff;

            if (((x += 8) & mask) == 0)
                p += slice_inc;
        }

        if (xrem1 != 0) {
            const uint32_t p3a = *p++;
            const uint32_t p3b = *p++;

            *du++ = p3a & 0x3ff;
            *dv++ = (p3a >> 10) & 0x3ff;
            if (xrem1 == 2)
            {
                *du++ = (p3a >> 20) & 0x3ff;
                *dv++ = p3b & 0x3ff;
            }
        }
    }
}

// Fetches a single patch - offscreen fixup not done here
// w <= stride1
// single lose bottom 2 bits truncation
// _x & _w in pixels, strides in bytes
void av_rpi_sand30_to_planar_y8(uint8_t * dst, const unsigned int dst_stride,
                             const uint8_t * src,
                             unsigned int stride1, unsigned int stride2,
                             unsigned int _x, unsigned int y,
                             unsigned int _w, unsigned int h)
{
    const unsigned int x0 = (_x / 3) * 4; // Byte offset of the word
    const unsigned int xskip0 = _x - (x0 >> 2) * 3;
    const unsigned int x1 = ((_x + _w) / 3) * 4;
    const unsigned int xrem1 = _x + _w - (x1 >> 2) * 3;
    const unsigned int mask = stride1 - 1;
    const uint8_t * p0 = src + (x0 & mask) + y * stride1 + (x0 & ~mask) * stride2;
    const unsigned int slice_inc = ((stride2 - 1) * stride1) >> 2;  // RHS of a stripe to LHS of next in words

#if HAVE_SAND_ASM
    if (_x == 0) {
        ff_rpi_sand30_lines_to_planar_y8(dst, dst_stride, src, stride1, stride2, _x, y, _w, h);
        return;
    }
#endif

    if (x0 == x1) {
        // *******************
        // Partial single word xfer
        return;
    }

    for (unsigned int i = 0; i != h; ++i, dst += dst_stride, p0 += stride1)
    {
        unsigned int x = x0;
        const uint32_t * p = (const uint32_t *)p0;
        uint8_t * d = dst;

        if (xskip0 != 0) {
            const uint32_t p3 = *p++;

            if (xskip0 == 1)
                *d++ = (p3 >> 12) & 0xff;
            *d++ = (p3 >> 22) & 0xff;

            if (((x += 4) & mask) == 0)
                p += slice_inc;
        }

        while (x != x1) {
            const uint32_t p3 = *p++;
            *d++ = (p3 >> 2) & 0xff;
            *d++ = (p3 >> 12) & 0xff;
            *d++ = (p3 >> 22) & 0xff;

            if (((x += 4) & mask) == 0)
                p += slice_inc;
        }

        if (xrem1 != 0) {
            const uint32_t p3 = *p;

            *d++ = (p3 >> 2) & 0xff;
            if (xrem1 == 2)
                *d++ = (p3 >> 12) & 0xff;
        }
    }
}



// w/h in pixels
void av_rpi_sand16_to_sand8(uint8_t * dst, const unsigned int dst_stride1, const unsigned int dst_stride2,
                         const uint8_t * src, const unsigned int src_stride1, const unsigned int src_stride2,
                         unsigned int w, unsigned int h, const unsigned int shr)
{
    const unsigned int n = dst_stride1 / 2;
    unsigned int j;

    // This is true for our current layouts
    av_assert0(dst_stride1 == src_stride1);

    // As we have the same stride1 for src & dest and src is wider than dest
    // then if we loop on src we can always write contiguously to dest
    // We make no effort to copy an exact width - round up to nearest src stripe
    // as we will always have storage in dest for that

#if ARCH_ARM && HAVE_NEON
    if (shr == 3 && src_stride1 == 128) {
        for (j = 0; j + n < w; j += dst_stride1) {
            uint8_t * d = dst + j * dst_stride2;
            const uint8_t * s1 = src + j * 2 * src_stride2;
            const uint8_t * s2 = s1 + src_stride1 * src_stride2;

            ff_rpi_sand128b_stripe_to_8_10(d, s1, s2, h);
        }
    }
    else
#endif
    {
        for (j = 0; j + n < w; j += dst_stride1) {
            uint8_t * d = dst + j * dst_stride2;
            const uint8_t * s1 = src + j * 2 * src_stride2;
            const uint8_t * s2 = s1 + src_stride1 * src_stride2;

            for (unsigned int i = 0; i != h; ++i, s1 += src_stride1, s2 += src_stride1, d += dst_stride1) {
                cpy16_to_8(d, s1, n, shr);
                cpy16_to_8(d + n, s2, n, shr);
            }
        }
    }

    // Fix up a trailing dest half stripe
    if (j < w) {
        uint8_t * d = dst + j * dst_stride2;
        const uint8_t * s1 = src + j * 2 * src_stride2;

        for (unsigned int i = 0; i != h; ++i, s1 += src_stride1, d += dst_stride1) {
            cpy16_to_8(d, s1, n, shr);
        }
    }
}

// 10-bit sand chroma (interleaved U,V) to 8-bit planar U and V, dropping the
// low 2 bits the way av_rpi_sand30_to_planar_y8 does. Same addressing as
// av_rpi_sand30_to_planar_c16 (_x and _w in chroma pixels, y in chroma rows).
static void sand30_to_planar_c8(uint8_t * dst_u, const unsigned int dst_stride_u,
                                uint8_t * dst_v, const unsigned int dst_stride_v,
                                const uint8_t * src,
                                unsigned int stride1, unsigned int stride2,
                                unsigned int _x, unsigned int y,
                                unsigned int _w, unsigned int h)
{
    const unsigned int x0 = (_x / 3) * 8; // Byte offset of the word pair
    const unsigned int xskip0 = _x - (x0 >> 3) * 3;
    const unsigned int x1 = ((_x + _w) / 3) * 8;
    const unsigned int xrem1 = _x + _w - (x1 >> 3) * 3;
    const unsigned int mask = stride1 - 1;
    const uint8_t * p0 = src + (x0 & mask) + y * stride1 + (x0 & ~mask) * stride2;
    const unsigned int slice_inc = ((stride2 - 1) * stride1) >> 2;  // RHS of a stripe to LHS of next in words

    if (x0 == x1)
        return;

    for (unsigned int i = 0; i != h; ++i, dst_u += dst_stride_u, dst_v += dst_stride_v, p0 += stride1)
    {
        unsigned int x = x0;
        const uint32_t * p = (const uint32_t *)p0;
        uint8_t * du = dst_u;
        uint8_t * dv = dst_v;

        if (xskip0 != 0) {
            const uint32_t p3a = *p++;
            const uint32_t p3b = *p++;

            if (xskip0 == 1)
            {
                *du++ = (p3a >> 22) & 0xff;
                *dv++ = (p3b >>  2) & 0xff;
            }
            *du++ = (p3b >> 12) & 0xff;
            *dv++ = (p3b >> 22) & 0xff;

            if (((x += 8) & mask) == 0)
                p += slice_inc;
        }

        while (x != x1) {
            const uint32_t p3a = *p++;
            const uint32_t p3b = *p++;

            du[0] = (p3a >>  2) & 0xff;
            dv[0] = (p3a >> 12) & 0xff;
            du[1] = (p3a >> 22) & 0xff;
            dv[1] = (p3b >>  2) & 0xff;
            du[2] = (p3b >> 12) & 0xff;
            dv[2] = (p3b >> 22) & 0xff;
            du += 3;
            dv += 3;

            if (((x += 8) & mask) == 0)
                p += slice_inc;
        }

        if (xrem1 != 0) {
            const uint32_t p3a = *p++;
            const uint32_t p3b = *p++;

            *du++ = (p3a >>  2) & 0xff;
            *dv++ = (p3a >> 12) & 0xff;
            if (xrem1 == 2)
            {
                *du++ = (p3a >> 22) & 0xff;
                *dv++ = (p3b >>  2) & 0xff;
            }
        }
    }
}

// Banded conversion over the CPU cores, for the 8-bit outputs the Android
// codec2 HAL copies every picture through. Each band is a run of whole luma
// rows (a multiple of 16, so chroma rows split evenly); bands write disjoint
// destination rows, so the output is identical to the single-thread one.
// The aarch64 NEON 8-bit row converters (ff_rpi_sand8_lines_to_planar_y8/c8)
// ignore their y argument and always start at row 0, so 8-bit bands start
// the source at their first row and pass y = 0; within a sand column, rows
// are stride1 bytes apart, so that addresses the same pixels.
// SAND30_TO_YUV420P lets the HAL write 10-bit pictures straight into its
// planar output buffer, without an NV12 copy and a second conversion.
enum sand_band_mode { SAND8_TO_YUV420P, SAND8_TO_NV12, SAND30_TO_NV12, SAND30_TO_YUV420P };

typedef struct SandBandJob {
    enum sand_band_mode mode;
    AVFrame *dst;
    const AVFrame *src;
    unsigned int x, y, w, stride1, stride2_y, stride2_c;
    unsigned int y0, y1; // luma rows of this band, relative to y
} SandBandJob;

static void sand_band(const SandBandJob * const j)
{
    const unsigned int h = j->y1 - j->y0;
    AVFrame * const dst = j->dst;
    const AVFrame * const src = j->src;
    uint8_t * const d0 = dst->data[0] + (size_t)j->y0 * dst->linesize[0];
    uint8_t * const d1 = dst->data[1] + (size_t)(j->y0 / 2) * dst->linesize[1];

    switch (j->mode) {
    case SAND8_TO_YUV420P:
        av_rpi_sand_to_planar_y8(d0, dst->linesize[0],
                                 src->data[0] + (size_t)(j->y + j->y0) * j->stride1, j->stride1, j->stride2_y,
                                 j->x, 0, j->w, h);
        av_rpi_sand_to_planar_c8(d1, dst->linesize[1],
                                 dst->data[2] + (size_t)(j->y0 / 2) * dst->linesize[2], dst->linesize[2],
                                 src->data[1] + (size_t)((j->y + j->y0) / 2) * j->stride1, j->stride1, j->stride2_c,
                                 j->x / 2, 0, j->w / 2, h / 2);
        break;
    case SAND8_TO_NV12:
        av_rpi_sand_to_planar_y8(d0, dst->linesize[0],
                                 src->data[0] + (size_t)(j->y + j->y0) * j->stride1, j->stride1, j->stride2_y,
                                 j->x, 0, j->w, h);
        av_rpi_sand_to_planar_y8(d1, dst->linesize[1],
                                 src->data[1] + (size_t)((j->y + j->y0) / 2) * j->stride1, j->stride1, j->stride2_c,
                                 j->x / 2, 0, j->w, h / 2);
        break;
    case SAND30_TO_NV12:
        av_rpi_sand30_to_planar_y8(d0, dst->linesize[0], src->data[0], j->stride1, j->stride2_y,
                                   j->x, j->y + j->y0, j->w, h);
        av_rpi_sand30_to_planar_y8(d1, dst->linesize[1], src->data[1], j->stride1, j->stride2_c,
                                   j->x / 2, (j->y + j->y0) / 2, j->w, h / 2);
        break;
    case SAND30_TO_YUV420P:
        av_rpi_sand30_to_planar_y8(d0, dst->linesize[0], src->data[0], j->stride1, j->stride2_y,
                                   j->x, j->y + j->y0, j->w, h);
        sand30_to_planar_c8(d1, dst->linesize[1],
                            dst->data[2] + (size_t)(j->y0 / 2) * dst->linesize[2], dst->linesize[2],
                            src->data[1], j->stride1, j->stride2_c,
                            j->x / 2, (j->y + j->y0) / 2, j->w / 2, h / 2);
        break;
    }
}

static void *sand_band_thread(void *job)
{
    sand_band(job);
    return NULL;
}

#define SAND_MAX_BANDS 4
#define SAND_POOL_WORKERS (SAND_MAX_BANDS - 1)

// Helper threads started once and kept for the life of the process: starting
// three threads per picture cost 0.35-0.7 ms on the Pi 4. One caller at a
// time owns the pool (busy); a concurrent caller (a second decoder) starts
// its own threads instead, as before.
typedef struct SandPool {
    pthread_mutex_t lock;
    pthread_cond_t work, done;
    pthread_mutex_t busy;
    const SandBandJob *jobs[SAND_POOL_WORKERS];
    unsigned int generation;
    unsigned int pending;
    int ready;
} SandPool;

static SandPool sand_pool = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .work = PTHREAD_COND_INITIALIZER,
    .done = PTHREAD_COND_INITIALIZER,
    .busy = PTHREAD_MUTEX_INITIALIZER,
};
static pthread_once_t sand_pool_once = PTHREAD_ONCE_INIT;

static void *sand_pool_worker(void *arg)
{
    SandPool * const p = &sand_pool;
    const unsigned int slot = (unsigned int)(uintptr_t)arg;
    unsigned int seen = 0;

    pthread_mutex_lock(&p->lock);
    for (;;) {
        const SandBandJob *job;
        while (p->generation == seen)
            pthread_cond_wait(&p->work, &p->lock);
        seen = p->generation;
        job = p->jobs[slot];
        if (!job)
            continue;
        pthread_mutex_unlock(&p->lock);
        sand_band(job);
        pthread_mutex_lock(&p->lock);
        if (--p->pending == 0)
            pthread_cond_signal(&p->done);
    }
    return NULL;
}

static void sand_pool_start(void)
{
    pthread_attr_t attr;
    unsigned int i;

    if (pthread_attr_init(&attr))
        return;
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    for (i = 0; i < SAND_POOL_WORKERS; i++) {
        pthread_t t;
        if (pthread_create(&t, &attr, sand_pool_worker, (void *)(uintptr_t)i))
            break;
    }
    pthread_attr_destroy(&attr);
    // Use the pool only if every worker started; a partial pool would leave
    // some slots unserved. Started workers just stay asleep.
    sand_pool.ready = i == SAND_POOL_WORKERS;
}

// Runs jobs[1..bands-1] on the pool and jobs[0] here. Returns 0 if the pool
// is unavailable or owned by another caller; nothing has run then.
static int sand_pool_run(const SandBandJob * const jobs, const unsigned int bands)
{
    SandPool * const p = &sand_pool;

    pthread_once(&sand_pool_once, sand_pool_start);
    if (!p->ready || pthread_mutex_trylock(&p->busy))
        return 0;

    pthread_mutex_lock(&p->lock);
    for (unsigned int i = 0; i < SAND_POOL_WORKERS; i++)
        p->jobs[i] = i + 1 < bands ? &jobs[i + 1] : NULL;
    p->pending = bands - 1;
    p->generation++;
    pthread_cond_broadcast(&p->work);
    pthread_mutex_unlock(&p->lock);

    sand_band(&jobs[0]);

    pthread_mutex_lock(&p->lock);
    while (p->pending)
        pthread_cond_wait(&p->done, &p->lock);
    pthread_mutex_unlock(&p->lock);

    pthread_mutex_unlock(&p->busy);
    return 1;
}

// Cores to split over, looked up once: av_cpu_count() reads the affinity
// mask, and the HAL calls this for every picture.
static unsigned int sand_bands_for_cpus(void)
{
    static atomic_int cached;
    int n = atomic_load_explicit(&cached, memory_order_relaxed);
    if (n <= 0) {
        n = av_cpu_count();
        n = n < 1 ? 1 : n > SAND_MAX_BANDS ? SAND_MAX_BANDS : n;
        atomic_store_explicit(&cached, n, memory_order_relaxed);
    }
    return n;
}

static void sand_to_planar_banded(enum sand_band_mode mode, AVFrame * const dst, const AVFrame * const src,
                                  unsigned int x, unsigned int y, unsigned int w, unsigned int h,
                                  unsigned int stride2_y, unsigned int stride2_c)
{
    SandBandJob jobs[SAND_MAX_BANDS];
    pthread_t threads[SAND_MAX_BANDS];
    int started[SAND_MAX_BANDS] = { 0 };
    unsigned int bands = sand_bands_for_cpus();
    unsigned int band_h;

    // Small pictures and odd crops stay on one thread.
    if (h < 720 || (y & 1))
        bands = 1;
    // Even bands: the height split rounded up to whole 16-row groups (so
    // chroma rows split evenly); the last band takes what is left, and a
    // band that would be empty is dropped.
    band_h = ((h + bands * 16 - 1) / (bands * 16)) * 16;
    while (bands > 1 && (bands - 1) * band_h >= h)
        bands--;

    for (unsigned int i = 0; i < bands; i++) {
        jobs[i] = (SandBandJob){
            .mode = mode, .dst = dst, .src = src, .x = x, .y = y, .w = w,
            .stride1 = av_rpi_sand_frame_stride1(src), .stride2_y = stride2_y, .stride2_c = stride2_c,
            .y0 = i * band_h, .y1 = i == bands - 1 ? h : (i + 1) * band_h,
        };
    }
    if (bands > 1 && sand_pool_run(jobs, bands))
        return;
    // Without the pool: bands 1.. on new threads, band 0 here. A helper that
    // cannot start runs inline, so the picture is always complete.
    for (unsigned int i = 1; i < bands; i++)
        started[i] = pthread_create(&threads[i], NULL, sand_band_thread, &jobs[i]) == 0;
    sand_band(&jobs[0]);
    for (unsigned int i = 1; i < bands; i++) {
        if (started[i])
            pthread_join(threads[i], NULL);
        else
            sand_band(&jobs[i]);
    }
}

int av_rpi_sand_to_planar_frame(AVFrame * const dst, const AVFrame * const src)
{
    const int w = av_frame_cropped_width(src);
    const int h = av_frame_cropped_height(src);
    const int x = src->crop_left;
    const int y = src->crop_top;
    const unsigned int stride2_y = av_rpi_sand_frame_stride2_y(src);
    const unsigned int stride2_c = av_rpi_sand_frame_stride2_c(src);

    // We will crop as part of the conversion
    dst->crop_top = 0;
    dst->crop_left = 0;
    dst->crop_bottom = 0;
    dst->crop_right = 0;

    switch (src->format){
        case AV_PIX_FMT_SAND128:
        case AV_PIX_FMT_RPI4_8:
            switch (dst->format){
                case AV_PIX_FMT_YUV420P:
                    sand_to_planar_banded(SAND8_TO_YUV420P, dst, src, x, y, w, h, stride2_y, stride2_c);
                    break;
                case AV_PIX_FMT_NV12:
                    sand_to_planar_banded(SAND8_TO_NV12, dst, src, x, y, w, h, stride2_y, stride2_c);
                    break;
                default:
                    return -1;
            }
            break;
        case AV_PIX_FMT_SAND64_10:
            switch (dst->format){
                case AV_PIX_FMT_YUV420P10:
                    av_rpi_sand_to_planar_y16(dst->data[0], dst->linesize[0],
                                             src->data[0],
                                             av_rpi_sand_frame_stride1(src), stride2_y,
                                             x*2, y, w*2, h);
                    av_rpi_sand_to_planar_c16(dst->data[1], dst->linesize[1],
                                             dst->data[2], dst->linesize[2],
                                             src->data[1],
                                             av_rpi_sand_frame_stride1(src), stride2_c,
                                             x, y/2,  w, h/2);
                    break;
                default:
                    return -1;
            }
            break;
        case AV_PIX_FMT_RPI4_10:
            switch (dst->format){
                case AV_PIX_FMT_YUV420P10:
                    av_rpi_sand30_to_planar_y16(dst->data[0], dst->linesize[0],
                                             src->data[0],
                                             av_rpi_sand_frame_stride1(src), stride2_y,
                                             x, y, w, h);
                    av_rpi_sand30_to_planar_c16(dst->data[1], dst->linesize[1],
                                             dst->data[2], dst->linesize[2],
                                             src->data[1],
                                             av_rpi_sand_frame_stride1(src), stride2_c,
                                             x/2, y/2, w/2, h/2);
                    break;
                case AV_PIX_FMT_NV12:
                    sand_to_planar_banded(SAND30_TO_NV12, dst, src, x, y, w, h, stride2_y, stride2_c);
                    break;
                case AV_PIX_FMT_YUV420P:
                    sand_to_planar_banded(SAND30_TO_YUV420P, dst, src, x, y, w, h, stride2_y, stride2_c);
                    break;
                default:
                    return -1;
            }
            break;
        default:
            return -1;
    }

    return av_frame_copy_props(dst, src);
}
