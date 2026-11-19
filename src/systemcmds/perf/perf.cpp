/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/



#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <stdlib.h>
#include <inttypes.h>

/* perf snapshot: capture every counter line into a buffer between two hrt
 * timestamps, then print. A plain dump prints while it iterates, so over a
 * slow shell the first and last counters are seconds apart; a bench that
 * computes rates from two dumps needs the capture bracketed instead. */
struct snapshot_s {
	char *buf;
	size_t cap;
	size_t len;
	unsigned count;
	bool overflow;
};

static void snapshot_cb(const char *line, void *user)
{
	snapshot_s *s = (snapshot_s *)user;
	size_t n = strlen(line);

	if (s->len + n + 1 >= s->cap) {
		s->overflow = true;
		return;
	}

	memcpy(s->buf + s->len, line, n);
	s->len += n;
	s->buf[s->len++] = '\n';
	s->count++;
}

static int perf_snapshot()
{
	static unsigned snapshot_id = 0;
	snapshot_s s{};
	s.cap = 16384;
	s.buf = (char *)malloc(s.cap);

	if (s.buf == nullptr) {
		printf("perf snapshot error: no memory\n");
		return -1;
	}

	const hrt_abstime t_begin = hrt_absolute_time();
	perf_iterate_all(snapshot_cb, &s);
	const hrt_abstime t_end = hrt_absolute_time();
	const unsigned id = ++snapshot_id;

	if (s.overflow) {
		printf("perf snapshot error: buffer overflow after %u counters\n", s.count);
		free(s.buf);
		return -1;
	}

	printf("perf snapshot %u begin %" PRIu64 " end %" PRIu64 " counters %u\n", id, t_begin, t_end, s.count);
	fwrite(s.buf, 1, s.len, stdout);
	printf("perf snapshot %u done\n", id);
	fflush(stdout);
	free(s.buf);
	return 0;
}

static void print_usage()
{
	PRINT_MODULE_DESCRIPTION("Tool to print performance counters");

	PRINT_MODULE_USAGE_NAME_SIMPLE("perf", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset all counters");
	PRINT_MODULE_USAGE_COMMAND_DESCR("latency", "Print HRT timer latency histogram");
	PRINT_MODULE_USAGE_COMMAND_DESCR("snapshot", "Capture all counters between two timestamps, then print");

	PRINT_MODULE_USAGE_PARAM_COMMENT("Prints all performance counters if no arguments given");
}

extern "C" __EXPORT int perf_main(int argc, char *argv[])
{
	if (argc > 1) {
		if (strcmp(argv[1], "reset") == 0) {
			perf_reset_all();
			return 0;

		} else if (strcmp(argv[1], "latency") == 0) {
			perf_print_latency();
			fflush(stdout);
			return 0;

		} else if (strcmp(argv[1], "snapshot") == 0) {
			return perf_snapshot();
		}

		print_usage();
		return -1;
	}

	perf_print_all();
	fflush(stdout);
	return 0;
}
