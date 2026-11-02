/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

/**
 * @file cpu_load.hpp
 * CPU load from idle time over the capacity of all cores. Shared by
 * load_mon and the on-board load printer so the arithmetic is tested once.
 */

#pragma once

#include <stdint.h>

/**
 * Fraction of the machine that was busy over an interval.
 *
 * @param idle_delta_us idle time summed over all cores in the interval
 * @param interval_us   wall-clock length of the interval
 * @param ncpus         number of cores, each contributing interval_us of capacity
 * @return 1 - idle / (ncpus * interval), clamped to [0, 1]; -1 when the
 *         interval or the core count is zero (nothing can be computed)
 */
static inline float cpu_load_from_idle(uint64_t idle_delta_us, uint64_t interval_us, unsigned ncpus)
{
	if (interval_us == 0 || ncpus == 0) {
		return -1.f;
	}

	const double capacity = (double)interval_us * (double)ncpus;
	double load = 1.0 - (double)idle_delta_us / capacity;

	if (load < 0.0) {
		load = 0.0;

	} else if (load > 1.0) {
		load = 1.0;
	}

	return (float)load;
}
