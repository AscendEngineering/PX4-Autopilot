############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

px4_add_library(dshot_codec
	dshot_codec.c
)
target_include_directories(dshot_codec PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(dshot_codec PRIVATE ${MAX_CUSTOM_OPT_LEVEL})

px4_add_unit_gtest(SRC DShotCodecTest.cpp
	INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/test
	LINKLIBS dshot_codec
)

# The PIO programs of the RP2350 backend are checked on the host: the platform
# directory is not part of the SITL build, so the test lives here.
px4_add_unit_gtest(SRC DShotPioProgramTest.cpp
	INCLUDES ${PX4_SOURCE_DIR}/platforms/nuttx/src/px4/rpi/rpi_common/dshot
	LINKLIBS dshot_codec
)

# The RP2350 backend itself, compiled for the host on a model of the PIO block.
px4_add_unit_gtest(SRC DShotBackendTest.cpp
	EXTRA_SRCS
		test/dshot_host_fakes.c
		${PX4_SOURCE_DIR}/platforms/nuttx/src/px4/rpi/rpi_common/dshot/dshot.c
	COMPILE_FLAGS -DDSHOT_HOST_TEST=1
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}/test
		${PX4_SOURCE_DIR}/platforms/nuttx/src/px4/rpi/rpi_common/dshot
	LINKLIBS dshot_codec
)
