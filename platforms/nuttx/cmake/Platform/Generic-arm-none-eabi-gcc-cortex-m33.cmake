
# Mirrors NuttX arch/arm/src/armv8-m/Toolchain.defs so that PX4 and NuttX agree
# on the float ABI; a mismatch selects a different libgcc multilib and corrupts
# the long-division veneers. The Cortex-M33 FPU is single precision (FPv5-SP)
# only, so there is no CONFIG_ARCH_DPFPU variant as on Cortex-M7.
if(CONFIG_ARCH_FPU AND CONFIG_ARM_FPU_ABI_SOFT)
	set(cpu_flags "-mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=softfp")
elseif(CONFIG_ARCH_FPU)
	set(cpu_flags "-mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard")
else()
	set(cpu_flags "-mcpu=cortex-m33 -mthumb -mfloat-abi=soft")
endif()

set(CMAKE_C_FLAGS "${cpu_flags}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${cpu_flags}" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS "${cpu_flags} -D__ASSEMBLY__" CACHE STRING "" FORCE)
