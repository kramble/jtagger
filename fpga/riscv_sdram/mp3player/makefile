# Modify this variable to fit your NEORV32 setup (neorv32 home folder)
NEORV32_HOME ?= ../neorv32_sdram

EFFORT=-O1				# O1 is faster than O3 in this specific instance
MARCH=rv32imc_zicsr		# m is essential for speed and c helps somewhat

# These need to be set independently, not as USER_FLAGS+="-DFPM_DEFAULT -DHAVE_ASSERT_H etc"
USER_FLAGS+=-DFPM_DEFAULT
USER_FLAGS+=-DHAVE_ASSERT_H
USER_FLAGS+=-DNDEBUG
USER_FLAGS+=-DASO_INTERLEAVE2
# USER_FLAGS+=-Wno-stringop-overflow	# uncomment to disable warning, see README_MP3

include $(NEORV32_HOME)/sw/common/common.mk

LD_SCRIPT = ./mad.ld	# Override the default linker script in sw/common

