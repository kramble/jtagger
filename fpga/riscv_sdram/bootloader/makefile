# Modify this variable to fit your NEORV32 setup (neorv32 home folder)
NEORV32_HOME ?= ../neorv32

# These need to be set independently, not as USER_FLAGS+="-DUART_BAUD=19200 -DAUTO_BOOT_TIMEOUT=0 etc"
USER_FLAGS+=-DUART_BAUD=19200
USER_FLAGS+=-DAUTO_BOOT_TIMEOUT=0
USER_FLAGS+=-DSPI_EN=0
USER_FLAGS+=-DXIP_EN=0

include $(NEORV32_HOME)/sw/common/common.mk

# Override the default linker script in sw/common ... currently DISABLED as using sdram MP3 memory definitions
# does not work with the bootloader which silently fails, I suspect due to data/stack positioning. Instead use
# the standard sw/common/neorv32.ld with 16kB IMEM and 8kB DMEM.
# LD_SCRIPT = ./bootloader.ld

