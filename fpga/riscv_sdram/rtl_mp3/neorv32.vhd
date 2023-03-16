-- #################################################################################################
-- # << NEORV32 - Test Setup using the default UART-Bootloader to upload and run executables >>    #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32                           #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_test_setup_bootloader is
  generic (
    -- adapt these for your setup --
 -- CLOCK_FREQUENCY   : natural := 100000000; -- clock frequency of clk_i in Hz
    CLOCK_FREQUENCY   : natural := 125000000; -- clock frequency of clk_i in Hz
 -- CLOCK_FREQUENCY   : natural := 133333333; -- clock frequency of clk_i in Hz
    MEM_INT_IMEM_SIZE : natural := 16*1024;   -- size of processor-internal instruction memory in bytes
    MEM_INT_DMEM_SIZE : natural := 8*1024     -- size of processor-internal data memory in bytes
  );
  port (
    -- Global control --
    clk_i       : in  std_ulogic; -- global clock, rising edge
    rstn_i      : in  std_ulogic; -- global reset, low-active, async
    -- GPIO --
    gpio_o      : out std_ulogic_vector(7 downto 0); -- parallel output
    -- UART0 --
    uart0_txd_o : out std_ulogic; -- UART0 send data
    uart0_rxd_i : in  std_ulogic; -- UART0 receive data
   -- Wishbone bus interface (available if MEM_EXT_EN = true) --
   wb_tag_o       : out std_ulogic_vector(02 downto 0); -- request tag
   wb_adr_o       : out std_ulogic_vector(31 downto 0); -- address
   wb_dat_i       : in  std_ulogic_vector(31 downto 0) := (others => 'U'); -- read data
   wb_dat_o       : out std_ulogic_vector(31 downto 0); -- write data
   wb_we_o        : out std_ulogic; -- read/write
   wb_sel_o       : out std_ulogic_vector(03 downto 0); -- byte enable
   wb_stb_o       : out std_ulogic; -- strobe
   wb_cyc_o       : out std_ulogic; -- valid cycle
   wb_ack_i       : in  std_ulogic := 'L'; -- transfer acknowledge
   wb_err_i       : in  std_ulogic := 'L'  -- transfer error
  );
end entity;

architecture neorv32_test_setup_bootloader_rtl of neorv32_test_setup_bootloader is

  signal con_gpio_o : std_ulogic_vector(63 downto 0);

begin

  -- The Core Of The Problem ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_top_inst: neorv32_top
  generic map (
    -- General --
    CLOCK_FREQUENCY              => CLOCK_FREQUENCY,   -- clock frequency of clk_i in Hz
    INT_BOOTLOADER_EN            => true,              -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_C        => true,              -- implement compressed extension?
    CPU_EXTENSION_RISCV_M        => true,              -- implement mul/div extension?
    CPU_EXTENSION_RISCV_Zicsr    => true,              -- implement CSR system?
    CPU_EXTENSION_RISCV_Zicntr   => true,              -- implement base counters?
    CPU_EXTENSION_RISCV_Zifencei => true,              -- implement instruction stream sync.?
    -- Tuning Options --
    FAST_MUL_EN                  => true,              -- use DSPs for M extension's multiplier (MJ added)
    FAST_SHIFT_EN                => true,              -- use barrel shifter for shift operations
    CPU_IPB_ENTRIES              => 2,                 -- entries in instruction prefetch buffer, has to be a power of 2, min 1
    -- Internal Instruction memory --
    MEM_INT_IMEM_EN              => true,              -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE            => MEM_INT_IMEM_SIZE, -- size of processor-internal instruction memory in bytes
    -- Internal Data memory --
    MEM_INT_DMEM_EN              => true,              -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE            => MEM_INT_DMEM_SIZE, -- size of processor-internal data memory in bytes
    -- Internal Cache memory (iCACHE) --
    ICACHE_EN                    => true,             -- implement instruction cache
    ICACHE_NUM_BLOCKS            => 64,                -- i-cache: number of blocks (min 1), has to be a power of 2
    ICACHE_BLOCK_SIZE            => 256,               -- i-cache: block size in bytes (min 4), has to be a power of 2
    ICACHE_ASSOCIATIVITY         => 1,                 -- i-cache: associativity / number of sets (1=direct_mapped), has to be a power of 2
    -- External memory interface (WISHBONE) --
    MEM_EXT_EN                   => true,              -- implement external memory bus interface?
    -- Processor peripherals --
    IO_GPIO_NUM                  => 8,                 -- number of GPIO input/output pairs (0..64)
    IO_MTIME_EN                  => true,              -- implement machine system timer (MTIME)?
    IO_UART0_EN                  => true               -- implement primary universal asynchronous receiver/transmitter (UART0)?
  )
  port map (
    -- Global control --
    clk_i       => clk_i,       -- global clock, rising edge
    rstn_i      => rstn_i,      -- global reset, low-active, async
    -- GPIO (available if IO_GPIO_EN = true) --
    gpio_o      => con_gpio_o,  -- parallel output
    -- primary UART0 (available if IO_GPIO_NUM > 0) --
    uart0_txd_o => uart0_txd_o, -- UART0 send data
    uart0_rxd_i => uart0_rxd_i, -- UART0 receive data
     -- Wishbone bus interface (available if MEM_EXT_EN = true) --
    wb_tag_o      => wb_tag_o,                         -- tag
    wb_adr_o      => wb_adr_o,                         -- address
    wb_dat_i      => wb_dat_i,                         -- read data
    wb_dat_o      => wb_dat_o,                         -- write data
    wb_we_o       => wb_we_o,                          -- read/write
    wb_sel_o      => wb_sel_o,                         -- byte enable
    wb_stb_o      => wb_stb_o,                         -- strobe
    wb_cyc_o      => wb_cyc_o,                         -- valid cycle
    wb_ack_i      => wb_ack_i,                         -- transfer acknowledge
    wb_err_i      => wb_err_i                          -- transfer error

  );

  -- GPIO output --
  gpio_o <= con_gpio_o(7 downto 0);


end architecture;
