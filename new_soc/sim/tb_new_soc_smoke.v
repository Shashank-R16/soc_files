`timescale 1ns/1ps

//============================================================================
// tb_new_soc_smoke.v
//----------------------------------------------------------------------------
// Self-checking smoke testbench for new_soc_top (plain Verilog-2001:
// no SystemVerilog keywords, no $urandom*, continuous drivers are `wire`).
//
// Runs the shipped firmware (sim/fw_sram.hex, regenerate with gen_fw.py):
//   1. seeds 8 pattern words (0x11,0x22,...,0x88) into SRAM 0x1000
//   2. DMA ch0 copies them to 0x3000 (firmware polls CFG[0] auto-clear)
//   3. firmware compares copy against source
//   4. prints "PASS" on the UART (8N1, divisor 54 -> ~115740 baud,
//      8640 ns per bit at 100 MHz), or a single "E" on mismatch
//
// This TB implements a UART receiver on uart_tx_o, decodes up to 4
// characters and compares against "PASS".  SMOKE PASSED therefore proves:
// core boot + firmware execution + AHB fabric + SRAM + APB bridge + UART
// TX + DMA master + DMA data integrity.
//
// Firmware image resolution (ahb_sram_dp / ahb_flash_dp search order):
//   1. "sram.hex"      in xsim's working directory (CLI: package root)
//   2. "fw_sram.hex"   in xsim's working directory (drop it in the folder
//                      that CONTAINS xsim.dir -- no rename needed)
//   3. "sim/fw_sram.hex" walking up to 9 parent directories (finds the
//                      package copy automatically in the GUI flow)
// A "$display ... memory image '<path>' loaded" line confirms which one won.
// Override the primary name with the SRAM_HEX parameter.  Flash boot:
// FLASH_HEX="flash.hex" and boot_mode_i = 2'b01 (XIP) or 2'b10 (boot-copy).
//============================================================================

module tb_new_soc_smoke;

  //---------------------------------------------------------------------------
  // Parameters
  //---------------------------------------------------------------------------
  parameter integer CLK_PERIOD_NS = 10;              // 100 MHz
  parameter integer UART_DIV      = 54;              // must match firmware DLL
  parameter [31:0]  BOOT_ADDR     = 32'h0000_0000;
  parameter [31:0]  MTVEC_ADDR    = 32'h0000_0000;
  parameter         SRAM_HEX      = "";              // "" -> "sram.hex"
  parameter         FLASH_HEX     = "";              // "" -> no flash preload

  localparam         SRAM_FILE      = (SRAM_HEX != "") ? SRAM_HEX : "sram.hex";
  localparam integer BIT_NS         = CLK_PERIOD_NS * 16 * UART_DIV; // 8640 ns
  localparam integer TIMEOUT_CYCLES = 3_000_000;      // 30 ms watchdog
  localparam [31:0]  EXPECTED       = "PASS";

  //---------------------------------------------------------------------------
  // Clock / reset
  //---------------------------------------------------------------------------
  reg clk   = 1'b0;
  reg rst_n = 1'b0;

  always #(CLK_PERIOD_NS/2) clk = ~clk;

  initial begin
    rst_n = 1'b0;
    repeat (8) @(posedge clk);
    rst_n = 1'b1;
  end

  //---------------------------------------------------------------------------
  // DUT connections
  //---------------------------------------------------------------------------
  wire        uart_tx, uart_int;
  wire        spi_clk_o, spi_csn0_o, spi_sdo0_o;
  wire [1:0]  spi_events;
  wire        i2c_scl_o, i2c_scl_oen, i2c_sda_o, i2c_sda_oen, i2c_irq;
  wire        i2s_sck, i2s_ws, i2s_sd;
  wire        wdg_rst;
  wire        core_sleep, dbg_halted;
  wire [63:0] mcycle;

  // SRAM boot (mode 00): the boot controller releases the core immediately.
  // Mode 01 = flash XIP, mode 10 = boot-copy flash->SRAM (see boot_ctrl.v).
  localparam [1:0] BOOT_MODE = 2'b00;

  new_soc_top #(
    .BOOT_ADDR  (BOOT_ADDR),
    .MTVEC_ADDR (MTVEC_ADDR),
    .SRAM_HEX   (SRAM_FILE),
    .FLASH_HEX  (FLASH_HEX)
  ) dut (
    .clk_i        (clk),
    .rst_ni       (rst_n),
    .boot_mode_i  (BOOT_MODE),
    .uart_tx_o    (uart_tx),
    .uart_rx_i    (1'b1),            // line idle; firmware only transmits
    .uart_int_o   (uart_int),
    .spi_clk_o    (spi_clk_o),
    .spi_csn0_o   (spi_csn0_o),
    .spi_sdo0_o   (spi_sdo0_o),
    .spi_sdi0_i   (spi_sdo0_o),      // MOSI->MISO loopback for future SPI tests
    .spi_events_o (spi_events),
    .i2c_scl_i    (1'b1),            // open-drain bus idle (pulled up)
    .i2c_scl_o    (i2c_scl_o),
    .i2c_scl_oen_o(i2c_scl_oen),
    .i2c_sda_i    (1'b1),
    .i2c_sda_o    (i2c_sda_o),
    .i2c_sda_oen_o(i2c_sda_oen),
    .i2c_irq_o    (i2c_irq),
    .i2s_sck_o    (i2s_sck),
    .i2s_ws_o     (i2s_ws),
    .i2s_sd_o     (i2s_sd),
    .i2s_sd_i     (1'b0),            // no I2S source in the smoke test
    .wdg_reset_o  (wdg_rst),         // observed only; never fed back
    .core_sleep_o (core_sleep),
    .dbg_halted_o (dbg_halted),
    .dbg_mcycle_o (mcycle)
  );

  //---------------------------------------------------------------------------
  // UART receiver: decodes 8N1 characters on uart_tx_o.
  // Bit timing is exact: the UART's TX counter is phase-synced on frame
  // start, so data bit i is sampled at (1.5 + i) bit times after the
  // falling edge of the start bit.
  //---------------------------------------------------------------------------
  reg  [7:0]  rx_byte;
  reg  [31:0] rx_word;
  integer     rx_cnt;
  integer     bit_i;

  initial begin
    rx_byte = 8'h00;
    rx_word = 32'h0000_0000;
    rx_cnt  = 0;
    wait (rst_n === 1'b1);
    forever begin
      @(negedge uart_tx);                    // start bit
      #(BIT_NS + BIT_NS/2);                  // -> centre of data bit 0
      for (bit_i = 0; bit_i < 8; bit_i = bit_i + 1) begin
        if (bit_i != 0) #(BIT_NS);
        rx_byte[bit_i] = uart_tx;            // LSB first
      end
      #(BIT_NS);                             // skip stop bit
      rx_cnt  = rx_cnt + 1;
      rx_word = {rx_word[23:0], rx_byte};
      $display("[%0t ns] tb: UART char #%0d = 0x%02h ('%c')",
               $time, rx_cnt, rx_byte, rx_byte);

      if (rx_cnt == 1 && rx_byte == 8'h45) begin   // 'E': DMA verify failed
        $display("");
        $display("==================================================");
        $display(" SMOKE FAILED: firmware printed 'E'");
        $display(" (DMA copy did not match the source data)");
        $display("==================================================");
        $finish;
      end

      if (rx_cnt == 4) begin
        $display("");
        if (rx_word == EXPECTED) begin
          $display("==================================================");
          $display(" SMOKE PASSED: UART received \"PASS\"");
          $display(" core boot, firmware, AHB fabric, SRAM, APB");
          $display(" bridge, UART TX, DMA copy + integrity all OK");
          $display(" mcycle = %0d", mcycle);
          $display("==================================================");
        end else begin
          $display("==================================================");
          $display(" SMOKE FAILED: expected \"PASS\", got 0x%08h", rx_word);
          $display("==================================================");
        end
        $finish;
      end
    end
  end

  //---------------------------------------------------------------------------
  // Watchdog
  //---------------------------------------------------------------------------
  integer cycle_cnt;

  initial begin
    cycle_cnt = 0;
    forever begin
      @(posedge clk);
      cycle_cnt = cycle_cnt + 1;
      if (cycle_cnt >= TIMEOUT_CYCLES) begin
        $display("");
        $display("==================================================");
        $display(" SMOKE FAILED: timeout after 30 ms");
        $display(" saw %0d/4 UART chars, mcycle = %0d", rx_cnt, mcycle);
        $display(" Hints:");
        $display("  - confirm u_sram printed a 'memory image ... loaded' line");
        $display("    (XSim folder: <proj>.sim/sim_1/behav/xsim)");
        $display("  - BOOT_ADDR must be 0 for the SRAM image");
        $display("  - E40P mcycle is not exposed; debug output is tied to zero");
        $display("==================================================");
        $finish;
      end
    end
  end

  //---------------------------------------------------------------------------
  // Banner + optional VCD dump
  //---------------------------------------------------------------------------
  initial begin
    $display("[%0t ns] tb: new_soc smoke start: BOOT_ADDR=0x%08h SRAM image='%s' FLASH image='%s'",
             $time, BOOT_ADDR, SRAM_FILE, FLASH_HEX);
`ifdef VCD
    $dumpfile("tb_new_soc_smoke.vcd");
    $dumpvars(0, tb_new_soc_smoke);
`endif
  end

endmodule
