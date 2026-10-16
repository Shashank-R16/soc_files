`timescale 1ns/1ps

//=============================================================================
// dma8_ahb_master.v
//-----------------------------------------------------------------------------
// AHB-Lite execution master of the 8-stream DMA controller.  Direct 8-lane
// extension of the proven dma_ahb_master (4-channel) FSM; the AHB contract
// is unchanged:
//
//   * HBUSREQ asserted while the engine has pending bus work; the fabric
//     answers on HGRANT (combinational one-hot grant).
//   * the address phase (HTRANS = NONSEQ) is presented for exactly ONE cycle,
//     starting on the first cycle HGRANT is seen; afterwards HTRANS = IDLE.
//   * the data phase waits for the shared HREADY; HWDATA is the FIFO head
//     (combinational), valid on the write address-acceptance cycle.
//   * the bus is re-requested between the read and write phase of every beat,
//     so the CPU is never locked out during long copies.
//
// Word (32-bit) transfers only, addresses word-aligned, HBURST = SINGLE.
// Plain Verilog (IEEE 1364-2001).
//============================================================================

module dma8_ahb_master (
  input  wire          clk,
  input  wire          rstn,

  // from the stream arbiter (one-hot)
  input  wire [7:0]    grant,

  // stream configuration (from the APB slave), 8 x 32-bit packed
  input  wire [255:0]  src_addr_flat,
  input  wire [255:0]  dest_addr_flat,
  input  wire [255:0]  count_flat,
  output reg  [7:0]    transfer_done,

  // internal buffer FIFO
  input  wire          fifo_full,
  input  wire          fifo_empty,
  input  wire [31:0]   fifo_rdata,
  output reg           fifo_w_en,
  output reg           fifo_r_en,
  output wire [31:0]   fifo_wdata,

  // AHB-Lite master (fabric convention)
  input  wire          HGRANT,
  output wire          HBUSREQ,
  input  wire          HREADY,
  input  wire [31:0]   HRDATA,
  output wire [31:0]   HWDATA,
  output wire [2:0]    HSIZE,
  output reg  [31:0]   HADDR,
  output reg  [1:0]    HTRANS,
  output wire [2:0]    HBURST,
  output reg           HWRITE
);

  // fixed constraints: 32-bit words, SINGLE bursts
  assign HSIZE  = 3'b010;
  assign HBURST = 3'b000;

  // read data is registered at data-phase completion (the fabric response
  // mux clears HRDATA one cycle later, so it must not feed the FIFO
  // directly); write data is the FIFO head, live during the address phase
  reg [31:0] read_cap_q;
  assign fifo_wdata = read_cap_q;
  assign HWDATA     = fifo_rdata;

  localparam [3:0] S_IDLE      = 4'd0;
  localparam [3:0] S_REQ_R     = 4'd1; // request fabric for the read phase
  localparam [3:0] S_READ_SET  = 4'd2; // register HADDR/HTRANS (bus still IDLE)
  localparam [3:0] S_READ_ADDR = 4'd3; // ADDRESS phase: NONSEQ visible 1 cycle
  localparam [3:0] S_READ_DATA = 4'd4; // DATA phase: wait HREADY, capture FIFO
  localparam [3:0] S_REQ_W     = 4'd5; // request fabric for the write phase
  localparam [3:0] S_WRITE_SET = 4'd6; // register HADDR/HTRANS (bus still IDLE)
  localparam [3:0] S_WRITE_ADDR= 4'd7; // ADDRESS phase: NONSEQ + HWDATA live
  localparam [3:0] S_WRITE_DATA= 4'd8; // DATA phase: wait HREADY
  localparam [3:0] S_CHECK     = 4'd9;

  reg [3:0]  state_q;
  reg [2:0]  active_ch_q;
  reg [31:0] current_src_q;
  reg [31:0] current_dest_q;
  reg [31:0] current_count_q;

  // request the fabric for the address phases only; BUSREQ is deasserted
  // during the data phases so the fabric can interleave other masters
  // (the data phase is protected by the fabric's d_sel_q tracking, and the
  // grant-persistence rule covers the request->presentation gap)
  assign HBUSREQ = (state_q == S_REQ_R)     || (state_q == S_READ_SET)  ||
                   (state_q == S_READ_ADDR) || (state_q == S_REQ_W)     ||
                   (state_q == S_WRITE_SET) || (state_q == S_WRITE_ADDR);

  // ---- stream configurator (one-hot grant selects the lane) ----
  reg [31:0] next_src, next_dest, next_count;
  reg [2:0]  next_ch;
  integer    m;

  always @(*) begin
    next_src   = 32'd0;
    next_dest  = 32'd0;
    next_count = 32'd0;
    next_ch    = 3'd0;
    for (m = 0; m < 8; m = m + 1) begin
      if (grant[m]) begin
        next_src   = src_addr_flat[m*32 +: 32];
        next_dest  = dest_addr_flat[m*32 +: 32];
        next_count = count_flat[m*32 +: 32];
        next_ch    = m[2:0];
      end
    end
  end

  wire [7:0] active_onehot_c = 8'b0000_0001 << active_ch_q;

  // ---- main FSM (identical to the proven 4-channel engine) ----
  always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      state_q        <= S_IDLE;
      read_cap_q     <= 32'd0;
      active_ch_q    <= 3'd0;
      current_src_q  <= 32'd0;
      current_dest_q <= 32'd0;
      current_count_q<= 32'd0;
      HADDR          <= 32'd0;
      HWRITE         <= 1'b0;
      HTRANS         <= 2'b00;
      fifo_w_en      <= 1'b0;
      fifo_r_en      <= 1'b0;
      transfer_done  <= 8'h00;
    end else begin
      // default: IDLE on the bus, no FIFO motion, no done pulse
      HTRANS    <= 2'b00;
      fifo_w_en <= 1'b0;
      fifo_r_en <= 1'b0;
      transfer_done <= 8'h00;

      case (state_q)
        S_IDLE: begin
          if (grant != 8'h00) begin
            current_src_q   <= next_src;
            current_dest_q  <= next_dest;
            current_count_q <= next_count;
            active_ch_q     <= next_ch;
            state_q         <= S_REQ_R;
          end
        end

        S_REQ_R: begin
          if (HGRANT) state_q <= S_READ_SET;
        end

        // HADDR/HTRANS are registered here and become visible during
        // S_READ_ADDR -- that cycle is the real AHB address phase.
        S_READ_SET: begin
          if (!fifo_full) begin
            HADDR  <= current_src_q;
            HWRITE <= 1'b0;
            HTRANS <= 2'b10;          // NONSEQ for exactly one cycle
            state_q <= S_READ_ADDR;
          end
        end

        // ADDRESS phase: NONSEQ is held until HREADY=1 accepts it (another
        // master's data phase may still be in flight: HREADY=0 stretches
        // this phase -- dropping NONSEQ early would lose the transfer).
        S_READ_ADDR: begin
          if (HREADY) state_q <= S_READ_DATA;
          else        HTRANS  <= 2'b10;   // stretch: keep NONSEQ asserted
        end

        // DATA phase: our slave is now selected; HREADY=1 ends it.
        S_READ_DATA: begin
          if (HREADY) begin
            read_cap_q    <= HRDATA;  // register the beat...
            fifo_w_en     <= 1'b1;    // ...and push it next cycle
            current_src_q <= current_src_q + 32'd4;
            state_q       <= S_REQ_W;
          end
        end

        S_REQ_W: begin
          if (HGRANT && !fifo_empty) state_q <= S_WRITE_SET;
        end

        S_WRITE_SET: begin
          HADDR   <= current_dest_q;
          HWRITE  <= 1'b1;
          HTRANS  <= 2'b10;           // visible during S_WRITE_ADDR;
                                      // HWDATA = FIFO head, live then
          state_q <= S_WRITE_ADDR;
        end

        // ADDRESS phase: held until accepted; HWDATA (FIFO head) is live
        // and the slave latches it on the acceptance edge.
        S_WRITE_ADDR: begin
          if (HREADY) begin
            fifo_r_en <= !fifo_empty; // pop lands after the slave latched
            state_q   <= S_WRITE_DATA;
          end else begin
            HTRANS    <= 2'b10;       // stretch: keep NONSEQ asserted
          end
        end

        S_WRITE_DATA: begin
          if (HREADY) begin
            current_dest_q  <= current_dest_q + 32'd4;
            current_count_q <= current_count_q - 32'd1;
            state_q         <= S_CHECK;
          end
        end

        S_CHECK: begin
          if (current_count_q == 32'd0) begin
            transfer_done[active_ch_q] <= 1'b1;
            state_q <= S_IDLE;
          end else if (grant != active_onehot_c) begin
            state_q <= S_IDLE;        // arbiter moved on: reload config
          end else begin
            state_q <= S_REQ_R;       // next beat of the same stream
          end
        end

        default: state_q <= S_IDLE;
      endcase
    end
  end

endmodule
