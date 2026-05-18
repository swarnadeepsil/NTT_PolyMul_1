// ============================================================
// ntt_core.v  -  NTT / INTT processor  (RTL-to-GDSII ready)
//
// Architecture (verified against naive O(n²) reference):
//   Forward NTT : DIF  - GS butterfly, natural input, bit-reversed output
//   Inverse NTT : DIT  - CT butterfly, bit-reversed input, natural output
//
// The DIF forward output feeds directly into DIT inverse.
// The bit-reversal permutation cancels between the two stages,
// so NO explicit bit-reversal hardware is needed.
//
// Reference: EE522L Project Report, IIT Tirupati, May 2026.
//
// Ports:
//   clk/rst   : 100 MHz, synchronous active-high reset
//   load_en   : pulse high + set load_addr/data_in to pre-load coefficients
//   start     : pulse 1 cycle to begin; latch inverse flag at this edge
//   inverse   : 0 = forward NTT (DIF),  1 = inverse NTT (DIT)
//   modulus   : prime q
//   omega     : ω   for forward,  ω⁻¹ for inverse  (from Python)
//   n_inv     : n⁻¹ mod q  (used only in INTT scaling stage)
//   done      : pulses high 1 cycle when output is valid
//   data_out  : read output coefficient selected by out_addr
//
// Cycle budget (8-point example):
//   Twiddle precompute : N-1  = 7   cycles
//   Butterfly stages   : 3×4  = 12  cycles
//   INTT scaling       : N    = 8   cycles  (forward skips)
//   Total forward      :           20  cycles
//   Total inverse      :           28  cycles
// ============================================================

`timescale 1ns/1ps
`default_nettype none

module ntt_core #(
    parameter N_LEN  = 8,
    parameter LOG2N  = 3,
    parameter DATA_W = 32
)(
    input  wire                 clk,
    input  wire                 rst,

    // load interface
    input  wire [DATA_W-1:0]    data_in,
    input  wire                 load_en,
    input  wire [LOG2N-1:0]     load_addr,

    // NTT parameters
    input  wire [DATA_W-1:0]    modulus,
    input  wire [DATA_W-1:0]    omega,     // ω (fwd) or ω⁻¹ (inv)
    input  wire [DATA_W-1:0]    n_inv,     // n⁻¹ mod q

    // control
    input  wire                 start,
    input  wire                 inverse,   // 0=NTT, 1=INTT

    // output
    output reg                  done,
    output wire [DATA_W-1:0]    data_out,
    input  wire [LOG2N-1:0]     out_addr
);

    // ----------------------------------------------------------
    // Memory
    // ----------------------------------------------------------
    reg [DATA_W-1:0] mem     [0:N_LEN-1];
    reg [DATA_W-1:0] twiddle [0:N_LEN-1];  // omega^i mod q

    // ----------------------------------------------------------
    // FSM states
    // ----------------------------------------------------------
    localparam [2:0]
        S_IDLE    = 3'd0,
        S_TWIDDLE = 3'd1,
        S_COMPUTE = 3'd2,
        S_SCALE   = 3'd3,
        S_DONE    = 3'd4;

    reg [2:0]           state;
    reg [LOG2N-1:0]     stage;
    reg [LOG2N-1:0]     bfly_cnt;
    reg [LOG2N-1:0]     tw_idx;
    reg [LOG2N-1:0]     scale_idx;
    reg                 inv_reg;

    // ----------------------------------------------------------
    // Read port
    // ----------------------------------------------------------
    assign data_out = mem[out_addr];

    // ----------------------------------------------------------
    // Butterfly index computation (combinational)
    //
    // DIF (forward, inv_reg=0) - GS butterfly:
    //   half      = N >> (stage+1)
    //   log_step  = LOG2N-1-stage
    //   group     = bfly_cnt >> log_step   (= bfly_cnt / half)
    //   k         = bfly_cnt & (half-1)    (= bfly_cnt % half)
    //   idx_i     = (group << (LOG2N-stage)) | k
    //   idx_j     = idx_i | half
    //   tw_sel    = k << stage             (= k * 2^stage)
    //
    // DIT (inverse, inv_reg=1) - CT butterfly:
    //   half      = 1 << stage
    //   group     = bfly_cnt >> stage      (= bfly_cnt / half)
    //   k         = bfly_cnt & (half-1)    (= bfly_cnt % half)
    //   idx_i     = (group << (stage+1)) | k
    //   idx_j     = idx_i | half
    //   tw_sel    = k * (N >> (stage+1))   (= k * step)
    // ----------------------------------------------------------
    reg [LOG2N-1:0] idx_i, idx_j, tw_sel;

    always @(*) begin : idx_calc
        reg [LOG2N-1:0] half;
        reg [LOG2N-1:0] group;
        reg [LOG2N-1:0] k;
        reg [LOG2N-1:0] log_step;
        reg [LOG2N:0]   step;

        idx_i    = {LOG2N{1'b0}};
        idx_j    = {LOG2N{1'b0}};
        tw_sel   = {LOG2N{1'b0}};

        if (!inv_reg) begin
            // ── DIF (GS)  forward ─────────────────────────
            half     = N_LEN >> (stage + 1'b1);
            log_step = (LOG2N - 1'b1) - stage;
            group    = (half == {LOG2N{1'b0}}) ? bfly_cnt
                                                : bfly_cnt >> log_step;
            k        = (half == {LOG2N{1'b0}}) ? {LOG2N{1'b0}}
                                                : bfly_cnt & (half - 1'b1);
            idx_i    = (group << (LOG2N - stage)) | k;
            idx_j    = idx_i | half;
            tw_sel   = k << stage;
        end else begin
            // ── DIT (CT)  inverse ─────────────────────────
            half   = 1'b1 << stage;
            group  = bfly_cnt >> stage;
            k      = bfly_cnt & (half - 1'b1);
            idx_i  = (group << (stage + 1'b1)) | k;
            idx_j  = idx_i | half;
            step   = N_LEN >> (stage + 1'b1);
            tw_sel = k * step[LOG2N-1:0];
        end
    end

    // ----------------------------------------------------------
    // Butterfly data wires
    // ----------------------------------------------------------
    wire [DATA_W-1:0] a_val  = mem[idx_i];
    wire [DATA_W-1:0] b_val  = mem[idx_j];
    wire [DATA_W-1:0] tw_val = twiddle[tw_sel];

    // GS butterfly outputs  (DIF / forward)
    wire [DATA_W-1:0] x_gs, y_gs;
    gs_butterfly #(.DATA_W(DATA_W)) u_gs (
        .a(a_val), .b(b_val), .w(tw_val), .q(modulus),
        .x(x_gs),  .y(y_gs)
    );

    // CT butterfly outputs  (DIT / inverse)
    wire [DATA_W-1:0] x_ct, y_ct;
    ct_butterfly #(.DATA_W(DATA_W)) u_ct (
        .a(a_val), .b(b_val), .w(tw_val), .q(modulus),
        .x(x_ct),  .y(y_ct)
    );

    // Mux: forward uses GS, inverse uses CT
    wire [DATA_W-1:0] x_out = inv_reg ? x_ct : x_gs;
    wire [DATA_W-1:0] y_out = inv_reg ? y_ct : y_gs;

    // ----------------------------------------------------------
    // Twiddle precompute multiplier
    // twiddle[tw_idx] = twiddle[tw_idx-1] * omega mod q
    // ----------------------------------------------------------
    wire [DATA_W-1:0] tw_prev  = twiddle[(tw_idx == 0) ? 0 : tw_idx - 1'b1];
    wire [DATA_W-1:0] tw_next;
    mod_multiplier #(.DATA_W(DATA_W)) u_tw (
        .a(tw_prev), .b(omega), .q(modulus), .result(tw_next)
    );

    // ----------------------------------------------------------
    // INTT scaling multiplier
    // mem[scale_idx] = mem[scale_idx] * n_inv mod q
    // ----------------------------------------------------------
    wire [DATA_W-1:0] scaled_val;
    mod_multiplier #(.DATA_W(DATA_W)) u_sc (
        .a(mem[scale_idx]), .b(n_inv), .q(modulus), .result(scaled_val)
    );

    // ----------------------------------------------------------
    // Sequential FSM
    // ----------------------------------------------------------
    integer ii;
    always @(posedge clk) begin
        if (rst) begin
            state     <= S_IDLE;
            done      <= 1'b0;
            stage     <= {LOG2N{1'b0}};
            bfly_cnt  <= {LOG2N{1'b0}};
            tw_idx    <= {LOG2N{1'b0}};
            scale_idx <= {LOG2N{1'b0}};
            inv_reg   <= 1'b0;
            for (ii = 0; ii < N_LEN; ii = ii + 1)
                mem[ii] <= {DATA_W{1'b0}};
        end else begin
            done <= 1'b0;

            case (state)

                // ── IDLE ─────────────────────────────────────
                S_IDLE: begin
                    if (load_en)
                        mem[load_addr] <= data_in;

                    if (start) begin
                        inv_reg    <= inverse;
                        twiddle[0] <= {{(DATA_W-1){1'b0}}, 1'b1}; // omega^0 = 1
                        tw_idx     <= {{(LOG2N-1){1'b0}}, 1'b1};  // start at index 1
                        stage      <= {LOG2N{1'b0}};
                        state      <= S_TWIDDLE;
                    end
                end

                // ── TWIDDLE PRECOMPUTE ────────────────────────
                // Each cycle: twiddle[tw_idx] = twiddle[tw_idx-1] * omega mod q
                S_TWIDDLE: begin
                    twiddle[tw_idx] <= tw_next;
                    if (tw_idx == N_LEN - 1) begin
                        stage    <= {LOG2N{1'b0}};
                        bfly_cnt <= {LOG2N{1'b0}};
                        state    <= S_COMPUTE;
                    end else begin
                        tw_idx <= tw_idx + 1'b1;
                    end
                end

                // ── BUTTERFLY STAGES ──────────────────────────
                // LOG2N stages × N/2 pairs each
                S_COMPUTE: begin
                    mem[idx_i] <= x_out;
                    mem[idx_j] <= y_out;

                    if (bfly_cnt == (N_LEN/2) - 1) begin
                        bfly_cnt <= {LOG2N{1'b0}};
                        if (stage == LOG2N - 1) begin
                            if (inv_reg) begin
                                scale_idx <= {LOG2N{1'b0}};
                                state     <= S_SCALE;
                            end else
                                state <= S_DONE;
                        end else
                            stage <= stage + 1'b1;
                    end else
                        bfly_cnt <= bfly_cnt + 1'b1;
                end

                // ── INTT SCALING ──────────────────────────────
                // Multiply each element by n_inv mod q
                S_SCALE: begin
                    mem[scale_idx] <= scaled_val;
                    if (scale_idx == N_LEN - 1)
                        state <= S_DONE;
                    else
                        scale_idx <= scale_idx + 1'b1;
                end

                // ── DONE ─────────────────────────────────────
                S_DONE: begin
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire