`timescale 1ns/1ps
// ============================================================
// ntt_butterfly.v
//
// Butterfly submodules for NTT hardware:
//   ct_butterfly    - Cooley-Tukey (forward NTT)
//   gs_butterfly    - Gentleman-Sande (inverse NTT)
//   mod_multiplier  - (a * b) mod q
//   mod_adder       - (a + b) mod q
//   mod_subtractor  - (a - b) mod q
// ============================================================

`default_nettype none

// ------------------------------------------------------------
// CT (Cooley-Tukey) Butterfly
//
//   X = (a + b*w) mod q
//   Y = (a - b*w) mod q
// ------------------------------------------------------------
module ct_butterfly #(
    parameter DATA_W = 32
)(
    input  wire [DATA_W-1:0]  a,
    input  wire [DATA_W-1:0]  b,
    input  wire [DATA_W-1:0]  w,
    input  wire [DATA_W-1:0]  q,
    output wire [DATA_W-1:0]  x,
    output wire [DATA_W-1:0]  y
);
    wire [2*DATA_W-1:0] bw_full;
    wire [DATA_W-1:0]   bw;
    assign bw_full = b * w;
    assign bw      = bw_full % q;

    wire [DATA_W:0] sum;
    assign sum = {1'b0, a} + {1'b0, bw};
    assign x   = (sum >= {1'b0, q}) ? sum[DATA_W-1:0] - q : sum[DATA_W-1:0];
    assign y   = (a >= bw)          ? (a - bw)            : (a + q - bw);
endmodule


// ------------------------------------------------------------
// GS (Gentleman-Sande) Butterfly
//
//   X = (a + b) mod q
//   Y = (a - b)*w mod q
// ------------------------------------------------------------
module gs_butterfly #(
    parameter DATA_W = 32
)(
    input  wire [DATA_W-1:0]  a,
    input  wire [DATA_W-1:0]  b,
    input  wire [DATA_W-1:0]  w,
    input  wire [DATA_W-1:0]  q,
    output wire [DATA_W-1:0]  x,
    output wire [DATA_W-1:0]  y
);
    wire [DATA_W:0]     sum;
    wire [DATA_W-1:0]   diff;
    wire [2*DATA_W-1:0] yw_full;

    assign sum     = {1'b0, a} + {1'b0, b};
    assign x       = (sum >= {1'b0, q}) ? sum[DATA_W-1:0] - q : sum[DATA_W-1:0];
    assign diff    = (a >= b)           ? (a - b)              : (a + q - b);
    assign yw_full = diff * w;
    assign y       = yw_full % q;
endmodule


// ------------------------------------------------------------
// Modular Multiplier:  result = (a * b) mod q
// ------------------------------------------------------------
module mod_multiplier #(
    parameter DATA_W = 32
)(
    input  wire [DATA_W-1:0]  a,
    input  wire [DATA_W-1:0]  b,
    input  wire [DATA_W-1:0]  q,
    output wire [DATA_W-1:0]  result
);
    wire [2*DATA_W-1:0] full;
    assign full   = a * b;
    assign result = full % q;
endmodule


// ------------------------------------------------------------
// Modular Adder:  result = (a + b) mod q
// ------------------------------------------------------------
module mod_adder #(
    parameter DATA_W = 32
)(
    input  wire [DATA_W-1:0]  a,
    input  wire [DATA_W-1:0]  b,
    input  wire [DATA_W-1:0]  q,
    output wire [DATA_W-1:0]  result
);
    wire [DATA_W:0] sum;
    assign sum    = {1'b0, a} + {1'b0, b};
    assign result = (sum >= {1'b0, q}) ? sum[DATA_W-1:0] - q : sum[DATA_W-1:0];
endmodule


// ------------------------------------------------------------
// Modular Subtractor:  result = (a - b) mod q
// ------------------------------------------------------------
module mod_subtractor #(
    parameter DATA_W = 32
)(
    input  wire [DATA_W-1:0]  a,
    input  wire [DATA_W-1:0]  b,
    input  wire [DATA_W-1:0]  q,
    output wire [DATA_W-1:0]  result
);
    assign result = (a >= b) ? (a - b) : (a + q - b);
endmodule

`default_nettype wire