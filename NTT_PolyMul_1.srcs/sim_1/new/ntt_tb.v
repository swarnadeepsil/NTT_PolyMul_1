// ============================================================
// ntt_tb.v  -  Testbench for ntt_core + ntt_butterfly
//
// Compatible: Verilog-2005 / Vivado xsim / ModelSim / iverilog
//
// Architecture under test:
//   Forward NTT : DIF (GS butterfly) - natural input, bit-reversed output
//   Inverse NTT : DIT (CT butterfly) - bit-reversed input, natural output
//   No explicit bit-reversal needed between the two stages.
//
// Test flow:
//   1. Load NTT parameters from ntt_params.txt  (written by Python)
//   2. TEST 1 - Forward NTT of A,  compare vs Python A_ntt (DIF order)
//   3. TEST 2 - Forward NTT of B,  compare vs Python B_ntt (DIF order)
//   4. Pointwise multiply A_ntt * B_ntt mod q  (inside testbench)
//   5. TEST 3 - INTT of product,   compare vs brute-force polynomial result
//   6. Print full PASS / FAIL summary
//
// To simulate (iverilog):
//   iverilog -g2005 -o ntt_sim ntt_butterfly.v ntt_core.v ntt_tb.v
//   vvp ntt_sim
//
// To simulate (Vivado xsim):
//   Add all three .v files; set ntt_tb as top; run simulation.
//   ntt_params.txt must be in the simulation working directory.
// ============================================================

`timescale 1ns/1ps

module ntt_tb;

    // ----------------------------------------------------------
    // Parameters - must match ntt_core instantiation below
    // ----------------------------------------------------------
    parameter N_LEN    = 8;
    parameter LOG2N    = 3;
    parameter DATA_W   = 32;
    parameter CLK_HALF = 5;   // 10 ns period = 100 MHz

    // ----------------------------------------------------------
    // DUT signals
    // ----------------------------------------------------------
    reg                  clk;
    reg                  rst;
    reg  [DATA_W-1:0]    data_in;
    reg                  load_en;
    reg  [LOG2N-1:0]     load_addr;
    reg  [DATA_W-1:0]    modulus;
    reg  [DATA_W-1:0]    omega;
    reg  [DATA_W-1:0]    n_inv;
    reg                  start;
    reg                  inverse;
    wire                 done;
    wire [DATA_W-1:0]    data_out;
    reg  [LOG2N-1:0]     out_addr;

    // ----------------------------------------------------------
    // DUT instantiation
    // ----------------------------------------------------------
    ntt_core #(
        .N_LEN  (N_LEN),
        .LOG2N  (LOG2N),
        .DATA_W (DATA_W)
    ) dut (
        .clk       (clk),
        .rst       (rst),
        .data_in   (data_in),
        .load_en   (load_en),
        .load_addr (load_addr),
        .modulus   (modulus),
        .omega     (omega),
        .n_inv     (n_inv),
        .start     (start),
        .inverse   (inverse),
        .done      (done),
        .data_out  (data_out),
        .out_addr  (out_addr)
    );

    // ----------------------------------------------------------
    // Clock generation
    // ----------------------------------------------------------
    initial clk = 0;
    always  #CLK_HALF clk = ~clk;

    // ----------------------------------------------------------
    // Data arrays (module-scope - tasks share these)
    // ----------------------------------------------------------
    reg [DATA_W-1:0] A_coef    [0:N_LEN-1]; // input A (padded)
    reg [DATA_W-1:0] B_coef    [0:N_LEN-1]; // input B (padded)
    reg [DATA_W-1:0] C_coef    [0:N_LEN-1]; // pointwise product
    reg [DATA_W-1:0] A_ntt     [0:N_LEN-1]; // captured NTT(A)
    reg [DATA_W-1:0] B_ntt     [0:N_LEN-1]; // captured NTT(B)
    reg [DATA_W-1:0] result    [0:N_LEN-1]; // captured INTT output
    reg [DATA_W-1:0] A_ntt_ref [0:N_LEN-1]; // Python reference NTT(A)
    reg [DATA_W-1:0] B_ntt_ref [0:N_LEN-1]; // Python reference NTT(B)
    reg [DATA_W-1:0] C_ref     [0:N_LEN-1]; // Python reference C_ntt
    reg [DATA_W-1:0] expected  [0:N_LEN-1]; // brute-force poly product

    // ----------------------------------------------------------
    // Saved omega values
    // ----------------------------------------------------------
    reg [DATA_W-1:0] omega_fwd;
    reg [DATA_W-1:0] omega_inv_r;
    reg [DATA_W-1:0] n_inv_r;

    // ----------------------------------------------------------
    // Counters and loop vars
    // ----------------------------------------------------------
    integer pass_count;
    integer fail_count;
    integer j;
    integer tout;

    // ----------------------------------------------------------
    // File I/O vars
    // ----------------------------------------------------------
    integer         fid;
    reg [8*16-1:0]  tag;
    integer         idx_r;
    integer         val_r;
    integer         scan_ret;

    // ══════════════════════════════════════════════════════════
    // TASKS
    // ══════════════════════════════════════════════════════════

    // --- reset DUT -------------------------------------------
    task do_reset;
        begin
            rst     = 1; start = 0; load_en = 0;
            repeat(4) @(posedge clk);
            @(negedge clk); rst = 0;
        end
    endtask

    // --- load A_coef into DUT --------------------------------
    task load_A;
        begin
            for (j = 0; j < N_LEN; j = j + 1) begin
                @(negedge clk);
                load_en = 1; load_addr = j[LOG2N-1:0]; data_in = A_coef[j];
            end
            @(negedge clk); load_en = 0;
        end
    endtask

    // --- load B_coef into DUT --------------------------------
    task load_B;
        begin
            for (j = 0; j < N_LEN; j = j + 1) begin
                @(negedge clk);
                load_en = 1; load_addr = j[LOG2N-1:0]; data_in = B_coef[j];
            end
            @(negedge clk); load_en = 0;
        end
    endtask

    // --- load C_coef into DUT --------------------------------
    task load_C;
        begin
            for (j = 0; j < N_LEN; j = j + 1) begin
                @(negedge clk);
                load_en = 1; load_addr = j[LOG2N-1:0]; data_in = C_coef[j];
            end
            @(negedge clk); load_en = 0;
        end
    endtask

    // --- start transform and wait for done ------------------
    task run_transform;
        input inv_flag;
        begin
            @(negedge clk); inverse = inv_flag; start = 1;
            @(negedge clk); start = 0;
            tout = 0;
            while (done !== 1'b1 && tout < 5000) begin
                @(posedge clk); tout = tout + 1;
            end
            if (tout >= 5000)
                $display("  [TIMEOUT] Transform did not finish in 5000 cycles!");
            @(posedge clk); // extra settle cycle
        end
    endtask

    // --- capture DUT output into A_ntt[] -------------------
    task capture_A_ntt;
        begin
            for (j = 0; j < N_LEN; j = j + 1) begin
                out_addr = j[LOG2N-1:0]; #2; A_ntt[j] = data_out;
            end
        end
    endtask

    // --- capture DUT output into B_ntt[] -------------------
    task capture_B_ntt;
        begin
            for (j = 0; j < N_LEN; j = j + 1) begin
                out_addr = j[LOG2N-1:0]; #2; B_ntt[j] = data_out;
            end
        end
    endtask

    // --- capture DUT output into result[] ------------------
    task capture_result;
        begin
            for (j = 0; j < N_LEN; j = j + 1) begin
                out_addr = j[LOG2N-1:0]; #2; result[j] = data_out;
            end
        end
    endtask

    // --- print A_ntt[] vs A_ntt_ref[] ----------------------
    task check_A_ntt;
        integer lf;
        begin
            lf = 0;
            $display("  %4s  %10s  %10s  %s", "idx", "got", "expected", "status");
            $display("  %s","------------------------------------------");
            for (j = 0; j < N_LEN; j = j + 1) begin
                if (A_ntt[j] === A_ntt_ref[j]) begin
                    $display("  [%0d]  %10d  %10d    OK", j, A_ntt[j], A_ntt_ref[j]);
                    pass_count = pass_count + 1;
                end else begin
                    $display("  [%0d]  %10d  %10d    FAIL <<<", j, A_ntt[j], A_ntt_ref[j]);
                    fail_count = fail_count + 1; lf = lf + 1;
                end
            end
            $display("  TEST 1 : %s", (lf==0) ? "PASS" : "FAIL");
        end
    endtask

    // --- print B_ntt[] vs B_ntt_ref[] ----------------------
    task check_B_ntt;
        integer lf;
        begin
            lf = 0;
            $display("  %4s  %10s  %10s  %s", "idx", "got", "expected", "status");
            $display("  %s","------------------------------------------");
            for (j = 0; j < N_LEN; j = j + 1) begin
                if (B_ntt[j] === B_ntt_ref[j]) begin
                    $display("  [%0d]  %10d  %10d    OK", j, B_ntt[j], B_ntt_ref[j]);
                    pass_count = pass_count + 1;
                end else begin
                    $display("  [%0d]  %10d  %10d    FAIL <<<", j, B_ntt[j], B_ntt_ref[j]);
                    fail_count = fail_count + 1; lf = lf + 1;
                end
            end
            $display("  TEST 2 : %s", (lf==0) ? "PASS" : "FAIL");
        end
    endtask

    // --- print result[] vs expected[] ----------------------
    task check_result;
        integer lf;
        begin
            lf = 0;
            $display("  %4s  %10s  %10s  %s", "idx", "got", "expected", "status");
            $display("  %s","------------------------------------------");
            for (j = 0; j < N_LEN; j = j + 1) begin
                if (result[j] === expected[j]) begin
                    $display("  [%0d]  %10d  %10d    OK", j, result[j], expected[j]);
                    pass_count = pass_count + 1;
                end else begin
                    $display("  [%0d]  %10d  %10d    FAIL <<<", j, result[j], expected[j]);
                    fail_count = fail_count + 1; lf = lf + 1;
                end
            end
            $display("  TEST 3 : %s", (lf==0) ? "PASS" : "FAIL");
        end
    endtask

    // --- read ntt_params.txt --------------------------------
    task read_params;
        begin
            fid = $fopen("C:\Users\swarn\Desktop\Verilog_Vivado\NTT_PolyMul_1\ntt_params.txt", "r");
            if (fid == 0) begin
                $display("[TB] ntt_params.txt not found - using built-in defaults.");
            end else begin
                $display("[TB] Reading ntt_params.txt ...");
                while (!$feof(fid)) begin
                    scan_ret = $fscanf(fid, "%s", tag);
                    if (scan_ret < 1) begin end
                    else if (tag == "MODULUS")   begin scan_ret=$fscanf(fid,"%d",modulus);     end
                    else if (tag == "OMEGA")     begin scan_ret=$fscanf(fid,"%d",omega_fwd);   end
                    else if (tag == "OMEGA_INV") begin scan_ret=$fscanf(fid,"%d",omega_inv_r); end
                    else if (tag == "N_INV")     begin scan_ret=$fscanf(fid,"%d",n_inv_r);     end
                    else if (tag == "A") begin
                        scan_ret=$fscanf(fid,"%d %d",idx_r,val_r);
                        if(idx_r<N_LEN) A_coef[idx_r]=val_r[DATA_W-1:0];
                    end
                    else if (tag == "B") begin
                        scan_ret=$fscanf(fid,"%d %d",idx_r,val_r);
                        if(idx_r<N_LEN) B_coef[idx_r]=val_r[DATA_W-1:0];
                    end
                    else if (tag == "ANTT") begin
                        scan_ret=$fscanf(fid,"%d %d",idx_r,val_r);
                        if(idx_r<N_LEN) A_ntt_ref[idx_r]=val_r[DATA_W-1:0];
                    end
                    else if (tag == "BNTT") begin
                        scan_ret=$fscanf(fid,"%d %d",idx_r,val_r);
                        if(idx_r<N_LEN) B_ntt_ref[idx_r]=val_r[DATA_W-1:0];
                    end
                    else if (tag == "CNTT") begin
                        scan_ret=$fscanf(fid,"%d %d",idx_r,val_r);
                        if(idx_r<N_LEN) C_ref[idx_r]=val_r[DATA_W-1:0];
                    end
                    else if (tag == "EXP") begin
                        scan_ret=$fscanf(fid,"%d %d",idx_r,val_r);
                        if(idx_r<N_LEN) expected[idx_r]=val_r[DATA_W-1:0];
                    end
                end
                $fclose(fid);
                $display("[TB] Loaded: modulus=%0d  omega=%0d  omega_inv=%0d  n_inv=%0d",
                         modulus,omega_fwd,omega_inv_r,n_inv_r);
            end
        end
    endtask

    // ══════════════════════════════════════════════════════════
    // MAIN TEST SEQUENCE
    // ══════════════════════════════════════════════════════════
    integer pw_tmp;
    reg [2*DATA_W-1:0] pw_prod;

    initial begin
        $dumpfile("ntt_tb.vcd");
        $dumpvars(0, ntt_tb);

        // ── initialise signals ────────────────────────────────
        rst=1; start=0; load_en=0; load_addr=0;
        data_in=0; inverse=0; out_addr=0;
        omega=0; n_inv=0;
        pass_count=0; fail_count=0;

        // ── built-in defaults (DIF reference values) ─────────
        modulus    = 32'd89;
        omega_fwd  = 32'd37;
        omega_inv_r= 32'd77;
        n_inv_r    = 32'd78;

        A_coef[0]=1; A_coef[1]=2; A_coef[2]=3; A_coef[3]=4;
        A_coef[4]=0; A_coef[5]=0; A_coef[6]=0; A_coef[7]=0;

        B_coef[0]=5; B_coef[1]=4; B_coef[2]=3; B_coef[3]=2;
        B_coef[4]=0; B_coef[5]=0; B_coef[6]=0; B_coef[7]=0;

        // DIF-order NTT(A) reference
        A_ntt_ref[0]=10; A_ntt_ref[1]=87; A_ntt_ref[2]=19; A_ntt_ref[3]=66;
        A_ntt_ref[4]=47; A_ntt_ref[5]=70; A_ntt_ref[6]=71; A_ntt_ref[7]=83;

        // DIF-order NTT(B) reference
        B_ntt_ref[0]=14; B_ntt_ref[1]=2;  B_ntt_ref[2]=70; B_ntt_ref[3]=23;
        B_ntt_ref[4]=12; B_ntt_ref[5]=24; B_ntt_ref[6]=25; B_ntt_ref[7]=48;

        // C_ntt reference (pointwise)
        C_ref[0]=51; C_ref[1]=85; C_ref[2]=84; C_ref[3]=5;
        C_ref[4]=30; C_ref[5]=78; C_ref[6]=84; C_ref[7]=68;

        // (1+2x+3x^2+4x^3)(5+4x+3x^2+2x^3) = 5+14x+26x^2+40x^3+29x^4+18x^5+8x^6
        expected[0]=5;  expected[1]=14; expected[2]=26; expected[3]=40;
        expected[4]=29; expected[5]=18; expected[6]=8;  expected[7]=0;

        // ── override from file ────────────────────────────────
        read_params;

        // ── banner ────────────────────────────────────────────
        $display("");
        $display("============================================================");
        $display("  NTT Core Testbench");
        $display("  modulus=%0d  omega=%0d  omega_inv=%0d  n_inv=%0d",
                 modulus, omega_fwd, omega_inv_r, n_inv_r);
        $display("============================================================");

        // ── show inputs as arrays ─────────────────────────────
        $write("  Input  A = [");
        for (j = 0; j < N_LEN; j = j + 1) begin
            if (j < N_LEN-1) $write("%0d, ", A_coef[j]);
            else              $write("%0d",   A_coef[j]);
        end
        $display("]");

        $write("  Input  B = [");
        for (j = 0; j < N_LEN; j = j + 1) begin
            if (j < N_LEN-1) $write("%0d, ", B_coef[j]);
            else              $write("%0d",   B_coef[j]);
        end
        $display("]");
        $display("============================================================");

        // ═════════════════════════════════════════════════════
        // TEST 1 - Forward NTT of A  (DIF)
        // ═════════════════════════════════════════════════════
        $display("\n[TEST 1]  NTT(A)  -  DIF (GS butterfly)");
        do_reset;
        omega = omega_fwd;
        n_inv = n_inv_r;
        load_A;
        run_transform(1'b0);
        capture_A_ntt;

        // print NTT(A) as array
        $write("  NTT(A) = [");
        for (j = 0; j < N_LEN; j = j + 1) begin
            if (j < N_LEN-1) $write("%0d, ", A_ntt[j]);
            else              $write("%0d",   A_ntt[j]);
        end
        $display("]");

        check_A_ntt;

        // ═════════════════════════════════════════════════════
        // TEST 2 - Forward NTT of B  (DIF)
        // ═════════════════════════════════════════════════════
        $display("\n[TEST 2]  NTT(B)  -  DIF (GS butterfly)");
        do_reset;
        omega = omega_fwd;
        n_inv = n_inv_r;
        load_B;
        run_transform(1'b0);
        capture_B_ntt;

        // print NTT(B) as array
        $write("  NTT(B) = [");
        for (j = 0; j < N_LEN; j = j + 1) begin
            if (j < N_LEN-1) $write("%0d, ", B_ntt[j]);
            else              $write("%0d",   B_ntt[j]);
        end
        $display("]");

        check_B_ntt;

        // ═════════════════════════════════════════════════════
        // Pointwise multiply  C[k] = A_ntt[k] * B_ntt[k] mod q
        // ═════════════════════════════════════════════════════
        $display("\n[STEP]  Pointwise multiply  C = NTT(A) * NTT(B) mod %0d", modulus);
        for (j = 0; j < N_LEN; j = j + 1) begin
            pw_prod   = A_ntt[j] * B_ntt[j];
            C_coef[j] = pw_prod % modulus;
        end

        // print C as array
        $write("  C      = [");
        for (j = 0; j < N_LEN; j = j + 1) begin
            if (j < N_LEN-1) $write("%0d, ", C_coef[j]);
            else              $write("%0d",   C_coef[j]);
        end
        $display("]");

        // ═════════════════════════════════════════════════════
        // TEST 3 - INTT of C  (DIT)
        // ═════════════════════════════════════════════════════
        $display("\n[TEST 3]  INTT(C)  -  DIT (CT butterfly) + scale by n_inv");
        do_reset;
        omega = omega_inv_r;   // INTT uses omega_inv
        n_inv = n_inv_r;
        load_C;
        run_transform(1'b1);   // inverse = 1
        capture_result;

        // print INTT(C) as array
        $write("  INTT(C)= [");
        for (j = 0; j < N_LEN; j = j + 1) begin
            if (j < N_LEN-1) $write("%0d, ", result[j]);
            else              $write("%0d",   result[j]);
        end
        $display("]");

        check_result;

        // ═════════════════════════════════════════════════════
        // Summary
        // ═════════════════════════════════════════════════════
        $display("");
        $display("============================================================");
        $display("  FINAL SUMMARY");
        $display("  Checks passed : %0d / %0d", pass_count, pass_count+fail_count);
        if (fail_count == 0)
            $display("  OVERALL       : *** ALL TESTS PASSED ***");
        else
            $display("  OVERALL       : *** %0d FAILURE(S) ***", fail_count);
        $display("============================================================");
        $display("");

        $finish;
    end

    // ----------------------------------------------------------
    // Watchdog
    // ----------------------------------------------------------
    initial begin
        #1_000_000;
        $display("[WATCHDOG] Simulation exceeded time limit.");
        $finish;
    end

endmodule