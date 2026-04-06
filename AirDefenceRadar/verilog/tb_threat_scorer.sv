// =========================================================
// FILE    : tb_threat_scorer.sv
// TOOL    : ModelSim Intel FPGA Edition (no QuestaSim needed)
// READS   : ../data/input_data.txt
// WRITES  : ../data/verilog_output.txt
// =========================================================
// HOW TO RUN:
//   1. vlog threat_scorer.v
//   2. vlog -sv tb_threat_scorer.sv
//   3. vsim -novopt tb_threat_scorer
//   4. run -all
// =========================================================
// NOTE: Covergroup and SVA assertions removed — those require
//       QuestaSim. Equivalent checks done in plain SV instead.
// =========================================================

`timescale 1ns/1ps

// ─────────────────────────────────────────────────────────
//  CLASS: TargetPacket
// ─────────────────────────────────────────────────────────
class TargetPacket;

  string  label;
  int     speed;        // km/h
  int     distance;     // km
  real    rcs;          // m²
  int     approach;     // +1 or -1
  int     rcs_scaled;   // rcs x 10, clamped 1-150
  logic [1:0] appr_enc; // 2'b01=approaching, 2'b00=receding

  int exp_lo = 0;
  int exp_hi = 9;

  function new(string lbl, int spd, int dst, real r, int appr);
    label    = lbl;
    speed    = (spd > 980) ? 980 : (spd < 0) ? 0 : spd;
    distance = (dst > 320) ? 320 : (dst < 0) ? 0 : dst;
    rcs      = r;
    approach = appr;

    rcs_scaled = int'(r * 10.0 + 0.5);
    if (rcs_scaled < 1)   rcs_scaled = 1;
    if (rcs_scaled > 150) rcs_scaled = 150;

    appr_enc = (appr == 1) ? 2'b01 : 2'b00;
  endfunction

  function void set_expected(int lo, int hi);
    exp_lo = lo;
    exp_hi = hi;
  endfunction

  function void print();
    $display("  [PKT] %-25s spd=%-4d dst=%-4d rcs=%4.1f appr=%s",
             label, speed, distance, rcs,
             (approach == 1) ? "APPROACH" : "RECEDE");
  endfunction

endclass

// ─────────────────────────────────────────────────────────
//  CLASS: ResultRecord
// ─────────────────────────────────────────────────────────
class ResultRecord;
  string label;
  int    score;
  int    exp_lo, exp_hi;
  bit    passed;

  function new(string lbl, int sc, int lo, int hi);
    label  = lbl;
    score  = sc;
    exp_lo = lo;
    exp_hi = hi;
    passed = (sc >= lo && sc <= hi);
  endfunction

endclass

// ─────────────────────────────────────────────────────────
//  TESTBENCH MODULE
// ─────────────────────────────────────────────────────────
module tb_threat_scorer;

  // ── DUT SIGNALS ────────────────────────────────────────
  logic        clk;
  logic        rst_n;
  logic        valid_in;
  logic [15:0] speed_in;
  logic [15:0] dist_in;
  logic [15:0] rcs_in;
  logic [1:0]  approach_in;
  logic [7:0]  threat_out;
  logic        valid_out;

  // ── DUT INSTANCE ───────────────────────────────────────
  threat_scorer dut (
    .clk         (clk),
    .rst_n       (rst_n),
    .valid_in    (valid_in),
    .speed_in    (speed_in),
    .dist_in     (dist_in),
    .rcs_in      (rcs_in),
    .approach_in (approach_in),
    .threat_out  (threat_out),
    .valid_out   (valid_out)
  );

  // ── CLOCK: 100 MHz ─────────────────────────────────────
  initial clk = 1'b0;
  always  #5  clk = ~clk;

  // ── QUEUES & COUNTERS ──────────────────────────────────
  TargetPacket  pkt_q[$];
  ResultRecord  results[$];

  int  tgt_idx  = 0;
  int  out_idx  = 0;
  int  pass_cnt = 0;
  int  fail_cnt = 0;

  // ── MANUAL COVERAGE COUNTERS (replaces covergroup) ─────
  int cov_score_low  = 0;   // score 0-2
  int cov_score_mid  = 0;   // score 3-6
  int cov_score_high = 0;   // score 7-9
  int cov_dist_near  = 0;   // dist  0-79
  int cov_dist_mid   = 0;   // dist  80-200
  int cov_dist_far   = 0;   // dist  201-320
  int cov_spd_drone  = 0;   // speed 0-299
  int cov_spd_acft   = 0;   // speed 300-699
  int cov_spd_msle   = 0;   // speed 700-980
  int cov_dir_appr   = 0;
  int cov_dir_reced  = 0;

  integer fin, fout;

  // ── MANUAL ASSERTION TASK (replaces SVA) ───────────────
  task automatic check_output(int sc);
    // Assertion 1: score must be 0-9
    if (sc > 9)
      $error("[ASSERT FAIL] @%0d ns  threat_out=%0d  EXCEEDS 9!", longint'($time), sc);
    // Assertion 2: no X/Z on threat_out
    if ($isunknown(threat_out))
      $error("[ASSERT FAIL] @%0d ns  threat_out contains X/Z!", longint'($time));
  endtask

  // ── TASK: SEND ONE PACKET ──────────────────────────────
  task automatic send_packet(TargetPacket pkt);
    pkt.print();
    @(posedge clk); #1;
    valid_in    = 1'b1;
    speed_in    = 16'(pkt.speed);
    dist_in     = 16'(pkt.distance);
    rcs_in      = 16'(pkt.rcs_scaled);
    approach_in = pkt.appr_enc;
    tgt_idx++;

    @(posedge clk); #1;
    valid_in = 1'b0;

    // 2-cycle idle gap between targets
    @(posedge clk); #1;
    @(posedge clk); #1;
  endtask

  // ── TASK: DRAIN PIPELINE ───────────────────────────────
  task automatic drain(int n = 14);
    repeat(n) @(posedge clk);
  endtask

  // ── OUTPUT CAPTURE ─────────────────────────────────────
  always @(posedge clk) begin
    if (valid_out === 1'b1 && rst_n === 1'b1) begin
      automatic int sc = int'(threat_out);
      out_idx++;

      $fwrite(fout, "%0d\n", sc);

      // Manual assertions
      check_output(sc);

      // Manual coverage sampling
      if      (sc <= 2) cov_score_low++;
      else if (sc <= 6) cov_score_mid++;
      else              cov_score_high++;

      if      (dist_in <= 79)  cov_dist_near++;
      else if (dist_in <= 200) cov_dist_mid++;
      else                     cov_dist_far++;

      if      (speed_in <= 299) cov_spd_drone++;
      else if (speed_in <= 699) cov_spd_acft++;
      else                      cov_spd_msle++;

      if (approach_in == 2'b01) cov_dir_appr++;
      else                      cov_dir_reced++;

      // Result checking
      if (pkt_q.size() > 0) begin
        automatic TargetPacket p   = pkt_q.pop_front();
        automatic ResultRecord rec = new(p.label, sc, p.exp_lo, p.exp_hi);
        results.push_back(rec);

        if (rec.passed) begin
          $display("[PASS] Out-%0d | %-25s score=%0d (exp %0d-%0d)",
                   out_idx, p.label, sc, p.exp_lo, p.exp_hi);
          pass_cnt++;
        end else begin
          $error("[FAIL] Out-%0d | %-25s score=%0d (exp %0d-%0d)",
                 out_idx, p.label, sc, p.exp_lo, p.exp_hi);
          fail_cnt++;
        end

      end else begin
        if (sc <= 9) begin
          $display("[PASS] Out-%0d | DATA score=%0d", out_idx, sc);
          pass_cnt++;
        end else begin
          $error("[FAIL] Out-%0d | DATA score=%0d OUT OF RANGE", out_idx, sc);
          fail_cnt++;
        end
      end
    end
  end

  // ── MAIN STIMULUS ──────────────────────────────────────
  initial begin : stimulus

    automatic TargetPacket pkt;
    automatic int    raw_speed, raw_dist, raw_appr;
    automatic real   raw_rcs;
    automatic int    scan_ret;

    rst_n       = 1'b0;
    valid_in    = 1'b0;
    speed_in    = 16'd0;
    dist_in     = 16'd0;
    rcs_in      = 16'd0;
    approach_in = 2'b00;

    fout = $fopen("../data/verilog_output.txt", "w");
    if (fout == 0)
      $fatal(1, "[ERROR] Cannot create ../data/verilog_output.txt");

    $display("[TB] Output file opened: ../data/verilog_output.txt");

    repeat(3) @(posedge clk); #1;
    rst_n = 1'b1;
    @(posedge clk); #1;

    // ════════════════════════════════════════════════════
    //  PHASE 1 - EDGE CASE TESTS
    // ════════════════════════════════════════════════════
    $display("");
    $display("╔══════════════════════════════════════════════════╗");
    $display("║         PHASE 1 - EDGE CASE TESTS               ║");
    $display("╚══════════════════════════════════════════════════╝");

    pkt = new("EC1:MaxThreat",    980,   1, 15.0,  1); pkt.set_expected(9,9);
    pkt_q.push_back(pkt); send_packet(pkt);

    pkt = new("EC2:MinThreat",      0, 320,  0.1, -1); pkt.set_expected(0,2);
    pkt_q.push_back(pkt); send_packet(pkt);

    pkt = new("EC3:MissileClose", 900,  30, 12.0,  1); pkt.set_expected(8,9);
    pkt_q.push_back(pkt); send_packet(pkt);

    pkt = new("EC4:DroneFar",     120, 300,  0.8, -1); pkt.set_expected(0,2);
    pkt_q.push_back(pkt); send_packet(pkt);

    pkt = new("EC5:AircraftMid",  500, 160,  6.0,  1); pkt.set_expected(4,8);
    pkt_q.push_back(pkt); send_packet(pkt);

    pkt = new("EC6:DistMax",      700, 320,  8.0,  1); pkt.set_expected(2,6);
    pkt_q.push_back(pkt); send_packet(pkt);

    pkt = new("EC7:RCSMax",       600,  80, 15.0,  1); pkt.set_expected(6,9);
    pkt_q.push_back(pkt); send_packet(pkt);

    pkt = new("EC8:FormSalvoA",   850,  75, 12.0,  1); pkt.set_expected(7,9);
    pkt_q.push_back(pkt); send_packet(pkt);

    pkt = new("EC9:FormSalvoB",   870,  80, 11.5,  1); pkt.set_expected(7,9);
    pkt_q.push_back(pkt); send_packet(pkt);

    drain(14);

    $display("");
    $display("╔══════════════════════════════════════════════════╗");
    $display("║  PHASE 1 DONE - %0d edge cases sent              ║", tgt_idx);
    $display("╚══════════════════════════════════════════════════╝");

    // ════════════════════════════════════════════════════
    //  PHASE 2 - REAL MATLAB DATA
    // ════════════════════════════════════════════════════
    $display("");
    $display("╔══════════════════════════════════════════════════╗");
    $display("║         PHASE 2 - MATLAB REAL DATA              ║");
    $display("╚══════════════════════════════════════════════════╝");

    fin = $fopen("../data/input_data.txt", "r");
    if (fin == 0)
      $fatal(1, "[ERROR] Cannot open ../data/input_data.txt - run MATLAB first");

    scan_ret = 4;
    while (scan_ret == 4) begin
      scan_ret = $fscanf(fin, "%d %d %f %d\n",
                         raw_speed, raw_dist, raw_rcs, raw_appr);
      if (scan_ret < 4) begin
        scan_ret = 0;
      end else begin
        pkt = new(
          $sformatf("DATA:TGT-%0d", tgt_idx+1),
          raw_speed, raw_dist, raw_rcs, raw_appr
        );
        pkt.set_expected(0, 9);
        pkt_q.push_back(pkt);
        send_packet(pkt);
      end
    end
    $fclose(fin);

    drain(16);
    $fclose(fout);

    // ════════════════════════════════════════════════════
    //  FINAL REPORT
    // ════════════════════════════════════════════════════
    $display("");
    $display("╔══════════════════════════════════════════════════╗");
    $display("║         ADIR-7  VERIFICATION REPORT             ║");
    $display("╠══════════════════════════════════════════════════╣");
    $display("║  Targets sent     : %-4d                         ║", tgt_idx);
    $display("║  Outputs received : %-4d                         ║", out_idx);
    $display("║  PASS             : %-4d                         ║", pass_cnt);
    $display("║  FAIL             : %-4d                         ║", fail_cnt);
    $display("╠══════════════════════════════════════════════════╣");
    if (fail_cnt == 0)
      $display("║  RESULT : *** ALL TESTS PASSED ***              ║");
    else
      $display("║  RESULT : !!! %0d FAILURE(S) DETECTED !!!       ║", fail_cnt);
    $display("╠══════════════════════════════════════════════════╣");
    $display("║  Output: ../data/verilog_output.txt             ║");
    $display("╚══════════════════════════════════════════════════╝");

    // ── COVERAGE REPORT ────────────────────────────────
    $display("");
    $display("╔══════════════════════════════════════════════════╗");
    $display("║         FUNCTIONAL COVERAGE SUMMARY             ║");
    $display("╠══════════════════════════════════════════════════╣");
    $display("║  Score : low(0-2)=%-3d  mid(3-6)=%-3d  high(7-9)=%-3d ║",
             cov_score_low, cov_score_mid, cov_score_high);
    $display("║  Dist  : near=%-3d      mid=%-3d       far=%-3d       ║",
             cov_dist_near, cov_dist_mid, cov_dist_far);
    $display("║  Speed : drone=%-3d     acft=%-3d      msle=%-3d      ║",
             cov_spd_drone, cov_spd_acft, cov_spd_msle);
    $display("║  Dir   : approach=%-3d  recede=%-3d                ║",
             cov_dir_appr, cov_dir_reced);
    $display("╚══════════════════════════════════════════════════╝");

    // ── EDGE CASE DETAIL ───────────────────────────────
    $display("");
    $display("EDGE CASE DETAIL:");
    $display("  %-25s  %-6s  %s", "Label", "Score", "Result");
    $display("  -------------------------------------------------");
    foreach(results[i]) begin
      $display("  %-25s  %-6d  %s",
               results[i].label,
               results[i].score,
               results[i].passed ? "PASS" : "FAIL <- check waveform");
    end

    $finish;
  end

  // ── TIMEOUT ────────────────────────────────────────────
  initial begin
    #3_000_000;
    $display("[TIMEOUT] Simulation exceeded 3 ms limit.");
    $fclose(fout);
    $finish;
  end

endmodule
