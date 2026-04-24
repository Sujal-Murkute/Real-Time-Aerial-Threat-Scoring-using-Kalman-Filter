`timescale 1ns/1ps

class TargetPacket;
  string  label;
  int     speed;
  int     distance;
  real    rcs;
  int     approach;
  int     rcs_scaled;
  logic [1:0] appr_enc;
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
endclass

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

module tb_threat_scorer;

  logic        clk, rst_n, valid_in;
  logic [15:0] speed_in, dist_in, rcs_in;
  logic [1:0]  approach_in;
  logic [7:0]  threat_out;
  logic        valid_out;

  threat_scorer dut (
    .clk(clk), .rst_n(rst_n), .valid_in(valid_in),
    .speed_in(speed_in), .dist_in(dist_in), .rcs_in(rcs_in),
    .approach_in(approach_in), .threat_out(threat_out), .valid_out(valid_out)
  );

  initial clk = 0;
  always #5 clk = ~clk;

  TargetPacket pkt_q[$];
  ResultRecord results[$];
  int tgt_idx = 0, out_idx = 0, pass_cnt = 0, fail_cnt = 0;
  integer fin, fout;

  task automatic send_packet(TargetPacket pkt);
    @(posedge clk); #1;
    valid_in    = 1'b1;
    speed_in    = 16'(pkt.speed);
    dist_in     = 16'(pkt.distance);
    rcs_in      = 16'(pkt.rcs_scaled);
    approach_in = pkt.appr_enc;
    tgt_idx++;
    @(posedge clk); #1;
    valid_in = 1'b0;
    @(posedge clk); #1;
    @(posedge clk); #1;
  endtask

  task automatic drain(int n = 14);
    repeat(n) @(posedge clk);
  endtask

  always @(posedge clk) begin
    if (valid_out === 1'b1 && rst_n === 1'b1) begin
      automatic int sc = int'(threat_out);
      out_idx++;
      $fwrite(fout, "%0d\n", sc);

      if (sc > 9)
        $error("[ASSERT FAIL] threat_out=%0d EXCEEDS 9!", sc);
      if ($isunknown(threat_out))
        $error("[ASSERT FAIL] threat_out contains X/Z!");

      if (pkt_q.size() > 0) begin
        automatic TargetPacket p   = pkt_q.pop_front();
        automatic ResultRecord rec = new(p.label, sc, p.exp_lo, p.exp_hi);
        results.push_back(rec);
        if (rec.passed) pass_cnt++;
        else begin
          $error("[FAIL] %-25s score=%0d (exp %0d-%0d)", p.label, sc, p.exp_lo, p.exp_hi);
          fail_cnt++;
        end
      end
    end
  end

  initial begin : stimulus
    automatic TargetPacket pkt;
    automatic int  raw_speed, raw_dist, raw_appr;
    automatic real raw_rcs;
    automatic int  scan_ret;

    rst_n = 0; valid_in = 0;
    speed_in = 0; dist_in = 0; rcs_in = 0; approach_in = 2'b00;

    fout = $fopen("../data/verilog_output.txt", "w");
    if (fout == 0) $fatal(1, "Cannot create verilog_output.txt");

    repeat(3) @(posedge clk); #1;
    rst_n = 1;
    @(posedge clk); #1;

    pkt = new("EC1:MaxThreat",    980,   1, 15.0,  1); pkt.set_expected(9,9);   pkt_q.push_back(pkt); send_packet(pkt);
    pkt = new("EC2:MinThreat",      0, 320,  0.1, -1); pkt.set_expected(0,2);   pkt_q.push_back(pkt); send_packet(pkt);
    pkt = new("EC3:MissileClose", 900,  30, 12.0,  1); pkt.set_expected(8,9);   pkt_q.push_back(pkt); send_packet(pkt);
    pkt = new("EC4:DroneFar",     120, 300,  0.8, -1); pkt.set_expected(0,2);   pkt_q.push_back(pkt); send_packet(pkt);
    pkt = new("EC5:AircraftMid",  500, 160,  6.0,  1); pkt.set_expected(4,8);   pkt_q.push_back(pkt); send_packet(pkt);
    pkt = new("EC6:DistMax",      700, 320,  8.0,  1); pkt.set_expected(2,6);   pkt_q.push_back(pkt); send_packet(pkt);
    pkt = new("EC7:RCSMax",       600,  80, 15.0,  1); pkt.set_expected(6,9);   pkt_q.push_back(pkt); send_packet(pkt);
    pkt = new("EC8:FormSalvoA",   850,  75, 12.0,  1); pkt.set_expected(7,9);   pkt_q.push_back(pkt); send_packet(pkt);
    pkt = new("EC9:FormSalvoB",   870,  80, 11.5,  1); pkt.set_expected(7,9);   pkt_q.push_back(pkt); send_packet(pkt);
    drain(14);

    fin = $fopen("../data/input_data.txt", "r");
    if (fin == 0) $fatal(1, "Cannot open input_data.txt");

    scan_ret = 4;
    while (scan_ret == 4) begin
      scan_ret = $fscanf(fin, "%d %d %f %d\n", raw_speed, raw_dist, raw_rcs, raw_appr);
      if (scan_ret == 4) begin
        pkt = new($sformatf("DATA:TGT-%0d", tgt_idx+1), raw_speed, raw_dist, raw_rcs, raw_appr);
        pkt.set_expected(0, 9);
        pkt_q.push_back(pkt);
        send_packet(pkt);
      end
    end
    $fclose(fin);
    drain(16);
    $fclose(fout);

    $display("PASS=%0d  FAIL=%0d", pass_cnt, fail_cnt);
    $finish;
  end

  initial begin
    #3_000_000;
    $fatal(1, "TIMEOUT");
  end

endmodule
