// =========================================================
// PROJECT : Dynamic Weight-Adaptive Threat Scoring Framework
// FILE    : threat_scorer.v
// PURPOSE : Compute fixed-point threat score 0–9 per target
// FORMAT  : Q8.8 fixed-point (16-bit: 8 integer + 8 fractional)
// PIPELINE: 3 stages (latency = 3 clock cycles)
// =========================================================
// PORT MAP:
//   clk         — clock (rising edge)
//   rst_n       — active-low synchronous reset
//   valid_in    — pulse high 1 cycle when input data is ready
//   speed_in    — target speed in km/h (integer, 16-bit)
//   dist_in     — target distance in km (integer, 16-bit)
//   rcs_in      — RCS × 10 (e.g. 32 = 3.2 m²), integer, 16-bit
//   approach_in — 2'b01=approaching  2'b11=receding (2's complement -1)
//   threat_out  — threat score 0–9 (8-bit)
//   valid_out   — high for 1 cycle when threat_out is valid
// =========================================================

module threat_scorer (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid_in,
    input  wire [15:0] speed_in,
    input  wire [15:0] dist_in,
    input  wire [15:0] rcs_in,
    input  wire [1:0]  approach_in,
    output reg  [7:0]  threat_out,
    output reg         valid_out
);

    // ── WEIGHT CONSTANTS in Q8.8 (value × 256) ───────────
    // Threat formula:
    //   score = W_DIST  × (1 – dist/320)    ← closer = more dangerous
    //         + W_SPEED × (speed/980)
    //         + W_RCS   × (rcs×10/150)
    //         + W_DIR   × (1 if approaching, 0 if receding)
    //
    localparam [15:0] W_DIST  = 16'd640;  // 2.50 in Q8.8
    localparam [15:0] W_SPEED = 16'd384;  // 1.50 in Q8.8
    localparam [15:0] W_RCS   = 16'd256;  // 1.00 in Q8.8
    localparam [15:0] W_DIR   = 16'd512;  // 2.00 in Q8.8

    // ── NORMALISATION DENOMINATORS ────────────────────────
    localparam [15:0] DIST_MAX  = 16'd320;  // max radar range  (km)
    localparam [15:0] SPEED_MAX = 16'd980;  // max tracked speed (km/h)
    localparam [15:0] RCS_MAX   = 16'd150;  // max RCS×10 value

    // Max possible weighted_sum (all norms = 1.0 = 256 in Q8.8):
    //   = (640+384+256+512) = 1792
    // Scale to 0–9: divide by floor(1792/9) = 199
    localparam [31:0] SCORE_DIV = 32'd199;

    // ── STAGE 1 REGISTERS: Normalised values ─────────────
    reg [31:0] s1_dist_norm;   // (DIST_MAX - dist) / DIST_MAX  in Q8.8
    reg [31:0] s1_speed_norm;  // speed / SPEED_MAX              in Q8.8
    reg [31:0] s1_rcs_norm;    // rcs   / RCS_MAX                in Q8.8
    reg [15:0] s1_dir_val;     // 256 if approaching, 0 if not
    reg        s1_valid;

    // ── STAGE 2 REGISTERS: Weighted sum ──────────────────
    reg [31:0] s2_wsum;
    reg        s2_valid;

    // ── STAGE 1: Normalise inputs ─────────────────────────
    always @(posedge clk) begin
        if (!rst_n) begin
            s1_dist_norm  <= 0;
            s1_speed_norm <= 0;
            s1_rcs_norm   <= 0;
            s1_dir_val    <= 0;
            s1_valid      <= 0;
        end else begin
            s1_valid <= valid_in;

            if (valid_in) begin
                // Distance: invert so closer → higher score
                // norm = (DIST_MAX - clamp(dist)) × 256 / DIST_MAX
                if (dist_in >= DIST_MAX)
                    s1_dist_norm <= 32'd0;
                else
                    s1_dist_norm <= ((DIST_MAX - dist_in) << 8) / DIST_MAX;

                // Speed: higher → higher score
                if (speed_in >= SPEED_MAX)
                    s1_speed_norm <= 32'd256;        // cap at 1.0
                else
                    s1_speed_norm <= (speed_in << 8) / SPEED_MAX;

                // RCS: larger object → higher score (rcs_in is ×10 scaled)
                if (rcs_in >= RCS_MAX)
                    s1_rcs_norm <= 32'd256;
                else
                    s1_rcs_norm <= (rcs_in << 8) / RCS_MAX;

                // Direction: approaching = 1.0 (256 in Q8.8), else 0
                s1_dir_val <= (approach_in == 2'b01) ? 16'd256 : 16'd0;
            end
        end
    end

    // ── STAGE 2: Apply weights ────────────────────────────
    // Each term: norm(Q8.8) × weight(Q8.8) → Q16.16, shift back to Q8.8
    always @(posedge clk) begin
        if (!rst_n) begin
            s2_wsum  <= 0;
            s2_valid <= 0;
        end else begin
            s2_valid <= s1_valid;

            if (s1_valid) begin
                s2_wsum <=
                    ((s1_dist_norm  * W_DIST)  >> 8) +
                    ((s1_speed_norm * W_SPEED) >> 8) +
                    ((s1_rcs_norm   * W_RCS)   >> 8) +
                    ((s1_dir_val    * W_DIR)   >> 8);
            end
        end
    end

    // ── STAGE 3: Scale to 0–9 and output ─────────────────
    always @(posedge clk) begin
        if (!rst_n) begin
            threat_out <= 8'd0;
            valid_out  <= 1'b0;
        end else begin
            valid_out <= s2_valid;

            if (s2_valid) begin
                // Divide by SCORE_DIV to map Q8.8 sum → 0–9
                // Use intermediate reg to avoid part-select on expression
                begin : scale_block
                    reg [31:0] scaled;
                    scaled = s2_wsum / SCORE_DIV;
                    if (scaled > 32'd9)
                        threat_out <= 8'd9;
                    else
                        threat_out <= scaled[7:0];
                end
            end
        end
    end

endmodule
