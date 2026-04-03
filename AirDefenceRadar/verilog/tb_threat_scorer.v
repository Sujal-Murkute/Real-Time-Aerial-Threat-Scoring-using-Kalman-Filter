// =========================================================
// PROJECT : Dynamic Weight-Adaptive Threat Scoring Framework
// FILE    : threat_scorer.v
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

    localparam [15:0] W_DIST  = 16'd640;
    localparam [15:0] W_SPEED = 16'd384;
    localparam [15:0] W_RCS   = 16'd256;
    localparam [15:0] W_DIR   = 16'd512;

    localparam [15:0] DIST_MAX  = 16'd320;
    localparam [15:0] SPEED_MAX = 16'd980;
    localparam [15:0] RCS_MAX   = 16'd150;

    localparam [31:0] SCORE_DIV = 32'd199;

    reg [31:0] s1_dist_norm;
    reg [31:0] s1_speed_norm;
    reg [31:0] s1_rcs_norm;
    reg [15:0] s1_dir_val;
    reg        s1_valid;

    reg [31:0] s2_wsum;
    reg        s2_valid;

    // ── STAGE 1 ─────────────────────────
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
                if (dist_in >= DIST_MAX)
                    s1_dist_norm <= 32'd0;
                else
                    s1_dist_norm <= ((DIST_MAX - dist_in) << 8) / DIST_MAX;

                if (speed_in >= SPEED_MAX)
                    s1_speed_norm <= 32'd256;
                else
                    s1_speed_norm <= (speed_in << 8) / SPEED_MAX;

                if (rcs_in >= RCS_MAX)
                    s1_rcs_norm <= 32'd256;
                else
                    s1_rcs_norm <= (rcs_in << 8) / RCS_MAX;

                s1_dir_val <= (approach_in == 2'b01) ? 16'd256 : 16'd0;
            end
        end
    end

    // ── STAGE 2 ─────────────────────────
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

    // ── STAGE 3 ─────────────────────────
    always @(posedge clk) begin
        if (!rst_n) begin
            threat_out <= 8'd0;
            valid_out  <= 1'b0;
        end else begin
            valid_out <= s2_valid;

            if (s2_valid) begin
                if ((s2_wsum / SCORE_DIV) > 9)
                    threat_out <= 8'd9;
                else
                    threat_out <= (s2_wsum / SCORE_DIV); // ✅ FIXED LINE
            end
        end
    end

endmodule