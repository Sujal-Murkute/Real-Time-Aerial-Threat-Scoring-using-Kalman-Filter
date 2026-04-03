// =========================================================
// FIXED: threat_scorer.v
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

    // Weights (Q8.8)
    localparam [15:0] W_DIST  = 16'd640;
    localparam [15:0] W_SPEED = 16'd384;
    localparam [15:0] W_RCS   = 16'd256;
    localparam [15:0] W_DIR   = 16'd512;

    // Max values
    localparam [15:0] DIST_MAX  = 16'd320;
    localparam [15:0] SPEED_MAX = 16'd980;
    localparam [15:0] RCS_MAX   = 16'd150;

    localparam [31:0] SCORE_DIV = 32'd199;

    // Stage 1
    reg [31:0] s1_dist_norm, s1_speed_norm, s1_rcs_norm;
    reg [15:0] s1_dir_val;
    reg        s1_valid;

    // Stage 2
    reg [31:0] s2_wsum;
    reg        s2_valid;

    // Stage 3 temp
    reg [7:0] scaled_score;

    // ================= STAGE 1 =================
    always @(posedge clk) begin
        if (!rst_n) begin
            s1_valid <= 0;
        end else begin
            s1_valid <= valid_in;

            if (valid_in) begin
                // Distance
                if (dist_in >= DIST_MAX)
                    s1_dist_norm <= 0;
                else
                    s1_dist_norm <= ((DIST_MAX - dist_in) << 8) / DIST_MAX;

                // Speed
                if (speed_in >= SPEED_MAX)
                    s1_speed_norm <= 256;
                else
                    s1_speed_norm <= (speed_in << 8) / SPEED_MAX;

                // RCS
                if (rcs_in >= RCS_MAX)
                    s1_rcs_norm <= 256;
                else
                    s1_rcs_norm <= (rcs_in << 8) / RCS_MAX;

                // Direction
                s1_dir_val <= (approach_in == 2'b01) ? 16'd256 : 16'd0;
            end
        end
    end

    // ================= STAGE 2 =================
    always @(posedge clk) begin
        if (!rst_n) begin
            s2_valid <= 0;
            s2_wsum  <= 0;
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

    // ================= STAGE 3 =================
    always @(posedge clk) begin
        if (!rst_n) begin
            threat_out <= 0;
            valid_out  <= 0;
        end else begin
            valid_out <= s2_valid;

            if (s2_valid) begin
                scaled_score = s2_wsum / SCORE_DIV;

                if (scaled_score > 9)
                    threat_out <= 9;
                else
                    threat_out <= scaled_score;
            end
        end
    end

endmodule