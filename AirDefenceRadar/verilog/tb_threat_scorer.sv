`timescale 1ns/1ps

module threat_scorer (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_in,
    input  logic [15:0] speed_in,
    input  logic [15:0] dist_in,
    input  logic [15:0] rcs_in,
    input  logic [1:0]  approach_in,
    output logic [7:0]  threat_out,
    output logic        valid_out
);

    // ── PARAMETERS ──────────────────────
    localparam logic [15:0] W_DIST  = 16'd640;
    localparam logic [15:0] W_SPEED = 16'd384;
    localparam logic [15:0] W_RCS   = 16'd256;
    localparam logic [15:0] W_DIR   = 16'd512;

    localparam logic [15:0] DIST_MAX  = 16'd320;
    localparam logic [15:0] SPEED_MAX = 16'd980;
    localparam logic [15:0] RCS_MAX   = 16'd150;

    localparam logic [31:0] SCORE_DIV = 32'd199;

    // ── STAGE 1 REGS ───────────────────
    logic [31:0] s1_dist_norm;
    logic [31:0] s1_speed_norm;
    logic [31:0] s1_rcs_norm;
    logic [15:0] s1_dir_val;
    logic        s1_valid;

    // ── STAGE 2 REGS ───────────────────
    logic [31:0] s2_wsum;
    logic        s2_valid;

    // ── STAGE 1 ─────────────────────────
    always_ff @(posedge clk) begin
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
    always_ff @(posedge clk) begin
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
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            threat_out <= 8'd0;
            valid_out  <= 1'b0;
        end else begin
            valid_out <= s2_valid;

            if (s2_valid) begin
                if ((s2_wsum / SCORE_DIV) > 9)
                    threat_out <= 8'd9;
                else
                    threat_out <= (s2_wsum / SCORE_DIV);
            end
        end
    end

endmodule