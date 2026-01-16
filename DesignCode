`timescale 1ns/1ps

// 8-bit LFSR Module
module LFSR_8bit(
    input clk,
    input reset,
    input enable,
    output reg [7:0] out
);
    wire feedback;
    assign feedback = out[7] ^ out[5] ^ out[4] ^ out[3];
    
    always @(posedge clk or posedge reset) begin
        if (reset)
            out <= 8'b11111111;
        else if (enable)
            out <= {feedback, out[7:1]};
    end
endmodule

// Main RBLWE Accelerator - POLYMUL + POLYADD + BINADD + ADDE + SAMPLE
module rblwe_accelerator_top(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [4:0]  opcode,

    input wire [31:0] d_poly,
    input wire [31:0] b_poly,
    input wire [31:0] g_poly,
    input wire [35:0] h_poly,  // can carry 36-bit H now

    output reg [35:0] w_poly,  // widened to 36 bits
    output reg        valid,
    output reg        done
);

// State machine
reg [2:0] state;
localparam IDLE   = 3'd0;
localparam EXEC   = 3'd1;
localparam SAMPLE = 3'd2;
localparam FINISH = 3'd3;

// Latched inputs
reg [31:0] D, G, B;
reg [35:0] H;           // H is 36-bit internally
reg [4:0]  op;

// LFSR signals
wire [7:0] lfsr_out;
reg        lfsr_reset, lfsr_enable;
reg [3:0]  sample_count;

// Instantiate LFSR
LFSR_8bit lfsr_inst(
    .clk   (clk),
    .reset (lfsr_reset),
    .enable(lfsr_enable),
    .out   (lfsr_out)
);

// Simple mod 7 function (for integer-domain ops: POLYADD)
function [2:0] mod7;
    input [31:0] value;
    begin
        mod7 = value % 7;
    end
endfunction

integer ii, jj;
reg  signed [7:0] temp_signed;   
reg  [35:0]       result_bits;   // widened to 36 bits

// file handle for saving H
integer h_file;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        state        <= IDLE;
        w_poly       <= 36'd0;
        valid        <= 1'b0;
        done         <= 1'b0;
        D            <= 32'd0;
        G            <= 32'd0;
        B            <= 32'd0;
        H            <= 36'd0;   // default H = 0
        op           <= 5'd0;
        lfsr_reset   <= 1'b1;
        lfsr_enable  <= 1'b0;
        sample_count <= 4'd0;
        result_bits  <= 36'd0;
    end
    else begin
        case(state)
            IDLE: begin
                valid       <= 1'b0;
                done        <= 1'b0;
                lfsr_enable <= 1'b0;
                lfsr_reset  <= 1'b0;
                
                if(start) begin
                    // Latch inputs
                    D <= d_poly;
                    G <= g_poly;
                    B <= b_poly;

                    // For non-SAMPLE operations, load H from external h_poly.
                    if (opcode != 5'b00100)
                        H <= h_poly;

                    op <= opcode;
                    
                    if(opcode == 5'b00100) begin  // SAMPLE
                        state        <= SAMPLE;
                        sample_count <= 4'd0;
                        w_poly       <= 36'd0;
                        lfsr_reset   <= 1'b1;
                        $display("[%0t] DESIGN: Starting SAMPLE operation", $time);
                    end
                    else begin
                        state <= EXEC;
                        $display("[%0t] DESIGN: Latched opcode=%b, D=%0d, G=%0d, H_LSB32=%0d", 
                                 $time, opcode, d_poly, g_poly, H[31:0]);
                    end
                end
            end
            
            SAMPLE: begin
                lfsr_reset  <= 1'b0;
                lfsr_enable <= 1'b1;
                
                // Collect 4 bytes (32 bits) from LFSR → this becomes lower 32 bits of H
                if(sample_count < 4) begin
                    // shift 8 bits into the lower 32 bits, keep top 4 bits zero
                    w_poly       <= {w_poly[27:0], lfsr_out}; // 28+8 = 36
                    sample_count <= sample_count + 1;
                    $display("[%0t] DESIGN: SAMPLE byte %0d = 0x%h", 
                             $time, sample_count, lfsr_out);
                end
                else begin
                    lfsr_enable <= 1'b0;
                    valid       <= 1'b1;
                    state       <= FINISH;

                    // Store sampled value as internal H polynomial
                    H <= w_poly;

                    // Write H to a file so it can be used in the next sim run
                    h_file = $fopen("H_value.mem", "w");
                    if (h_file == 0) begin
                        $display("[%0t] ERROR: Could not open H_value.mem for writing", $time);
                    end
                    else begin
                        $fdisplay(h_file, "%h", w_poly); // 36-bit hex
                        $fclose(h_file);
                        $display("[%0t] DESIGN: SAMPLE complete - H (and W) = 0x%09h, saved to H_value.mem", 
                                 $time, w_poly);
                    end
                end
            end
            
            EXEC: begin
                // Perform operation based on opcode
                case(op)
                    // POLYMUL: negacyclic convolution mod (x^32 + 1), 1-bit coefficients
                    5'b00001: begin
                        result_bits = 36'd0;
                        for (ii = 0; ii < 32; ii = ii + 1) begin
                            temp_signed = 0;
                            for (jj = 0; jj < 32; jj = jj + 1) begin
                                if (ii >= jj) begin
                                    if (D[jj] & B[ii-jj])
                                        temp_signed = temp_signed + 1;
                                end
                                else begin
                                    if (D[jj] & B[32 + ii - jj])
                                        temp_signed = temp_signed - 1;
                                end
                            end
                            // pack LSB into result_bits; upper 4 bits stay 0
                            result_bits[ii] = temp_signed[0];
                        end
                        w_poly <= result_bits;
                        $display("[%0t] DESIGN: POLYMUL (negacyclic) - D=0x%08h B=0x%08h => W_LSB32=0x%08h", 
                                 $time, D, B, result_bits[31:0]);
                    end
                    
                    // POLYADD: integer-domain addition, mod 7 (no error)
                    5'b00010: begin
                        w_poly <= {33'd0, mod7(D + G)};  // 3 LSBs carry mod7 result
                        $display("[%0t] DESIGN: POLYADD - (%0d + %0d) mod 7 = %0d", 
                                 $time, D, G, mod7(D + G));
                    end
                    
                    // BINADD: binary polynomial addition, GF(2) XOR (on 32 bits)
                    5'b00011: begin
                        w_poly <= {4'd0, (D ^ H[31:0])};
                        $display("[%0t] DESIGN: BINADD (binary poly add) - D ^ H_LSB32 = 0x%08h", 
                                 $time, (D ^ H[31:0]));
                    end
                    
                    // ADDE: binary polynomial add with error, D ^ G ^ H (on 32 bits)
                    5'b00110: begin
                        w_poly <= {4'd0, (D ^ G ^ H[31:0])};
                        $display("[%0t] DESIGN: ADDE (binary poly add with error) - D ^ G ^ H_LSB32 = 0x%08h",
                                 $time, (D ^ G ^ H[31:0]));
                    end
                    
                    default: begin
                        w_poly <= 36'd0;
                        $display("[%0t] DESIGN: UNKNOWN opcode=%b", $time, op);
                    end
                endcase
                
                valid <= 1'b1;
                state <= FINISH;
            end
            
            FINISH: begin
                done  <= 1'b1;
                state <= IDLE;
            end
            
            default: state <= IDLE;
        endcase
    end
end

endmodule
