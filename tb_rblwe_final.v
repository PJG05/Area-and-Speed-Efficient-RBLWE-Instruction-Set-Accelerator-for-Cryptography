`timescale 1ns/1ps

module tb_rblwe_final;
    reg         clk;
    reg         rst_n;
    reg         start;
    reg  [4:0]  opcode;
    reg  [31:0] d_poly, g_poly, b_poly;
    reg  [35:0] h_poly;          // 36-bit H
    wire [35:0] w_poly;          // 36-bit W
    wire        valid, done;

    integer     h_file;
    reg [35:0]  H_from_file;
    
    // Instantiate DUT
    rblwe_accelerator_top dut(
        .clk   (clk),
        .rst_n (rst_n),
        .start (start),
        .opcode(opcode),
        .d_poly(d_poly),
        .b_poly(b_poly),
        .g_poly(g_poly),
        .h_poly(h_poly),
        .w_poly(w_poly),
        .valid(valid),
        .done (done)
    );
    
    // Clock generation (10 ns period)
    initial clk = 1'b0;
    always #5 clk = ~clk;
    
    initial begin
        $display("\n========================================");
        $display("   RBLWE ACCELERATOR WITH ADDE (36-bit W)");
        $display("========================================");
        $display("Opcode Guide (5-bit):");
        $display("  5'b00001 = POLYMUL (negacyclic)");
        $display("  5'b00010 = POLYADD (integer) : (D+G) mod 7 -> LSBs");
        $display("  5'b00011 = BINADD  (binary poly): D ^ H_LSB32");
        $display("  5'b00100 = SAMPLE  (Generate 32-bit H in LSBs, save)");
        $display("  5'b00110 = ADDE    (binary poly with error): D ^ G ^ H_LSB32");
        $display("========================================\n");
        
        // Initialize
        rst_n        = 1'b0;
        start        = 1'b0;
        d_poly       = 32'd0;
        g_poly       = 32'd0;
        b_poly       = 32'd0;
        h_poly       = 36'd0;
        H_from_file  = 36'd0;
        
        // Try to load H from file (if exists)
        h_file = $fopen("H_value.mem", "r");
        if (h_file == 0) begin
            $display("TB: H_value.mem not found, using default H = 0");
        end
        else begin
            if ($fscanf(h_file, "%h", H_from_file) == 1) begin
                h_poly = H_from_file;
                $display("TB: Loaded H from H_value.mem: 0x%09h", h_poly);
            end
            else begin
                $display("TB: H_value.mem is empty or invalid, using H = 0");
            end
            $fclose(h_file);
        end

        #30 rst_n = 1'b1;
        #20;
        
        // CHANGE OPCODE HERE FOR THIS SIMULATION RUN 
        // 5'b00001 = POLYMUL
        // 5'b00010 = POLYADD
        // 5'b00011 = BINADD
        // 5'b00100 = SAMPLE
        // 5'b00110 = ADDE
        opcode = 5'b00010;  // example: ADDE using loaded H
        
        // Example fixed inputs: D=5, G=2, B=all ones
        d_poly = 32'd5;
        g_poly = 32'd2;
        b_poly = 32'hFFFF_FFFF;
        
        $display("\n>>> Running test with:");
        $display("    Opcode = 5'b%b", opcode);
        $display("    D = %0d (0x%08h)", d_poly, d_poly);
        $display("    G = %0d (0x%08h)", g_poly, g_poly);
        $display("    B = 0x%08h", b_poly);
        $display("    H = 0x%09h (LSB32=0x%08h)", h_poly, h_poly[31:0]);
        $display("");
        
        #20;
        
        // Start the operation
        start = 1'b1;
        #10;
        start = 1'b0;
        
        // Wait for done signal
        wait(done == 1'b1);
        #10;
        
        // Display result based on opcode
        $display("\n========================================");
        case(opcode)
            5'b00001: begin
                $display("Operation: POLYMUL (NEGACYCLIC)");
                $display("W (36b)      = 0x%09h", w_poly);
                $display("W_LSB32      = 0x%08h", w_poly[31:0]);
            end
            
            5'b00010: begin
                $display("Operation: POLYADD (NO ERROR, integer mod 7)");
                $display("Expected W_LSB3 = %0d", (d_poly + g_poly) % 7);
                $display("Actual W        = 0x%09h", w_poly);
            end
            
            5'b00011: begin
                $display("Operation: BINADD (binary polynomial add: D ^ H_LSB32)");
                $display("Expected W_LSB32 = 0x%08h", (d_poly ^ h_poly[31:0]));
                $display("Actual W         = 0x%09h", w_poly);
            end
            
            5'b00100: begin
                $display("Operation: SAMPLE");
                $display("Result H/W = 0x%09h (LSB32=0x%08h)", w_poly, w_poly[31:0]);
                $display(" SAMPLE COMPLETE, H saved to H_value.mem ");
            end
            
            5'b00110: begin
                $display("Operation: ADDE (binary poly add with error: D ^ G ^ H_LSB32)");
                $display("Expected W_LSB32 = 0x%08h", (d_poly ^ g_poly ^ h_poly[31:0]));
                $display("Actual W         = 0x%09h", w_poly);
            end
            
            default: begin
                $display("Operation: UNKNOWN");
                $display("Invalid opcode: 5'b%b", opcode);
            end
        endcase
        
        $display("========================================\n");
        
        #100;
        $finish;
    end
    
    // Simple monitor
    always @(posedge clk) begin
        if (start)
            $display("[%0t] TB: START with opcode=%b", $time, opcode);
        if (valid)
            $display("[%0t] TB: VALID=1, W=0x%09h (LSB32=0x%08h)", $time, w_poly, w_poly[31:0]);
        if (done)
            $display("[%0t] TB: DONE=1", $time);
    end
    
endmodule
