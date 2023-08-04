module tb();
    reg clk;
    wire [31:0] IR;
    wire [31:0] PC;
    cpu uut(clk, IR, PC);
    reg signed [31:0]a=32'b00000000000000000000000000000010;
    reg signed [31:0]b=32'b10000000000000000000000000000001;
    
    always begin
        #1 clk=~clk;
       end

    initial begin
        $dumpfile("PDS.vcd");
        $dumpvars(0,tb);
        clk=1'b0;
       //$display("%b", b>>>31);
    end
endmodule