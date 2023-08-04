module VEDA(reset, we, mode, address, data_in, data_out); //module for memory
  input clk;
  input reset;
  input we;
  input mode;
  input [31:0] address;
  input [31:0] data_in;
  output [31:0] data_out;

  reg [31:0] mem[127:0];
  reg [31:0] data_reg;
   
  initial begin
    mem[128'd0] = 32'b000000_11111_11111_11111_00000_100000;
    mem[128'd1] = 32'b000000_00000_00000_01000_00000_100000;
    mem[128'd2] = 32'b001000_10001_10111_1111111111111111;
    //loop1
    mem[128'd3] = 32'b000000_01000_10111_11000_00000_101010;
    mem[128'd4] = 32'b000100_11000_00000_0000000000010101;
    mem[128'd5] = 32'b000000_00000_00000_01001_00000_100000;
    //loop2
    mem[128'd6] = 32'b000000_10111_01000_11001_00000_100010;
    mem[128'd7] = 32'b000000_01001_11001_11000_00000_101010;
    mem[128'd8] = 32'b000100_11000_00000_0000000000010011;
    mem[128'd9] = 32'b000000_01001_00000_01010_00000_000000;
    mem[128'd10] = 32'b000000_01010_10000_01011_00000_100000;
    mem[128'd11]= 32'b100011_01011_10010_0000000000000000;
    mem[128'd12]= 32'b001000_01011_01100_0000000000000001;
    mem[128'd13]= 32'b100011_01100_10011_0000000000000000;
    mem[128'd14]= 32'b000000_10011_10010_01101_00000_101010;
    mem[128'd15]= 32'b000100_01101_00000_0000000000010001;
    mem[128'd16]= 32'b101011_01100_10010_0000000000000000;
    mem[128'd17]= 32'b101011_01011_10011_0000000000000000;
    //if
    mem[128'd18]= 32'b001000_01001_01001_0000000000000001; 
    mem[128'd19]= 32'b000010_00000000000000000000000110;
    //loop2exit
    mem[128'd20]= 32'b001000_01000_01000_0000000000000001;
    mem[128'd21]= 32'b000010_00000000000000000000000011;
    //loop1exit
    mem[128'd22]= 32'b111111_00000000000000000000000000;
    mem[128'd32]=32'd15;
    mem[128'd33]=32'd10;
    mem[128'd34]=32'd12;
    mem[128'd35]=32'd8;
    mem[128'd36]=32'd21;
    mem[128'd37]=32'd56;
    mem[128'd38]=32'd2;
    mem[128'd39]=32'd30;

    $monitor("%d, %d, %d, %d, %d", mem[128'd32], mem[128'd33], mem[128'd34], mem[128'd35], mem[128'd36], mem[128'd37], mem[128'd38], mem[128'd39]);
  end
  always @* begin
    if (reset) begin
      data_reg <= 0;
    end else if (mode == 0) begin 
      if (we) begin
        mem[address] <= data_in;
      end 
      data_reg <= data_in;
    end else begin
      data_reg <= mem[address];
    end
  end

  assign data_out = data_reg;
endmodule

/*
Below this is the module for the ALU of the processor
I takes 7 inputs and 1 output. I will break them down below:-
alu_en --> Basically turns the ALU on and off as per need. If alu_en=0, whatever the inputs be, the output will not change
type1, type2 --> Decoded instructions from the CPU, they will be used by the ALU to identify what operation needs to be carried out on RS and RT
RS, RT --> Basically the 2 operands
SHAMT --> Used during sll and srl instructions
IMM --> Used for I-type instructions
RD--> Is the lone output which is the result of the Arithemtic or Logical Operation
*/
module alu(alu_en,type1, type2, RS, RT, RD, SHAMT, IMM); 
    input [1:0] type1;
    input [5:0] type2;
    input [31:0] RS;
    input [31:0] RT;
    output reg [31:0] RD;
    input [4:0] SHAMT;
    input [31:0] IMM;
    input alu_en;

    always@* begin
      if(alu_en===1) begin
        if(type1===0) begin
            if(type2===0) begin 
              RD=RS+RT;
            end 
            if(type2===1) begin
              RD=RS-RT;
            end
            if(type2===2) begin
              RD=RS+RT;
            end
            if(type2===3) begin
              RD=RS-RT;
            end
            if(type2===4) begin
               RD=RS&RT;
            end
            if(type2===5) begin
               RD=RS|RT;
            end
            if(type2===6) begin
               RD=RS<<SHAMT;
            end
            if(type2===7) begin
               RD=RS>>SHAMT;
            end
            if(type2===9) begin
               if(RS<RT) RD=1;
               else RD=0;
            end
        end
        else if(type1===3) begin
          if(type2===0) begin
             RD=RS+IMM;;
          end
          if(type2===1) begin
            RD=RS+IMM;
          end
          if(type2===2) begin
            RD=RS&IMM;
          end
          if(type2===3) begin
            RD=RS|IMM;
          end
          if(type2===12) begin
            if(RS<IMM) RD=1;
            else RD=0;
          end
        end
      end
    end

endmodule

/*
Below is the decoder for the CPU, which decodes what type of instruction is stored in the IR, 
Input:- IR
Output:- type1, type2
*/
module decoder(IR, type1, type2);
  input [31:0] IR;
  output [1:0]type1;
  output [5:0]type2;

  reg [1:0] type_1;
  reg [31:0] type_2;

  reg [31:0] shifted_opcode1;
  reg [31:0] shifted_opcode2;

  always@* begin
    shifted_opcode1=IR >> 26;

    if(shifted_opcode1===0) begin
      type_1=0;
      shifted_opcode2=(IR<<26)>>26;
      if(shifted_opcode2===32) type_2=0;//add-alu
      else if(shifted_opcode2===34) type_2=1;//sub-alu
      else if(shifted_opcode2===33) type_2=2;//addu-alu
      else if(shifted_opcode2===35) type_2=3;//subu-alu
      else if(shifted_opcode2===36) type_2=4;//and-alu
      else if(shifted_opcode2===37) type_2=5;//or-alu
      else if(shifted_opcode2===0) type_2=6;//sll-alu
      else if(shifted_opcode2===2) type_2=7;//srl-alu
      else if(shifted_opcode2===8) type_2=8;//jr-branch
      else if(shifted_opcode2===42) type_2=9;//slt-alu
    end
    else if(shifted_opcode1===2) begin
      type_1=1;
    end
    else if(shifted_opcode1===3) begin
      type_1=2;
    end
    else begin
      if(shifted_opcode1===8) type_2=0;//addi-alu
      else if(shifted_opcode1===9) type_2=1;//addui-alu
      else if(shifted_opcode1===12) type_2=2;//andi-alu
      else if(shifted_opcode1===13) type_2=3;//ori-alu
      else if(shifted_opcode1===35) type_2=4;//lw-dt
      else if(shifted_opcode1===43) type_2=5;//sw-dt
      else if(shifted_opcode1===4) type_2=6;//beq-branch
      else if(shifted_opcode1===5) type_2=7;//bne-branch
      else if(shifted_opcode1===7) type_2=8;//bgt-branch
      else if(shifted_opcode1===1) type_2=9;//bgte-branch
      else if(shifted_opcode1===14) type_2=10;//ble-branch
      else if(shifted_opcode1===15) type_2=11;//bleq-branch
      else if(shifted_opcode1===10) type_2=12;//stli-alu
      else if(shifted_opcode1===63) type_2=13;//exit

      type_1=3;
    end
  end  

  assign type1=type_1;
  assign type2=type_2;
endmodule

module cpu(clk, IR, PC);
  input clk;
  output reg [31:0]PC; //Program Counter to store the address of the next instruction to be executed
  output wire signed[31:0]IR; //Stores the instruction to be executed
  wire [31:0]RD_i; //RD register to be used in various instructions. More details would be specifed ahead
  reg [31:0]RS_i; //RS refister to be used in various instructions. More details would be specified ahead
  reg [31:0]RT_i; //RT register to be used in various instructions. More details would be specified ahead
  wire [31:0]RD_d; //RD register to be used in various instructions. More details would be specifed ahead
  reg [31:0]RS_d; //RS refister to be used in various instructions. More details would be specified ahead
  reg [31:0]RD_d2;
  reg [31:0]registers[31:0]; //The 32 registers 0-31 which would be used to store and carry out various instructrions
  /*
   The breakdown of the above 32 registers is similar to the MIPS structure, their breakdown is as follows:-
   $0 --> Always stores 0
   $1 ($at) --> Assembler temporary
   $2-$3($v0-$v1) --> Stores return values for function calls
   $4-$7($a0-$a3) --> First four parameters of a function call
   $8-$15($t0-$t7) --> Temporary varible
   $16-$23($s0-$s7) --> Saved Registers
   $24-$25($t8-$t9) --> 2 more temp variables
   $26-$27($k0-$k1) --> kernel use registers
   $28($gp) --> global pointers
   $29($sp) --> Stack pointers
   $30($fp) --> Frame pointer
   $31($ra) --> Stores return address of procedural call
  */
  wire [1:0] type1; //output from the decoder module
  wire [5:0] type2; //output from the decoder module
  reg [4:0] SHAMT; //SHAMT defined in various machine codes
  reg [31:0] IMM; //IMM defined in various machine codes
  reg alu_en; // Used for enabling and consequently disabling the ALU
  initial begin PC=5'b00000;
  end
  reg we,mode;
  initial begin
    registers[0]=0;
    registers[16]=32;
    registers[17]=8;
  end

  //instantiation of various modules used by the processor
  VEDA memory_instr(1'b0, 1'b0, 1'b1, PC, 0, IR);
  VEDA memory_data(1'b0, we, mode, RS_d+IMM, RD_d2, RD_d);
  decoder instr(IR, type1, type2);
  alu ALU(alu_en, type1, type2, RS_i, RT_i, RD_i, SHAMT, IMM);

  always@(posedge clk) begin 
      if(type1==0) begin
        if (type2!=8) begin
            alu_en=0;
          registers[(IR<<16)>>27]=RD_i;
          PC=PC+1;
        end
        else begin
          alu_en=0;
          PC=registers[(IR<<6)>>27];
          PC=PC+1;
        end
      end
      else if(type1==1) begin
        alu_en=0;
        PC=(IR<<6)>>6;
      end
      else if(type1===2) begin
        alu_en=0;
        registers[31]=PC;
        PC=(IR<<6)>>6;
      end
      else if(type1===3) begin
        if(type2<4 || type2===12) begin
          alu_en=0;
         registers[(IR<<11)>>27]=RD_i;
         PC=PC+1;
        end
         else if(type2===6) begin
          alu_en=0;
        if(RS_i==RT_i) PC=1+IMM;
        else PC=PC+1;
        end
        else if(type2===7) begin
          alu_en=0;
          if (!RS_i===RT_i) PC=1+IMM;
          else PC=PC+1;
        end
        else if(type2===8) begin
          alu_en=0;
          if(RS_i>RT_i) PC=1+IMM;
          else PC=PC+1;
        end
        else if(type2===9) begin
          alu_en=0;
          if(RS_i>=RT_i) PC=1+IMM;
          else PC=PC+1;
        end
        else if(type2===10) begin
          alu_en=0;
          if(RS_i<RT_i) PC=1+IMM+PC;
          else PC=PC+1;
        end
        else if(type2===11) begin
          alu_en=0;
          if(RS_i<=RT_i) PC=1+IMM;
          else PC=PC+1;
        end
        else if(type2==4) begin
            alu_en=0;
            registers[(IR<<11)>>27]=RD_d;
            PC=PC+1;
        end
        else if(type2==5) begin
            alu_en=0;
            RS_d=32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx;
            PC=PC+1;
        end
        else if(type2==13) begin
          alu_en=0;
          PC=PC;
        end
      end
  end

  always@(negedge clk) begin
      if(type1==0) begin
        if (type2!=8) begin
          alu_en=1;
          RS_i=registers[(IR<<6)>>27];
          RT_i=registers[(IR<<11)>>27];
          SHAMT=(IR<<21)>>27;
        end
      end
      if(type1==3) begin
        if(type2<4 || type2===12) begin
          alu_en=1;
          RS_i=registers[(IR<<6)>>27];
          RT_i=registers[(IR<<11)>>27];
          IMM=(IR<<16)>>>16;
        end
        if(type2==4) begin
            alu_en=0;
            we=1'b0;
            mode=1'b1;
            RS_d=registers[(IR<<6)>>27];
            IMM=(IR<<16)>>>16;
        end
        else if(type2==5) begin
            alu_en=0;
            RD_d2=registers[(IR<<11)>>27];
            we=1'b1;
            mode=1'b0;
            RS_d=registers[(IR<<6)>>27];
            IMM=(IR<<16)>>16;
        end
        if(type2==10) begin
            alu_en=0;
            RS_i=registers[(IR<<6)>>27];
            RT_i=registers[(IR<<11)>>27];
            IMM=(IR<<16)>>>16;
        end
        if(type2==6) begin
            alu_en=0;
            RS_i=registers[(IR<<6)>>27];
            RT_i=registers[(IR<<11)>>27];
            IMM=(IR<<16)>>>16;
        end
      end
  end
endmodule