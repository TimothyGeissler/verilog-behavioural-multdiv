module dffe(q, d, clk, en, clr);
   //Inputs
   input d, clk, en, clr;
   //Internal wire
   wire clr;
   //Output
   output q;
   //Register
   reg q;
   //Intialize q to 0
   initial
   begin
       q = 1'b0;
   end

   //Set value of q on positive edge of the clock or clear
   always @(posedge clk or posedge clr) begin
       //If clear is high, set q to 0
       if (clr) begin
           q <= 1'b0;
       //If enable is high, set q to the value of d
       end else if (en) begin
           q <= d;
       end
   end
endmodule

 module div(  
   input      clk,  
       input                     reset,  
   input      start,  
   input [31:0]  A,  
   input [31:0]  B,  
   output [31:0]  D,  
   output [31:0]  R,  
   output     ok ,   // =1 when ready to get the result   
       output err  
   );  
   reg       active;   // True if the divider is running  
   reg [4:0]    cycle;   // Number of cycles to go  
   reg [31:0]   result;   // Begin with A, end with D  
   reg [31:0]   denom;   // B  
   reg [31:0]   work;    // Running R  

   // Send the results to our master  
   //assign D = result;  
   assign R = work;  
   //assign ok = ~active;
 // fpga4student.com FPGA projects, Verilog projects, VHDL projects   
   // The state machine  
   /*always @(posedge clk,posedge reset) begin  
     if (reset) begin  
       active <= 0;  
       cycle <= 0;  
       result <= 0;  
       denom <= 0;  
       work <= 0;  
     end if(start || running) begin  
      running <= 1'b1;
       if (active) begin  
         // Run an iteration of the divide.  
         if (sub[32] == 0) begin  
           work <= sub[31:0];  
           result <= {result[30:0], 1'b1};  
         end  
         else begin  
           work <= {work[30:0], result[31]};  
           result <= {result[30:0], 1'b0};  
         end  
         if (cycle == 0) begin  
           active <= 0;  
         end  
         cycle <= cycle - 5'd1;  
       end else begin  
         // Set up for an unsigned divide.  
         cycle <= 5'd31;  
         result <= A;  
         denom <= B;  
         work <= 32'b0;  
         active <= 1;  
       end  
     end  
   end  */

  reg running, done;
  reg [31:0] m;
  reg [31:0] mComp; //twos comp of m
  reg [63:0] aq;
  reg [4:0] n; // counter

  assign ok = done;

  // temp regs
  reg [31:0] subReg, addReg;

  //assign ok = ~(start | running);

  // If MSB = 1 do twos comp
	wire [31:0] a, b;
	assign a = A[31] ? (~A) + 1 : A; // 2s comp A to positive
	assign b = B[31] ? (~B) + 1 : B; // 2s comp B to positive

  // Flag to check for 2s comp flip
	wire twosComp;
	assign twosComp = A[31] ^ B[31];

  wire [63:0] compRes, ans;
	assign compRes = twosComp ? (~result) + 1 : result;
  assign ans = compRes * 2;

  assign err = (~|B) ? 1'b1 : 1'b0;
  assign D = (~|B) ? 1'b0 : ans;

  always @(posedge clk) begin

    if (start) begin
      //Initialize regs
      m = b;
      mComp = (~b) + 1'b1; 
      aq[63:32] = 32'b0; // high half = 0
      aq[31:0] = a; // low half = A
      n = 5'd31; // from 31 to 0
      running = 1; // keep running
      done = 0;
    end 
    if (running) begin
      if (n == 0) begin
        running = 0; // stop running

        //final branch
        addReg = aq[63:32] + m;
        aq[63:32] = aq[63] ? addReg : aq[63:32];

        result = aq[31:0];
        done = 1'b1;
      end
      
      aq = aq << 1;//sla aq
      
      // Precalc first branchesl
      subReg = aq[63:32] + mComp;
      addReg = aq[63:32] + m;
      //First split
      aq[63:32] = aq[63] ? addReg : subReg;

      //second branch
      aq[0] = aq[63] ? 1'b0 : 1'b1; 

      //decrement counter
      n = n - 5'd1;
    end
   end
 endmodule   

module multdiv(
	data_operandA, data_operandB, 
	ctrl_MULT, ctrl_DIV, 
	clock, 
	data_result, data_exception, data_resultRDY);

    input [31:0] data_operandA, data_operandB;
    input ctrl_MULT, ctrl_DIV, clock;

    output [31:0] data_result;
    output data_exception, data_resultRDY;

    // add your code here
    /*
    input clk,  
       input reset,  
   input start,  
   input [31:0] A,  
   input [31:0] B,  
   output [31:0] D,  
   output [31:0] R,  
   output ok,   // =1 when ready to get the result   
    output err   */
   wire [31:0] remainder;
    div divider(clock, 1'b0, ctrl_DIV, data_operandA, data_operandB, data_result, remainder, data_resultRDY, data_exception);

endmodule