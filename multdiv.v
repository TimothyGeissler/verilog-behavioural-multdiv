module half_adder(
    Data_in_A,
    Data_in_B,
    Data_out_Sum,
    Data_out_Carry
    );

    //what are the input ports.
    input Data_in_A;
    input Data_in_B;
    //What are the output ports.
    output Data_out_Sum;
     output Data_out_Carry;
     
     //Implement the Sum and Carry equations using Verilog Bit operators.
     assign Data_out_Sum = Data_in_A ^ Data_in_B;  //XOR operation
     assign Data_out_Carry = Data_in_A & Data_in_B; //AND operation
    
endmodule

module full_adder(
    Data_in_A,  //input A
    Data_in_B,  //input B
    Data_in_C,  //input C
     Data_out_Sum,
     Data_out_Carry
    );

    //what are the input ports.
    input Data_in_A;
    input Data_in_B;
     input Data_in_C;
    //What are the output ports.
    output Data_out_Sum;
     output Data_out_Carry;
     //Internal variables
     wire ha1_sum;
     wire ha2_sum;
     wire ha1_carry;
     wire ha2_carry;
     wire Data_out_Sum;
     wire Data_out_Carry;

     //Instantiate the half adder 1
    half_adder  ha1(
        .Data_in_A(Data_in_A),
        .Data_in_B(Data_in_B),
        .Data_out_Sum(ha1_sum),
        .Data_out_Carry(ha1_carry)
    );
    
    //Instantiate the half adder 2
    half_adder  ha2(
        .Data_in_A(Data_in_C),
        .Data_in_B(ha1_sum),
        .Data_out_Sum(ha2_sum),
        .Data_out_Carry(ha2_carry)
    );

    //sum output from 2nd half adder is connected to full adder output
    assign Data_out_Sum = ha2_sum;  
    //The carry's from both the half adders are OR'ed to get the final carry./
    assign Data_out_Carry = ha1_carry | ha2_carry;
    
endmodule

module wallace32(compA, compB, prod, ovf);
    input [31:0] compA, compB;
    output [31:0] prod;
	output ovf; // Overflow

	wire [63:0] p;

	// If MSB = 1 do twos comp
	wire [31:0] a, b;
	assign a = compA[31] ? (~compA) + 1 : compA; // 2s comp A to positive
	assign b = compB[31] ? (~compB) + 1 : compB; // 2s comp B to positive

	// Flag to check for 2s comp flip
	wire twosComp;
	assign twosComp = compA[31] ^ compB[31];

	wire [63:0] compProd;
	assign compProd = twosComp ? (~p) + 1 : p;

	// Check overflow into high 32 bits
	wire allones, allzeros, highBit, msb_match;
	assign allones = & compProd[63:32];
	assign allzeros = | compProd[63:32];
	assign topAllSame = allones ^ allzeros; //ovf = 1 if !allzeros and !allones
	assign msb_match = allzeros ^ compProd[31];
	assign ovf = topAllSame | msb_match; //signMismatch;// || zero2it;*/


	//Check if a != 0, b != 0, and p = 0 then ovf
	/*wire zeroOvf, zero2it;
	assign zeroOvf = a && b && ~p[31:0];
	//check signs match
	wire signMismatch;
	xor(signMismatch, a[31], b[31], p[31]); //(a[31] & b[31] & p[31]) | (a[31] & ~b[31] & ~p[31]) | (~a[31] & ~b[31] & p[31]) | (~a[31] & b[31] & ~p[31]);
	
	// Ovf if mismatch | highBit
	assign zero2it = (signMismatch == 0) && (zeroOvf == 1);
	assign ovf = signMismatch;*/


	//Assign lower half of p
	assign prod = compProd[31:0];


    wire Cout;

wire [32:0] c1;
wire [32:0] c2;
wire [32:0] c3;
wire [32:0] c4;
wire [32:0] c5;
wire [32:0] c6;
wire [32:0] c7;
wire [32:0] c8;
wire [32:0] c9;
wire [32:0] c10;
wire [32:0] c11;
wire [32:0] c12;
wire [32:0] c13;
wire [32:0] c14;
wire [32:0] c15;
wire [32:0] c16;
wire [32:0] c17;
wire [32:0] c18;
wire [32:0] c19;
wire [32:0] c20;
wire [32:0] c21;
wire [32:0] c22;
wire [32:0] c23;
wire [32:0] c24;
wire [32:0] c25;
wire [32:0] c26;
wire [32:0] c27;
wire [32:0] c28;
wire [32:0] c29;
wire [32:0] c30;
wire [32:0] c31;
wire [32:0] c32;
assign c1[0] = 1'b0;
assign c2[0] = 1'b0;
assign c3[0] = 1'b0;
assign c4[0] = 1'b0;
assign c5[0] = 1'b0;
assign c6[0] = 1'b0;
assign c7[0] = 1'b0;
assign c8[0] = 1'b0;
assign c9[0] = 1'b0;
assign c10[0] = 1'b0;
assign c11[0] = 1'b0;
assign c12[0] = 1'b0;
assign c13[0] = 1'b0;
assign c14[0] = 1'b0;
assign c15[0] = 1'b0;
assign c16[0] = 1'b0;
assign c17[0] = 1'b0;
assign c18[0] = 1'b0;
assign c19[0] = 1'b0;
assign c20[0] = 1'b0;
assign c21[0] = 1'b0;
assign c22[0] = 1'b0;
assign c23[0] = 1'b0;
assign c24[0] = 1'b0;
assign c25[0] = 1'b0;
assign c26[0] = 1'b0;
assign c27[0] = 1'b0;
assign c28[0] = 1'b0;
assign c29[0] = 1'b0;
assign c30[0] = 1'b0;
assign c31[0] = 1'b0;
assign c32[0] = 1'b0;
wire [32:0] s1;
wire [32:0] s2;
wire [32:0] s3;
wire [32:0] s4;
wire [32:0] s5;
wire [32:0] s6;
wire [32:0] s7;
wire [32:0] s8;
wire [32:0] s9;
wire [32:0] s10;
wire [32:0] s11;
wire [32:0] s12;
wire [32:0] s13;
wire [32:0] s14;
wire [32:0] s15;
wire [32:0] s16;
wire [32:0] s17;
wire [32:0] s18;
wire [32:0] s19;
wire [32:0] s20;
wire [32:0] s21;
wire [32:0] s22;
wire [32:0] s23;
wire [32:0] s24;
wire [32:0] s25;
wire [32:0] s26;
wire [32:0] s27;
wire [32:0] s28;
wire [32:0] s29;
wire [32:0] s30;
wire [32:0] s31;
wire [32:0] s32;
wire [32:0] s33;
wire [31:0] w1;
wire [31:0] w2;
wire [31:0] w3;
wire [31:0] w4;
wire [31:0] w5;
wire [31:0] w6;
wire [31:0] w7;
wire [31:0] w8;
wire [31:0] w9;
wire [31:0] w10;
wire [31:0] w11;
wire [31:0] w12;
wire [31:0] w13;
wire [31:0] w14;
wire [31:0] w15;
wire [31:0] w16;
wire [31:0] w17;
wire [31:0] w18;
wire [31:0] w19;
wire [31:0] w20;
wire [31:0] w21;
wire [31:0] w22;
wire [31:0] w23;
wire [31:0] w24;
wire [31:0] w25;
wire [31:0] w26;
wire [31:0] w27;
wire [31:0] w28;
wire [31:0] w29;
wire [31:0] w30;
wire [31:0] w31;
wire [31:0] w32;

    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin
            assign s1[i] = a[i] & b[0];
        end
    endgenerate

//--------------- LVL 2 --------------------------
genvar ab;
generate
	for (ab = 0; ab < 31; ab = ab + 1) begin
		assign w1[ab] = a[ab] & b[1];
		full_adder f2(w1[ab], s1[ab + 1], c1[ab], s2[ab], c1[ab + 1]);
	end
endgenerate
assign w1[31] = a[31] & b[1];
full_adder f313(w1[31], 1'b0, c1[31], s2[31], c1[32]);

//--------------- LVL 3 --------------------------
genvar ac;
generate
	for (ac = 0; ac < 31; ac = ac + 1) begin
		assign w2[ac] = a[ac] & b[2];
		full_adder f3(w2[ac], s2[ac + 1], c2[ac], s3[ac], c2[ac + 1]);
	end
endgenerate
assign w2[31] = a[31] & b[2];
full_adder f314(w2[31], c1[32], c2[31], s3[31], c2[32]);

//--------------- LVL 4 --------------------------
genvar ad;
generate
	for (ad = 0; ad < 31; ad = ad + 1) begin
		assign w3[ad] = a[ad] & b[3];
		full_adder f4(w3[ad], s3[ad + 1], c3[ad], s4[ad], c3[ad + 1]);
	end
endgenerate
assign w3[31] = a[31] & b[3];
full_adder f315(w3[31], c2[32], c3[31], s4[31], c3[32]);

//--------------- LVL 5 --------------------------
genvar ae;
generate
	for (ae = 0; ae < 31; ae = ae + 1) begin
		assign w4[ae] = a[ae] & b[4];
		full_adder f5(w4[ae], s4[ae + 1], c4[ae], s5[ae], c4[ae + 1]);
	end
endgenerate
assign w4[31] = a[31] & b[4];
full_adder f316(w4[31], c3[32], c4[31], s5[31], c4[32]);

//--------------- LVL 6 --------------------------
genvar af;
generate
	for (af = 0; af < 31; af = af + 1) begin
		assign w5[af] = a[af] & b[5];
		full_adder f6(w5[af], s5[af + 1], c5[af], s6[af], c5[af + 1]);
	end
endgenerate
assign w5[31] = a[31] & b[5];
full_adder f317(w5[31], c4[32], c5[31], s6[31], c5[32]);

//--------------- LVL 7 --------------------------
genvar ag;
generate
	for (ag = 0; ag < 31; ag = ag + 1) begin
		assign w6[ag] = a[ag] & b[6];
		full_adder f7(w6[ag], s6[ag + 1], c6[ag], s7[ag], c6[ag + 1]);
	end
endgenerate
assign w6[31] = a[31] & b[6];
full_adder f318(w6[31], c5[32], c6[31], s7[31], c6[32]);

//--------------- LVL 8 --------------------------
genvar ah;
generate
	for (ah = 0; ah < 31; ah = ah + 1) begin
		assign w7[ah] = a[ah] & b[7];
		full_adder f8(w7[ah], s7[ah + 1], c7[ah], s8[ah], c7[ah + 1]);
	end
endgenerate
assign w7[31] = a[31] & b[7];
full_adder f319(w7[31], c6[32], c7[31], s8[31], c7[32]);

//--------------- LVL 9 --------------------------
genvar ai;
generate
	for (ai = 0; ai < 31; ai = ai + 1) begin
		assign w8[ai] = a[ai] & b[8];
		full_adder f9(w8[ai], s8[ai + 1], c8[ai], s9[ai], c8[ai + 1]);
	end
endgenerate
assign w8[31] = a[31] & b[8];
full_adder f3110(w8[31], c7[32], c8[31], s9[31], c8[32]);

//--------------- LVL 10 --------------------------
genvar aj;
generate
	for (aj = 0; aj < 31; aj = aj + 1) begin
		assign w9[aj] = a[aj] & b[9];
		full_adder f10(w9[aj], s9[aj + 1], c9[aj], s10[aj], c9[aj + 1]);
	end
endgenerate
assign w9[31] = a[31] & b[9];
full_adder f3111(w9[31], c8[32], c9[31], s10[31], c9[32]);

//--------------- LVL 11 --------------------------
genvar ak;
generate
	for (ak = 0; ak < 31; ak = ak + 1) begin
		assign w10[ak] = a[ak] & b[10];
		full_adder f11(w10[ak], s10[ak + 1], c10[ak], s11[ak], c10[ak + 1]);
	end
endgenerate
assign w10[31] = a[31] & b[10];
full_adder f3112(w10[31], c9[32], c10[31], s11[31], c10[32]);

//--------------- LVL 12 --------------------------
genvar al;
generate
	for (al = 0; al < 31; al = al + 1) begin
		assign w11[al] = a[al] & b[11];
		full_adder f12(w11[al], s11[al + 1], c11[al], s12[al], c11[al + 1]);
	end
endgenerate
assign w11[31] = a[31] & b[11];
full_adder f3113(w11[31], c10[32], c11[31], s12[31], c11[32]);

//--------------- LVL 13 --------------------------
genvar am;
generate
	for (am = 0; am < 31; am = am + 1) begin
		assign w12[am] = a[am] & b[12];
		full_adder f13(w12[am], s12[am + 1], c12[am], s13[am], c12[am + 1]);
	end
endgenerate
assign w12[31] = a[31] & b[12];
full_adder f3114(w12[31], c11[32], c12[31], s13[31], c12[32]);

//--------------- LVL 14 --------------------------
genvar an;
generate
	for (an = 0; an < 31; an = an + 1) begin
		assign w13[an] = a[an] & b[13];
		full_adder f14(w13[an], s13[an + 1], c13[an], s14[an], c13[an + 1]);
	end
endgenerate
assign w13[31] = a[31] & b[13];
full_adder f3115(w13[31], c12[32], c13[31], s14[31], c13[32]);

//--------------- LVL 15 --------------------------
genvar ao;
generate
	for (ao = 0; ao < 31; ao = ao + 1) begin
		assign w14[ao] = a[ao] & b[14];
		full_adder f15(w14[ao], s14[ao + 1], c14[ao], s15[ao], c14[ao + 1]);
	end
endgenerate
assign w14[31] = a[31] & b[14];
full_adder f3116(w14[31], c13[32], c14[31], s15[31], c14[32]);

//--------------- LVL 16 --------------------------
genvar ap;
generate
	for (ap = 0; ap < 31; ap = ap + 1) begin
		assign w15[ap] = a[ap] & b[15];
		full_adder f16(w15[ap], s15[ap + 1], c15[ap], s16[ap], c15[ap + 1]);
	end
endgenerate
assign w15[31] = a[31] & b[15];
full_adder f3117(w15[31], c14[32], c15[31], s16[31], c15[32]);

//--------------- LVL 17 --------------------------
genvar aq;
generate
	for (aq = 0; aq < 31; aq = aq + 1) begin
		assign w16[aq] = a[aq] & b[16];
		full_adder f17(w16[aq], s16[aq + 1], c16[aq], s17[aq], c16[aq + 1]);
	end
endgenerate
assign w16[31] = a[31] & b[16];
full_adder f3118(w16[31], c15[32], c16[31], s17[31], c16[32]);

//--------------- LVL 18 --------------------------
genvar ar;
generate
	for (ar = 0; ar < 31; ar = ar + 1) begin
		assign w17[ar] = a[ar] & b[17];
		full_adder f18(w17[ar], s17[ar + 1], c17[ar], s18[ar], c17[ar + 1]);
	end
endgenerate
assign w17[31] = a[31] & b[17];
full_adder f3119(w17[31], c16[32], c17[31], s18[31], c17[32]);

//--------------- LVL 19 --------------------------
genvar as;
generate
	for (as = 0; as < 31; as = as + 1) begin
		assign w18[as] = a[as] & b[18];
		full_adder f19(w18[as], s18[as + 1], c18[as], s19[as], c18[as + 1]);
	end
endgenerate
assign w18[31] = a[31] & b[18];
full_adder f3120(w18[31], c17[32], c18[31], s19[31], c18[32]);

//--------------- LVL 20 --------------------------
genvar at;
generate
	for (at = 0; at < 31; at = at + 1) begin
		assign w19[at] = a[at] & b[19];
		full_adder f20(w19[at], s19[at + 1], c19[at], s20[at], c19[at + 1]);
	end
endgenerate
assign w19[31] = a[31] & b[19];
full_adder f3121(w19[31], c18[32], c19[31], s20[31], c19[32]);

//--------------- LVL 21 --------------------------
genvar au;
generate
	for (au = 0; au < 31; au = au + 1) begin
		assign w20[au] = a[au] & b[20];
		full_adder f21(w20[au], s20[au + 1], c20[au], s21[au], c20[au + 1]);
	end
endgenerate
assign w20[31] = a[31] & b[20];
full_adder f3122(w20[31], c19[32], c20[31], s21[31], c20[32]);

//--------------- LVL 22 --------------------------
genvar av;
generate
	for (av = 0; av < 31; av = av + 1) begin
		assign w21[av] = a[av] & b[21];
		full_adder f22(w21[av], s21[av + 1], c21[av], s22[av], c21[av + 1]);
	end
endgenerate
assign w21[31] = a[31] & b[21];
full_adder f3123(w21[31], c20[32], c21[31], s22[31], c21[32]);

//--------------- LVL 23 --------------------------
genvar aw;
generate
	for (aw = 0; aw < 31; aw = aw + 1) begin
		assign w22[aw] = a[aw] & b[22];
		full_adder f23(w22[aw], s22[aw + 1], c22[aw], s23[aw], c22[aw + 1]);
	end
endgenerate
assign w22[31] = a[31] & b[22];
full_adder f3124(w22[31], c21[32], c22[31], s23[31], c22[32]);

//--------------- LVL 24 --------------------------
genvar ax;
generate
	for (ax = 0; ax < 31; ax = ax + 1) begin
		assign w23[ax] = a[ax] & b[23];
		full_adder f24(w23[ax], s23[ax + 1], c23[ax], s24[ax], c23[ax + 1]);
	end
endgenerate
assign w23[31] = a[31] & b[23];
full_adder f3125(w23[31], c22[32], c23[31], s24[31], c23[32]);

//--------------- LVL 25 --------------------------
genvar ay;
generate
	for (ay = 0; ay < 31; ay = ay + 1) begin
		assign w24[ay] = a[ay] & b[24];
		full_adder f25(w24[ay], s24[ay + 1], c24[ay], s25[ay], c24[ay + 1]);
	end
endgenerate
assign w24[31] = a[31] & b[24];
full_adder f3126(w24[31], c23[32], c24[31], s25[31], c24[32]);

//--------------- LVL 26 --------------------------
genvar az;
generate
	for (az = 0; az < 31; az = az + 1) begin
		assign w25[az] = a[az] & b[25];
		full_adder f26(w25[az], s25[az + 1], c25[az], s26[az], c25[az + 1]);
	end
endgenerate
assign w25[31] = a[31] & b[25];
full_adder f3127(w25[31], c24[32], c25[31], s26[31], c25[32]);

//--------------- LVL 27 --------------------------
genvar ba;
generate
	for (ba = 0; ba < 31; ba = ba + 1) begin
		assign w26[ba] = a[ba] & b[26];
		full_adder f27(w26[ba], s26[ba + 1], c26[ba], s27[ba], c26[ba + 1]);
	end
endgenerate
assign w26[31] = a[31] & b[26];
full_adder f3128(w26[31], c25[32], c26[31], s27[31], c26[32]);

//--------------- LVL 28 --------------------------
genvar bb;
generate
	for (bb = 0; bb < 31; bb = bb + 1) begin
		assign w27[bb] = a[bb] & b[27];
		full_adder f28(w27[bb], s27[bb + 1], c27[bb], s28[bb], c27[bb + 1]);
	end
endgenerate
assign w27[31] = a[31] & b[27];
full_adder f3129(w27[31], c26[32], c27[31], s28[31], c27[32]);

//--------------- LVL 29 --------------------------
genvar bc;
generate
	for (bc = 0; bc < 31; bc = bc + 1) begin
		assign w28[bc] = a[bc] & b[28];
		full_adder f29(w28[bc], s28[bc + 1], c28[bc], s29[bc], c28[bc + 1]);
	end
endgenerate
assign w28[31] = a[31] & b[28];
full_adder f3130(w28[31], c27[32], c28[31], s29[31], c28[32]);

//--------------- LVL 30 --------------------------
genvar bd;
generate
	for (bd = 0; bd < 31; bd = bd + 1) begin
		assign w29[bd] = a[bd] & b[29];
		full_adder f30(w29[bd], s29[bd + 1], c29[bd], s30[bd], c29[bd + 1]);
	end
endgenerate
assign w29[31] = a[31] & b[29];
full_adder f3131(w29[31], c28[32], c29[31], s30[31], c29[32]);

//--------------- LVL 31 --------------------------
genvar be;
generate
	for (be = 0; be < 31; be = be + 1) begin
		assign w30[be] = a[be] & b[30];
		full_adder f31(w30[be], s30[be + 1], c30[be], s31[be], c30[be + 1]);
	end
endgenerate
assign w30[31] = a[31] & b[30];
full_adder f3132(w30[31], c29[32], c30[31], s31[31], c30[32]);

//--------------- LVL 32 --------------------------
genvar bf;
generate
	for (bf = 0; bf < 31; bf = bf + 1) begin
		assign w31[bf] = a[bf] & b[31];
		full_adder f32(w31[bf], s31[bf + 1], c31[bf], s32[bf], c31[bf + 1]);
	end
endgenerate
assign w31[31] = a[31] & b[31];
full_adder f3133(w31[31], c30[32], c31[31], s32[31], c31[32]);

genvar m; 
    generate
        for (m = 0; m < 31; m = m + 1) begin
            assign w32[m] = a[m] & 1'b0;
            full_adder fend(w32[m], s32[m + 1], c32[m], s33[m], c32[m + 1]);
        end
    endgenerate

    assign w32[31] = a[31] & 1'b0;
    full_adder fend2(w32[31], c31[32], c32[31], s33[31], Cout);

    // Assign outputs
assign p[0] = s1[0];
assign p[1] = s2[0];
assign p[2] = s3[0];
assign p[3] = s4[0];
assign p[4] = s5[0];
assign p[5] = s6[0];
assign p[6] = s7[0];
assign p[7] = s8[0];
assign p[8] = s9[0];
assign p[9] = s10[0];
assign p[10] = s11[0];
assign p[11] = s12[0];
assign p[12] = s13[0];
assign p[13] = s14[0];
assign p[14] = s15[0];
assign p[15] = s16[0];
assign p[16] = s17[0];
assign p[17] = s18[0];
assign p[18] = s19[0];
assign p[19] = s20[0];
assign p[20] = s21[0];
assign p[21] = s22[0];
assign p[22] = s23[0];
assign p[23] = s24[0];
assign p[24] = s25[0];
assign p[25] = s26[0];
assign p[26] = s27[0];
assign p[27] = s28[0];
assign p[28] = s29[0];
assign p[29] = s30[0];
assign p[30] = s31[0];
assign p[31] = s32[0];
assign p[32] = s33[0];
assign p[33] = s33[0];
assign p[34] = s33[1];
assign p[35] = s33[2];
assign p[36] = s33[3];
assign p[37] = s33[4];
assign p[38] = s33[5];
assign p[39] = s33[6];
assign p[40] = s33[7];
assign p[41] = s33[8];
assign p[42] = s33[9];
assign p[43] = s33[10];
assign p[44] = s33[11];
assign p[45] = s33[12];
assign p[46] = s33[13];
assign p[47] = s33[14];
assign p[48] = s33[15];
assign p[49] = s33[16];
assign p[50] = s33[17];
assign p[51] = s33[18];
assign p[52] = s33[19];
assign p[53] = s33[20];
assign p[54] = s33[21];
assign p[55] = s33[22];
assign p[56] = s33[23];
assign p[57] = s33[24];
assign p[58] = s33[25];
assign p[59] = s33[26];
assign p[60] = s33[27];
assign p[61] = s33[28];
assign p[62] = s33[29];
assign p[63] = s33[30];
endmodule

 module div(  
   input clk,  
   input reset,  
   input start,  
   input [31:0] A,  
   input [31:0] B,  
   output [31:0] D,  
   output [31:0] R,  
   output ok,   // =1 when ready to get the result   
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
   wire [31:0] remainder, div_res, mult_res;
   wire div_ex, mult_ex, div_ready;
   reg divLatch;
   wire ex, ready;
   wire [31:0] result;
   
    div divider(clock, 1'b0, ctrl_DIV, data_operandA, data_operandB, div_res, remainder, div_ready, div_ex);
    wallace32 mult(data_operandA, data_operandB, mult_res, mult_ex);

    assign data_exception = ex;
    assign data_result = result;
    assign data_resultRDY = ready;

    assign result = divLatch ? div_res : mult_res;
    assign ex = divLatch ? div_ex : mult_ex;
    assign ready = divLatch ? div_ready : 1'b1;

    always @(posedge clock) begin
      if (ctrl_DIV) begin
        divLatch = 1;
      end else if (ctrl_MULT) begin
        divLatch = 0;
      end
    end
endmodule