/*
 * verilog model of 65C02 CPU.
 *
 * (C) Arlet Ottens, <arlet@c-scape.nl>
 * 24-bit address changes (C) Luni Libes, <https://www.lunarmobiscuit.com/the-apple-4-or-the-mos-652402/>
 *
 */

module cpu( 
    input clk,                          // CPU clock
`ifdef ONEND
    output onend,                        // trigger the end of the simulation
`endif
    input RST,                          // RST signal
    output [23:0] AB,                   // address bus (combinatorial) 
    output sync,                        // start of new instruction
    input [7:0] DI,                     // data bus input
    output reg [7:0] DO,                // data bus output 
    output WE,                          // write enable
    input IRQ,                          // interrupt request
    input NMI,                          // non-maskable interrupt request
    input RDY,                          // Ready signal. Pauses CPU when RDY=0
    input debug );                      // debug for simulation

`include "define.i"

`ifdef ONEND
    reg onend = 0;                      // a trigger for more debug messages
`endif

wire [23:0] PC;                         // program counter 

reg [7:0] D3;                           // data register, previous DR (for 24 bit addresses) 
reg [7:0] DR;                           // data register, previous DI stored in a register 
wire [7:0] IR;                          // instruction register
reg PREFIX;                             // previous instruction was a prefix opcode
reg ABWDTH;                             // width of the address bus

wire B = 1;
reg N, V, D, I, Z, C;                   // processor status flags 
wire [7:0] P = { N, V, 1'b1, B, D, I, Z, C };

wire alu_C, alu_Z, alu_V, alu_N;
wire [7:0] alu_out;

/*
 * control bits
 */

reg [5:0] state;
wire [5:0] init_state;
assign sync = (state == SYNC);
reg [17:0] control;

wire [1:0] ins_type = control[18:17];

wire adc_sbc = (ins_type == FLAG_ADD);
wire cmp     = (ins_type == FLAG_CMP);
wire bit_isn = (ins_type == FLAG_BIT);

// special instructions 
reg clc, sec, cld, sed, cli, sei, clv, plp, php, rti, txs, variation;

wire store = control[16];
wire load = control[15];

wire [8:0] alu_op = control[10:2];
wire shift = alu_op[8];
reg [3:0] cond_code;
reg cond;

// control bits per cycle
reg [19:0] bus_op;
wire [9:0] ab_op = bus_op[9:0];
wire [1:0] we_op = bus_op[11:10];
wire dr_di = bus_op[12];
wire [2:0] do_op = bus_op[15:13];
wire pull = bus_op[18];
wire push = bus_op[19];

wire [7:0] S;                         // stack pointer register
wire [7:0] R;                         // register from regfile to ALU
wire [7:0] XY;                        // index register from regfile to AB 

regfile regfile( 
    .clk(clk),
    .reg_idx(control[1:0] & bus_op[17:16]),     // YX (from decode) & reg_idx (from bus_op)
    .reg_src(control[14:13]),                   // reg_src = SRC = decode[20:19]
    .reg_dst(control[12:11]),                   // reg_dst = DST = decode[18:17]
    .reg_we(load & ~store & sync),              // load = L/S = decode[22]  // store = L/S = decode[21]
    .src(R),
    .idx(XY),
    .dst(alu_out),
    .S(S),
    .txs(txs),
    .push(push),
    .pull(pull),
    .variation(variation) );

/*
 * ALU
 */
alu alu(
    .alu_op(alu_op),        // alu_op = control[10:2] = decode[16:8]
    .R(R),
    .S(S),
    .DR(DR),
    .DI(DI),
    .C(C),
    .alu_out(alu_out),
    .alu_C(alu_C), 
    .alu_Z(alu_Z), 
    .alu_N(alu_N), 
    .alu_V(alu_V) );


/*
 * address bus
 */
ab ab(
    .clk(clk),
    .RST(RST),
    .ab_op(ab_op),          // ab_op = bus_op[9:0]
    .S(S),
    .DI(DI),
    .DR(DR),
    .D3(D3),
    .XY(XY),
    .ABWDTH(ABWDTH),
    .AB(AB),
    .PC(PC) );

/*
 * write enable
 *
 * we_op[0] is always write
 * we_op[1] is write when 'store' is set.
 */
assign WE = we_op[0] | (we_op[1] & store);  // we_op = bus_op[11:10]

/*
 * Data registers
 */
always @(posedge clk)
    if( dr_di ) begin
        D3 = DR;
        DR <= DI;
    end
        

/*
 * data output
 */
always @*
    case( do_op )
        DO_ALU: DO = alu_out;
        DO_PHP: DO = php ? P : alu_out;
        DO_PCL: DO = PC[7:0];
        DO_PCH: DO = PC[15:8];
        DO_PC3: DO = PC[23:16];
        default:; // for completeness
    endcase

/*
 * flags update
 */

/*
 * negative flag
 */
always @(posedge clk)
    case( state )
        RTS0: if( rti )                         N <= DI[7];
        SYNC: if( plp )                         N <= DI[7];
              else if( bit_isn )                N <= DR[7];
              else if( load | cmp )             N <= alu_N;
     default:; // for completeness
    endcase


/*
 * overflow flag
 */
always @(posedge clk)
    case( state )
        RTS0: if( rti )                         V <= DI[6];
        SYNC: if( plp )                         V <= DI[6];
              else if( clv )                    V <= 0;
              else if( bit_isn )                V <= DR[6];
              else if( adc_sbc )                V <= alu_V;
     default:; // for completeness
    endcase

/*
 * decimal flag
 */
always @(posedge clk)
    case( state )
        RTS0: if( rti )                         D <= DI[3];
        SYNC: if( plp )                         D <= DI[3];
              else if( cld )                    D <= 0;
              else if( sed )                    D <= 1;
     default:; // for completeness
    endcase

/*
 * interrupt flag 
 */
always @(posedge clk)
    case( state )
        BRK4:                                   I <= 1;
        RTS0: if( rti )                         I <= DI[2];
        SYNC: if( plp )                         I <= DI[2]; 
              else if( cli )                    I <= 0;
              else if( sei )                    I <= 1;
     default:; // for completeness
    endcase

/*
 * zero flag 
 */
always @(posedge clk)
    case( state )
        RTS0: if( rti )                         Z <= DI[1];
        SYNC: if( plp )                         Z <= DI[1]; 
              else if( load | cmp | bit_isn )   Z <= alu_Z;
     default:; // for completeness
    endcase

/*
 * carry flag
 */
always @(posedge clk) begin
    case( state )
        RTS0: if( rti )                         C <= DI[0];
        SYNC: if( plp )                         C <= DI[0];
              else if( clc )                    C <= 0;
              else if( sec )                    C <= 1;
              else if( cmp | shift | adc_sbc )  C <= alu_C;
     default:; // for completeness
    endcase
end

/*
 * DI hold register, holds the DI value containing the
 * next opcode for a couple of instructions where the 
 * opcode is fetched during instruction execution.
 */

reg [8:0] DIHOLD = {9'h1ea};

always @(posedge clk)
    case( state )
        PLA0: DIHOLD <= {1'b1, DI};
        PHA0: DIHOLD <= {1'b1, DI};
        RMW1: DIHOLD <= {1'b1, DI};
     default: DIHOLD <= {1'b0, DI};
    endcase

assign IR = DIHOLD[8] ? DIHOLD[7:0] : DI;

/*
 * condition code
 */
always @(posedge clk) 
    cond_code <= IR[7:4];

always @*
    casez( cond_code )
        4'b000?: cond = ~N;
        4'b001?: cond =  N;
        4'b010?: cond = ~V;
        4'b011?: cond =  V;
        4'b1000: cond =  1;
        4'b1001: cond = ~C;
        4'b101?: cond =  C;
        4'b110?: cond = ~Z;
        4'b111?: cond =  Z;
    endcase

/*
 * state machine. In the SYNC state, we decode the current
 * opcode, and then move to 'init_state'. From there, we follow
 * a sequence of cycles until we get back to the SYNC state for
 * the next opcode.
 */
always @(posedge clk)
    if( RST ) begin
        PREFIX = 1'b1;              // start in 24-bit address mode
        ABWDTH = 1'b1;              // start in 24-bit address mode
        state <= BRK4;
    end
    else case( state )
        SYNC:  state <= init_state;
        ABSW:  state <= ABS0;
        ABS0:  state <= ABS1;
        ABS1:  state <= DATA;
        ABWW:  state <= ABWW;
        ABW0:  state <= ABW1;
        ABW1:  state <= RMW0;
        BRA0:  state <= SYNC;
        BRK0:  state <= BRK1;
        BRK1:  state <= BRK2;
        BRK2:  state <= BRK3;
        BRK3:  state <= JMP0;
        BRK4:  state <= BRK3;
        DATA:  state <= SYNC;
        IDX0:  state <= IDX1;
        IDX1:  state <= IDX2;
        IDX2:  state <= DATA;
        IMM0:  state <= SYNC;
        INDW:  state <= IND0;
        IND0:  state <= IND1;
        IND1:  state <= (ABWDTH === 1'b0) ? JMP0 : JMPW;
        JMPW:  state <= JMP0;
        JMP0:  state <= JMP1;
        JMP1:  state <= SYNC;
        JSRW:  state <= JSRH;
        JSRH:  state <= JSR3;
        JSR3:  state <= JSRL;
        JSRL:  state <= JSR4;
        JSR4:  state <= JMP1;
        JSR0:  state <= JSR1;
        JSR1:  state <= JSR2;
        JSR2:  state <= JMP1;
        PHA0:  state <= SYNC;
        PLA0:  state <= SYNC;
        RMW0:  state <= RMW1;
        RMW1:  state <= SYNC;
        RTI0:  state <= RTS0;
        RTS0:  state <= (ABWDTH === 1'b0) ? RTS1 : RTSW;
        RTSW:  state <= RTS1;
        RTS1:  state <= RTS2;
        RTS2:  state <= SYNC;
        ZPG0:  state <= DATA;
        ZPW0:  state <= RMW0;
        default:  state <= SYNC;
    endcase

/*
 * decode vector
 */
reg [24:0] decode;

assign init_state = decode[5:0];


/*
 * control logic to determine operations per cycle
 */
always @* begin
    case( state )
         //                             push/pull reg_idx do_op   dr_di  we_op  ab_op
         //                                2'b    2'b     3'b     1'b    2'b    10'b
       ABSW:                    bus_op = { S____, IDX___, DO____, DR_DI, WE___, AB_OPER };          
       ABS0:                    bus_op = { S____, IDX___, DO____, DR_DI, WE___, AB_OPER };          
       ABS1:                    bus_op = { S____, IDX_XY, DO_ALU, DR___, WE_ST, AB_ABSX };			
       ABWW:                    bus_op = { S____, IDX___, DO____, DR_DI, WE___, AB_OPER };          
       ABW0:                    bus_op = { S____, IDX___, DO____, DR_DI, WE___, AB_OPER };          
       ABW1:                    bus_op = { S____, IDX_XY, DO____, DR___, WE___, AB_ABSX };			
       BRA0: if( cond & DI[7] ) bus_op = { S____, IDX___, DO____, DR___, WE___, AB_BACK };
             else if( cond )    bus_op = { S____, IDX___, DO____, DR___, WE___, AB_FWRD };
             else               bus_op = { S____, IDX___, DO____, DR___, WE___, AB_OPER };
       BRK0:                    bus_op = { S_DEC, IDX___, DO_PCH, DR___, WE_1_, AB_PUSH };			
       BRK1:                    bus_op = { S_DEC, IDX___, DO_PCL, DR___, WE_1_, AB_PUSH };			
       BRK2:                    bus_op = { S_DEC, IDX___, DO_PHP, DR___, WE_1_, AB_IRQV };			
       BRK3:                    bus_op = { S____, IDX___, DO____, DR_DI, WE___, AB_OPER };          
       BRK4:                    bus_op = { S____, IDX___, DO____, DR___, WE___, AB_OPER };          
       DATA:                    bus_op = { S____, IDX___, DO____, DR_DI, WE___, AB_OPER };			
       IDX0:                    bus_op = { S____, IDX_X_, DO____, DR_DI, WE___, AB_ZPGX };			
       IDX1:                    bus_op = { S____, IDX___, DO____, DR_DI, WE___, AB_NEXT };			
       IDX2:                    bus_op = { S____, IDX__Y, DO_ALU, DR___, WE_ST, AB_ABSX };			
       IMM0:                    bus_op = { S____, IDX___, DO____, DR_DI, WE___, AB_OPER };          
       INDW:                    bus_op = { S____, IDX___, DO____, DR_DI, WE___, AB_OPER };          
       IND0:                    bus_op = { S____, IDX___, DO____, DR_DI, WE___, AB_OPER };          
       IND1:                    bus_op = { S____, IDX_X_, DO____, DR___, WE___, AB_JMP0 };          
       JMPW:                    bus_op = { S____, IDX___, DO____, DR_DI, WE___, AB_OPER };          
       JMP0:                    bus_op = { S____, IDX___, DO____, DR_DI, WE___, AB_OPER };   
       JMP1:                    bus_op = { S____, IDX___, DO____, DR___, WE___, AB_JMP0 };          
       JSRW:                    bus_op = { S_DEC, IDX___, DO_PC3, DR_DI, WE_1_, AB_PUSH };          
       JSRH:                    bus_op = { S_DEC, IDX___, DO_PCH, DR___, WE_1_, AB_PUSH };          
       JSR3:                    bus_op = { S____, IDX___, DO____, DR___, WE___, AB_OPER };          
       JSR4:                    bus_op = { S____, IDX___, DO____, DR___, WE___, AB_OPER };          
       JSRL:                    bus_op = { S_DEC, IDX___, DO_PCL, DR_DI, WE_1_, AB_PUSH };          
       JSR0:                    bus_op = { S_DEC, IDX___, DO_PCH, DR_DI, WE_1_, AB_PUSH };          
       JSR1:                    bus_op = { S_DEC, IDX___, DO_PCL, DR___, WE_1_, AB_PUSH };			
       JSR2:                    bus_op = { S____, IDX___, DO____, DR___, WE___, AB_OPER };          
       PHA0:                    bus_op = { S_DEC, IDX___, DO_PHP, DR___, WE_1_, AB_PUSH };			
       PLA0:                    bus_op = { S_INC, IDX___, DO____, DR_DI, WE___, AB_PULL };			
       RMW0:                    bus_op = { S____, IDX___, DO____, DR_DI, WE___, AB_OPER };			
       RMW1:                    bus_op = { S____, IDX___, DO_ALU, DR___, WE_1_, AB_HOLD };			
       RTI0:                    bus_op = { S_INC, IDX___, DO____, DR___, WE___, AB_PULL };          
       RTS0:                    bus_op = { S_INC, IDX___, DO____, DR___, WE___, AB_PULL };          
       RTS1:                    bus_op = { S_INC, IDX___, DO____, DR_DI, WE___, AB_PULL };          
       RTSW:                    bus_op = { S_INC, IDX___, DO____, DR_DI, WE___, AB_PULL };          
       RTS2: if( rti )          bus_op = { S____, IDX___, DO____, DR___, WE___, AB_JMP0 };
             else               bus_op = { S____, IDX___, DO____, DR___, WE___, AB_JMP1 };
       SYNC: begin
                // wider address bus if after a prefix code (or for all RTI)
                ABWDTH = ((IR == 8'h1F) || (IR == 8'h40) || (PREFIX == 1'b1));       
                PREFIX = (IR == 8'h1F);
                                bus_op = { S____, IDX___, DO____, DR___, WE___, AB_OPER };	
             end		
       ZPG0:                    bus_op = { S____, IDX_XY, DO_ALU, DR___, WE_ST, AB_ZPGX };			
       ZPW0:                    bus_op = { S____, IDX_XY, DO____, DR___, WE___, AB_ZPGX };			
       default:                 bus_op = { S____, IDX___, DO____, DR___, WE___, AB_OPER };          // SYNC
    endcase
end

/*
 * control vector
 */
always @(posedge clk)
    if( sync ) begin
        control <= decode[24:6];
        clc <= (IR == 8'h18);
        sec <= (IR == 8'h38);
        cld <= (IR == 8'hD8);
        sed <= (IR == 8'hF8);
        cli <= (IR == 8'h58);
        sei <= (IR == 8'h78);
        rti <= (IR == 8'h40);
        clv <= (IR == 8'hB8);
        php <= (IR == 8'h08) || (IR == 8'h00);
        plp <= (IR == 8'h28);
        txs <= (IR == 8'h9A);
        variation <= (IR == 8'h0F);
    end

/*
 * decode vector
 */
always @* begin
    case( IR )
         //                type      L/S   SRC    DST    ALU      YX  STATE
         //                2'b       2'b   2'b    2'b    9'b      2'b  6'b
         //  decode[n:m]   24:23     22:21 20:19  18:17  16:8     7:6  5:0
         //  control[n:m]  18:17     16:15 14:13  12:11  10:2     1:0
         8'h6D: decode = { FLAG_ADD, LDA, SRC_A, DST_A, ALU_ADC , IZ, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // ADC ABS
         8'h7D: decode = { FLAG_ADD, LDA, SRC_A, DST_A, ALU_ADC , IX, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // ADC ABS,X
         8'h79: decode = { FLAG_ADD, LDA, SRC_A, DST_A, ALU_ADC , IY, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // ADC ABS,Y
         8'h69: decode = { FLAG_ADD, LDA, SRC_A, DST_A, ALU_ADC , IZ, IMM0 }; // ADC #IMM
         8'h65: decode = { FLAG_ADD, LDA, SRC_A, DST_A, ALU_ADC , IZ, ZPG0 }; // ADC ZP
         8'h72: decode = { FLAG_ADD, LDA, SRC_A, DST_A, ALU_ADC , IZ, IDX0 }; // ADC (ZP)
         8'h61: decode = { FLAG_ADD, LDA, SRC_A, DST_A, ALU_ADC , IX, IDX0 }; // ADC (ZP,X)
         8'h75: decode = { FLAG_ADD, LDA, SRC_A, DST_A, ALU_ADC , IX, ZPG0 }; // ADC ZP,X
         8'h71: decode = { FLAG_ADD, LDA, SRC_A, DST_A, ALU_ADC , IY, IDX0 }; // ADC (ZP),Y

         8'hED: decode = { FLAG_ADD, LDA, SRC_A, DST_A, ALU_SBC , IZ, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // SBC ABS
         8'hFD: decode = { FLAG_ADD, LDA, SRC_A, DST_A, ALU_SBC , IX, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // SBC ABS,X
         8'hF9: decode = { FLAG_ADD, LDA, SRC_A, DST_A, ALU_SBC , IY, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // SBC ABS,Y
         8'hE9: decode = { FLAG_ADD, LDA, SRC_A, DST_A, ALU_SBC , IZ, IMM0 }; // SBC #IMM
         8'hE5: decode = { FLAG_ADD, LDA, SRC_A, DST_A, ALU_SBC , IZ, ZPG0 }; // SBC ZP
         8'hF2: decode = { FLAG_ADD, LDA, SRC_A, DST_A, ALU_SBC , IZ, IDX0 }; // SBC (ZP)
         8'hE1: decode = { FLAG_ADD, LDA, SRC_A, DST_A, ALU_SBC , IX, IDX0 }; // SBC (ZP,X)
         8'hF5: decode = { FLAG_ADD, LDA, SRC_A, DST_A, ALU_SBC , IX, ZPG0 }; // SBC ZP,X
         8'hF1: decode = { FLAG_ADD, LDA, SRC_A, DST_A, ALU_SBC , IY, IDX0 }; // SBC (ZP),Y

         8'h2D: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_AND , IZ, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // AND ABS
         8'h3D: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_AND , IX, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // AND ABS,X
         8'h39: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_AND , IY, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // AND ABS,Y
         8'h29: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_AND , IZ, IMM0 }; // AND #IMM
         8'h25: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_AND , IZ, ZPG0 }; // AND ZP
         8'h32: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_AND , IZ, IDX0 }; // AND (ZP)
         8'h21: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_AND , IX, IDX0 }; // AND (ZP,X)
         8'h35: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_AND , IX, ZPG0 }; // AND ZP,X
         8'h31: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_AND , IY, IDX0 }; // AND (ZP),Y

         8'h0D: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_ORA , IZ, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // ORA ABS
         8'h1D: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_ORA , IX, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // ORA ABS,X
         8'h19: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_ORA , IY, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // ORA ABS,Y
         8'h09: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_ORA , IZ, IMM0 }; // ORA #IMM
         8'h05: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_ORA , IZ, ZPG0 }; // ORA ZP
         8'h12: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_ORA , IZ, IDX0 }; // ORA (ZP)
         8'h01: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_ORA , IX, IDX0 }; // ORA (ZP,X)
         8'h15: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_ORA , IX, ZPG0 }; // ORA ZP,X
         8'h11: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_ORA , IY, IDX0 }; // ORA (ZP),Y

         8'hAD: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_LDA , IZ, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // LDA ABS
         8'hBD: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_LDA , IX, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // LDA ABS,X
         8'hB9: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_LDA , IY, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // LDA ABS,Y
         8'hA9: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_LDA , IZ, IMM0 }; // LDA #IMM
         8'hA5: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_LDA , IZ, ZPG0 }; // LDA ZP
         8'hB2: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_LDA , IZ, IDX0 }; // LDA (ZP)
         8'hA1: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_LDA , IX, IDX0 }; // LDA (ZP,X)
         8'hB5: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_LDA , IX, ZPG0 }; // LDA ZP,X
         8'hB1: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_LDA , IY, IDX0 }; // LDA (ZP),Y

         8'hCD: decode = { FLAG_CMP, NOP, SRC_A, DST__, ALU_CMP , IZ, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // CMP ABS
         8'hDD: decode = { FLAG_CMP, NOP, SRC_A, DST__, ALU_CMP , IX, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // CMP ABS,X
         8'hD9: decode = { FLAG_CMP, NOP, SRC_A, DST__, ALU_CMP , IY, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // CMP ABS,Y
         8'hC9: decode = { FLAG_CMP, NOP, SRC_A, DST__, ALU_CMP , IZ, IMM0 }; // CMP #IMM
         8'hC5: decode = { FLAG_CMP, NOP, SRC_A, DST__, ALU_CMP , IZ, ZPG0 }; // CMP ZP
         8'hD2: decode = { FLAG_CMP, NOP, SRC_A, DST__, ALU_CMP , IZ, IDX0 }; // CMP (ZP)
         8'hC1: decode = { FLAG_CMP, NOP, SRC_A, DST__, ALU_CMP , IX, IDX0 }; // CMP (ZP,X)
         8'hD5: decode = { FLAG_CMP, NOP, SRC_A, DST__, ALU_CMP , IX, ZPG0 }; // CMP ZP,X
         8'hD1: decode = { FLAG_CMP, NOP, SRC_A, DST__, ALU_CMP , IY, IDX0 }; // CMP (ZP),Y

         8'h4D: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_EOR , IZ, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // EOR ABS
         8'h5D: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_EOR , IX, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // EOR ABS,X
         8'h59: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_EOR , IY, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // EOR ABS,Y
         8'h49: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_EOR , IZ, IMM0 }; // EOR #IMM
         8'h45: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_EOR , IZ, ZPG0 }; // EOR ZP
         8'h52: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_EOR , IZ, IDX0 }; // EOR (ZP)
         8'h41: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_EOR , IX, IDX0 }; // EOR (ZP,X)
         8'h55: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_EOR , IX, ZPG0 }; // EOR ZP,X
         8'h51: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_EOR , IY, IDX0 }; // EOR (ZP),Y

         8'h8D: decode = { FLAG____, STA, SRC_A, DST__, ALU_REG , IZ, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // STA ABS
         8'h9D: decode = { FLAG____, STA, SRC_A, DST__, ALU_REG , IX, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // STA ABS,X
         8'h99: decode = { FLAG____, STA, SRC_A, DST__, ALU_REG , IY, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // STA ABS,Y
         8'h85: decode = { FLAG____, STA, SRC_A, DST__, ALU_REG , IZ, ZPG0 }; // STA ZP
         8'h92: decode = { FLAG____, STA, SRC_A, DST__, ALU_REG , IZ, IDX0 }; // STA (ZP)
         8'h81: decode = { FLAG____, STA, SRC_A, DST__, ALU_REG , IX, IDX0 }; // STA (ZP,X)
         8'h95: decode = { FLAG____, STA, SRC_A, DST__, ALU_REG , IX, ZPG0 }; // STA ZP,X
         8'h91: decode = { FLAG____, STA, SRC_A, DST__, ALU_REG , IY, IDX0 }; // STA (ZP),Y

         8'h0A: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_ASLA, IZ, SYNC }; // ASL A
         8'h4A: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_LSRA, IZ, SYNC }; // LSR A
         8'h2A: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_ROLA, IZ, SYNC }; // ROL A
         8'h6A: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_RORA, IZ, SYNC }; // ROR A

         8'h0E: decode = { FLAG____, RMW, SRC__, DST__, ALU_ASLM, IZ, (ABWDTH == 1'b1) ? ABWW : ABW0 }; // ASL ABS
         8'h1E: decode = { FLAG____, RMW, SRC__, DST__, ALU_ASLM, IX, (ABWDTH == 1'b1) ? ABWW : ABW0 }; // ASL ABS,X
         8'h06: decode = { FLAG____, RMW, SRC__, DST__, ALU_ASLM, IZ, ZPW0 }; // ASL ZP
         8'h16: decode = { FLAG____, RMW, SRC__, DST__, ALU_ASLM, IX, ZPW0 }; // ASL ZP,X

         8'h4E: decode = { FLAG____, RMW, SRC__, DST__, ALU_LSRM, IZ, (ABWDTH == 1'b1) ? ABWW : ABW0 }; // LSR ABS
         8'h5E: decode = { FLAG____, RMW, SRC__, DST__, ALU_LSRM, IX, (ABWDTH == 1'b1) ? ABWW : ABW0 }; // LSR ABS,X
         8'h46: decode = { FLAG____, RMW, SRC__, DST__, ALU_LSRM, IZ, ZPW0 }; // LSR ZP
         8'h56: decode = { FLAG____, RMW, SRC__, DST__, ALU_LSRM, IX, ZPW0 }; // LSR ZP,X

         8'h2E: decode = { FLAG____, RMW, SRC__, DST__, ALU_ROLM, IZ, (ABWDTH == 1'b1) ? ABWW : ABW0 }; // ROL ABS
         8'h3E: decode = { FLAG____, RMW, SRC__, DST__, ALU_ROLM, IX, (ABWDTH == 1'b1) ? ABWW : ABW0 }; // ROL ABS,X
         8'h26: decode = { FLAG____, RMW, SRC__, DST__, ALU_ROLM, IZ, ZPW0 }; // ROL ZP
         8'h36: decode = { FLAG____, RMW, SRC__, DST__, ALU_ROLM, IX, ZPW0 }; // ROL ZP,X

         8'h6E: decode = { FLAG____, RMW, SRC__, DST__, ALU_RORM, IZ, (ABWDTH == 1'b1) ? ABWW : ABW0 }; // ROR ABS
         8'h7E: decode = { FLAG____, RMW, SRC__, DST__, ALU_RORM, IX, (ABWDTH == 1'b1) ? ABWW : ABW0 }; // ROR ABS,X
         8'h66: decode = { FLAG____, RMW, SRC__, DST__, ALU_RORM, IZ, ZPW0 }; // ROR ZP
         8'h76: decode = { FLAG____, RMW, SRC__, DST__, ALU_RORM, IX, ZPW0 }; // ROR ZP,X

         8'h90: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, BRA0 }; // BCC
         8'hB0: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, BRA0 }; // BCS
         8'hF0: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, BRA0 }; // BEQ
         8'h30: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, BRA0 }; // BMI
         8'hD0: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, BRA0 }; // BNE
         8'h10: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, BRA0 }; // BPL
         8'h80: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, BRA0 }; // BRA
         8'h50: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, BRA0 }; // BVC
         8'h70: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, BRA0 }; // BVS

         8'h2C: decode = { FLAG_BIT, NOP, SRC_A, DST__, ALU_AND , IZ, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // BIT ABS
         8'h3C: decode = { FLAG_BIT, NOP, SRC_A, DST__, ALU_AND , IX, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // BIT ABS,X
         8'h89: decode = { FLAG_BIT, NOP, SRC_A, DST__, ALU_AND , IZ, IMM0 }; // BIT #IMM
         8'h24: decode = { FLAG_BIT, NOP, SRC_A, DST__, ALU_AND , IZ, ZPG0 }; // BIT ZP
         8'h34: decode = { FLAG_BIT, NOP, SRC_A, DST__, ALU_AND , IX, ZPG0 }; // BIT ZP,X

         8'h18: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, SYNC }; // CLC
         8'hD8: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, SYNC }; // CLD
         8'h58: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, SYNC }; // CLI
         8'hB8: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, SYNC }; // CLV
         8'h38: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, SYNC }; // SEC
         8'hF8: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, SYNC }; // SED
         8'h78: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, SYNC }; // SEI
         8'hEA: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, SYNC }; // NOP

         8'hEC: decode = { FLAG_CMP, NOP, SRC_X, DST__, ALU_CMP , IZ, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // CPX ABS
         8'hE0: decode = { FLAG_CMP, NOP, SRC_X, DST__, ALU_CMP , IZ, IMM0 }; // CPX #IMM
         8'hE4: decode = { FLAG_CMP, NOP, SRC_X, DST__, ALU_CMP , IZ, ZPG0 }; // CPX ZP
         8'hCC: decode = { FLAG_CMP, NOP, SRC_Y, DST__, ALU_CMP , IZ, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // CPY ABS
         8'hC0: decode = { FLAG_CMP, NOP, SRC_Y, DST__, ALU_CMP , IZ, IMM0 }; // CPY #IMM
         8'hC4: decode = { FLAG_CMP, NOP, SRC_Y, DST__, ALU_CMP , IZ, ZPG0 }; // CPY ZP

         8'hCE: decode = { FLAG____, RMW, SRC__, DST__, ALU_DECM, IZ, (ABWDTH == 1'b1) ? ABWW : ABW0 }; // DEC ABS
         8'hDE: decode = { FLAG____, RMW, SRC__, DST__, ALU_DECM, IX, (ABWDTH == 1'b1) ? ABWW : ABW0 }; // DEC ABS,X
         8'hC6: decode = { FLAG____, RMW, SRC__, DST__, ALU_DECM, IZ, ZPW0 }; // DEC ZP
         8'hD6: decode = { FLAG____, RMW, SRC__, DST__, ALU_DECM, IX, ZPW0 }; // DEC ZP,X

         8'hEE: decode = { FLAG____, RMW, SRC__, DST__, ALU_INCM, IZ, (ABWDTH == 1'b1) ? ABWW : ABW0 }; // INC ABS
         8'hFE: decode = { FLAG____, RMW, SRC__, DST__, ALU_INCM, IX, (ABWDTH == 1'b1) ? ABWW : ABW0 }; // INC ABS,X
         8'hE6: decode = { FLAG____, RMW, SRC__, DST__, ALU_INCM, IZ, ZPW0 }; // INC ZP
         8'hF6: decode = { FLAG____, RMW, SRC__, DST__, ALU_INCM, IX, ZPW0 }; // INC ZP,X

         8'h3A: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_DECA, IZ, SYNC }; // DEA
         8'hCA: decode = { FLAG____, LDA, SRC_X, DST_X, ALU_DECA, IZ, SYNC }; // DEX
         8'h88: decode = { FLAG____, LDA, SRC_Y, DST_Y, ALU_DECA, IZ, SYNC }; // DEY
         8'h1A: decode = { FLAG____, LDA, SRC_A, DST_A, ALU_INCA, IZ, SYNC }; // INA
         8'hE8: decode = { FLAG____, LDA, SRC_X, DST_X, ALU_INCA, IZ, SYNC }; // INX
         8'hC8: decode = { FLAG____, LDA, SRC_Y, DST_Y, ALU_INCA, IZ, SYNC }; // INY
         8'hAA: decode = { FLAG____, LDA, SRC_A, DST_X, ALU_REG , IZ, SYNC }; // TAX
         8'hA8: decode = { FLAG____, LDA, SRC_A, DST_Y, ALU_REG , IZ, SYNC }; // TAY
         8'hBA: decode = { FLAG____, LDA, SRC__, DST_X, ALU_TSX , IZ, SYNC }; // TSX
         8'h8A: decode = { FLAG____, LDA, SRC_X, DST_A, ALU_REG , IZ, SYNC }; // TXA
         8'h9A: decode = { FLAG____, NOP, SRC_X, DST__, ALU_REG , IZ, SYNC }; // TXS
         8'h98: decode = { FLAG____, LDA, SRC_Y, DST_A, ALU_REG , IZ, SYNC }; // TYA

         8'h00: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, BRK0 }; // BRK
         8'h4C: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, (ABWDTH == 1'b1) ? JMPW : JMP0 }; // JMP ABS
         8'h6C: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, (ABWDTH == 1'b1) ? INDW : IND0 }; // JMP (IDX)
         8'h7C: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IX, (ABWDTH == 1'b1) ? INDW : IND0 }; // JMP (IDX,X)
         8'h20: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, (ABWDTH == 1'b1) ? JSRW : JSR0 }; // JSR ABS
         8'h40: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, RTI0 }; // RTI
         8'h60: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, RTS0 }; // RTS

         8'hAE: decode = { FLAG____, LDA, SRC__, DST_X, ALU_LDA , IZ, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // LDX ABS
         8'hBE: decode = { FLAG____, LDA, SRC__, DST_X, ALU_LDA , IY, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // LDX ABS,Y
         8'hA2: decode = { FLAG____, LDA, SRC__, DST_X, ALU_LDA , IZ, IMM0 }; // LDX #IMM
         8'hA6: decode = { FLAG____, LDA, SRC__, DST_X, ALU_LDA , IZ, ZPG0 }; // LDX ZP
         8'hB6: decode = { FLAG____, LDA, SRC__, DST_X, ALU_LDA , IY, ZPG0 }; // LDX ZP,Y
         8'hAC: decode = { FLAG____, LDA, SRC__, DST_Y, ALU_LDA , IZ, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // LDY ABS
         8'hBC: decode = { FLAG____, LDA, SRC__, DST_Y, ALU_LDA , IX, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // LDY ABS,X
         8'hA0: decode = { FLAG____, LDA, SRC__, DST_Y, ALU_LDA , IZ, IMM0 }; // LDY #IMM
         8'hA4: decode = { FLAG____, LDA, SRC__, DST_Y, ALU_LDA , IZ, ZPG0 }; // LDY ZP
         8'hB4: decode = { FLAG____, LDA, SRC__, DST_Y, ALU_LDA , IX, ZPG0 }; // LDY ZP,X

         8'h48: decode = { FLAG____, NOP, SRC_A, DST__, ALU_REG , IZ, PHA0 }; // PHA
         8'hDA: decode = { FLAG____, NOP, SRC_X, DST__, ALU_REG , IZ, PHA0 }; // PHX
         8'h5A: decode = { FLAG____, NOP, SRC_Y, DST__, ALU_REG , IZ, PHA0 }; // PHY
         8'h68: decode = { FLAG____, LDA, SRC__, DST_A, ALU_PLA , IZ, PLA0 }; // PLA
         8'hFA: decode = { FLAG____, LDA, SRC__, DST_X, ALU_PLA , IZ, PLA0 }; // PLX
         8'h7A: decode = { FLAG____, LDA, SRC__, DST_Y, ALU_PLA , IZ, PLA0 }; // PLY
         8'h08: decode = { FLAG____, NOP, SRC__, DST__, ALU_REG , IZ, PHA0 }; // PHP
         8'h28: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, PLA0 }; // PLP

         8'h8E: decode = { FLAG____, STA, SRC_X, DST__, ALU_REG , IZ, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // STX ABS
         8'h86: decode = { FLAG____, STA, SRC_X, DST__, ALU_REG , IZ, ZPG0 }; // STX ZP
         8'h96: decode = { FLAG____, STA, SRC_X, DST__, ALU_REG , IY, ZPG0 }; // STX ZP,Y
         8'h8C: decode = { FLAG____, STA, SRC_Y, DST__, ALU_REG , IZ, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // STY ABS
         8'h84: decode = { FLAG____, STA, SRC_Y, DST__, ALU_REG , IZ, ZPG0 }; // STY ZP
         8'h94: decode = { FLAG____, STA, SRC_Y, DST__, ALU_REG , IX, ZPG0 }; // STY ZP,X

         8'h9C: decode = { FLAG____, STA, SRC_Z, DST__, ALU_REG , IZ, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // STZ ABS
         8'h9E: decode = { FLAG____, STA, SRC_Z, DST__, ALU_REG , IX, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // STZ ABS,X
         8'h64: decode = { FLAG____, STA, SRC_Z, DST__, ALU_REG , IZ, ZPG0 }; // STZ ZP
         8'h74: decode = { FLAG____, STA, SRC_Z, DST__, ALU_REG , IX, ZPG0 }; // STZ ZP,X

         8'h0F: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, SYNC }; // CPU
         8'h1F: decode = { FLAG____, NOP, SRC__, DST__, ALU_____, IZ, SYNC }; // A24

         /* TBD */
         8'h1C: decode = { 19'b00_00_00_00_000000000_00, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // TRB ABS
         8'h14: decode = { 19'b00_00_00_00_000000000_00, ZPG0 }; // TRB ZP
         8'h0C: decode = { 19'b00_00_00_00_000000000_00, (ABWDTH == 1'b1) ? ABSW : ABS0 }; // TSB ABS
         8'h04: decode = { 19'b00_00_00_00_000000000_00, ZPG0 }; // TSB ZP
       default: decode = { 19'bxx_xx_xx_xx_xxxxxxxxx_xx, SYNC }; 
    endcase
//$display("DECODE W:%b %b", ABWDTH, decode);
end

/*
 *****************************************************************************
 * debug section
 *****************************************************************************
 */

`ifdef SIM

reg [39:0] statename;
always @*
    case( state )
        SYNC: statename = "SYNC";
        IMM0: statename = "IMM0";
        PHA0: statename = "PHA0";
        PLA0: statename = "PLA0";
        ZPG0: statename = "ZPG0";
        ZPW0: statename = "ZPW0";
        DATA: statename = "DATA";
        ABSW: statename = "ABSW";
        ABS0: statename = "ABS0";
        ABS1: statename = "ABS1";
        ABWW: statename = "ABWW";
        ABW0: statename = "ABW0";
        ABW1: statename = "ABW1";
        BRA0: statename = "BRA0";
        INDW: statename = "INDW";
        IND0: statename = "IND0";
        IND1: statename = "IND1";
        JMPW: statename = "JMPW";
        JMP0: statename = "JMP0";
        JMP1: statename = "JMP1";
        JSRW: statename = "JSRW";
        JSRH: statename = "JSRH";
        JSRL: statename = "JSRL";
        JSR0: statename = "JSR0";
        JSR1: statename = "JSR1";
        JSR2: statename = "JSR2";
        JSR3: statename = "JSR3";
        JSR4: statename = "JSR4";
        RTS0: statename = "RTS0";
        RTSW: statename = "RTSW";
        RTS1: statename = "RTS1";
        RTS2: statename = "RTS2";
        IDX0: statename = "IDX0";
        IDX1: statename = "IDX1";
        IDX2: statename = "IDX2";
        RMW0: statename = "RMW0";
        RMW1: statename = "RMW1";
        BRK0: statename = "BRK0";
        BRK1: statename = "BRK1";
        BRK2: statename = "BRK2";
        BRK3: statename = "BRK3";
        BRK4: statename = "BRK4";
        RTI0: statename = "RTI0";
     default: statename = "????";
    endcase

reg [7:0] opcode;
wire [23:0] mnemonic;

always @( posedge clk )
    if( sync )
        opcode <= IR;
/* 
 * disassembler translates binary opcode into 3 letter mnemonic
 */
disas disas( 
    .opcode(opcode),
    .mnemonic(mnemonic) );

integer cycle = 0;
always @( posedge clk )
    cycle <= cycle + 1;

wire [7:0] R_ = (RST === 1) ? "R" : "-";
wire [7:0] B_ = (B === 1) ? "B" : "-";
wire [7:0] C_ = (C === 1) ? "C" : "-";
wire [7:0] D_ = (D === 1) ? "D" : "-";
wire [7:0] I_ = (I === 1) ? "I" : "-";
wire [7:0] N_ = (N === 1) ? "N" : "-";
wire [7:0] V_ = (V === 1) ? "V" : "-";
wire [7:0] Z_ = (Z === 1) ? "Z" : "-";

wire [7:0] X = regfile.regs[SEL_X];
wire [7:0] Y = regfile.regs[SEL_Y];
wire [7:0] A = regfile.regs[SEL_A];

always @( posedge clk ) begin
    if( !debug || cycle < 100000 || cycle[10:0] == 0 )
      $display( "%4d %s %s %s PC:%h AB:%h W:%b DI:%h DR:%h D3:%h DO:%h IR:%h WE:%d ALU:%h S:%02x A:%h X:%h Y:%h P:%s%s%s%s%s%s", 
        cycle, R_, mnemonic, statename, PC, AB, ABWDTH, DI, DR, D3, DO, IR, WE, alu_out, S, A, X, Y, N_, V_, D_, I_, Z_, C_ );

      // end simulation on STP instruction
      if( opcode == 8'hdb ) begin
`ifdef ONEND
        onend = 1'b1;
`endif
        #1000 $finish( );
      end
end
`endif

endmodule
