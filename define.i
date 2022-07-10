parameter
    SYNC  = 6'd0,
    IMM0  = 6'd1,
    PHA0  = 6'd2,
    PLA0  = 6'd3,
    ZPG0  = 6'd4,
    DATA  = 6'd5,
    ABS0  = 6'd6,
    ABS1  = 6'd7,
    BRA0  = 6'd8,
    JSR0  = 6'd9,
    JSR1  = 6'd10,
    JSR2  = 6'd11,
    RTS0  = 6'd12,
    RTS1  = 6'd13,
    RTS2  = 6'd14,
    JMP0  = 6'd15,
    JMP1  = 6'd16,
    IDX0  = 6'd17,
    IDX1  = 6'd18,
    IDX2  = 6'd19,
    BRK0  = 6'd20,
    BRK1  = 6'd21,
    BRK2  = 6'd22,
    BRK3  = 6'd23,
    RTI0  = 6'd24,
    IND0  = 6'd25,
    IND1  = 6'd26,
    ZPW0  = 6'd27,
    ABW0  = 6'd28,
    ABW1  = 6'd29,
    RMW0  = 6'd30,
    RMW1  = 6'd31,
    BRK4  = 6'd32,
    JMPW  = 6'd33,
    INDW  = 6'd34,
    ABSW  = 6'd35,
    ABWW  = 6'd36,
    JSRW  = 6'd37,
    JSRH  = 6'd38,
    JSRL  = 6'd39,
    JSR3  = 6'd40,
    JSR4  = 6'd41,
    RTSW  = 6'd42;

parameter
    NOP   = 2'b00,
    LDA   = 2'b01,
    STA   = 2'b10,
    RMW   = 2'b11;

parameter
    DST__ = 2'bxx,
    DST_X = 2'b01,
    DST_Y = 2'b10,
    DST_A = 2'b11;

parameter
    SRC__ = 2'bxx,
    SRC_Z = 2'b00,
    SRC_X = 2'b01,
    SRC_Y = 2'b10,
    SRC_A = 2'b11;

parameter
    SEL_Z = 2'b00,
    SEL_X = 2'b01,
    SEL_Y = 2'b10,
    SEL_A = 2'b11;

/*
 * flag behavior
 */
parameter
    FLAG____ = 2'b00,
    FLAG_ADD = 2'b01,
    FLAG_CMP = 2'b10,
    FLAG_BIT = 2'b11;

/* 
 * index register bits from control[] vector
 */
parameter
    IZ = SEL_Z,
    IX = SEL_X,
    IY = SEL_Y;

/* 
 * index register mask from cycle operation
 */
parameter
    IDX___ = 2'b00,     // force zero register
    IDX_X_ = 2'b01,     // only allow X in this cycle
    IDX__Y = 2'b10,     // only allow Y in this cycle
    IDX_XY = 2'b11;     // allow both X/Y register
        
parameter //      SR__A__B__C      SR   A     B   C
          // -----------------------------------------
    ALU_____ = 9'bxx_xxx_xx_xx,
    ALU_REG  = 9'b00_000_00_00, //      R  +  0 + 0
    ALU_INCA = 9'b00_000_00_01, //      R  +  0 + 1
    ALU_DECA = 9'b00_000_10_00, //      R  + -1 + 0
    ALU_LDA  = 9'b00_001_00_00, //      M  +  0 + 0
    ALU_INCM = 9'b00_001_00_01, //      M  +  0 + 1
    ALU_DECM = 9'b00_001_10_00, //      M  + -1 + 0
    ALU_ADC  = 9'b00_000_01_10, //      R  +  M + C
    ALU_CMP  = 9'b00_000_11_01, //      R  + ~M + 1
    ALU_SBC  = 9'b00_000_11_10, //      R  + ~M + C
    ALU_ORA  = 9'b00_100_00_00, //     R|M +  0 + 0
    ALU_AND  = 9'b00_101_00_00, //     R&M +  0 + 0
    ALU_EOR  = 9'b00_110_00_00, //     R^M +  0 + 0
    ALU_TSX  = 9'b00_111_00_00, //      S  +  0 + 0
    ALU_PLA  = 9'b01_xxx_xx_xx, //         DI   
    ALU_ASLA = 9'b10_000_00_00, // asl( R  +  0 + 0)
    ALU_ROLA = 9'b10_000_00_11, // rol( R  +  0 + 0)
    ALU_LSRA = 9'b11_000_00_00, // lsr( R  +  0 + 0)
    ALU_RORA = 9'b11_000_00_11, // ror( R  +  0 + 0)
    ALU_ASLM = 9'b10_001_00_00, // asl( M  +  0 + 0)
    ALU_ROLM = 9'b10_001_00_11, // rol( M  +  0 + 0)
    ALU_LSRM = 9'b11_001_00_00, // lsr( M  +  0 + 0)
    ALU_RORM = 9'b11_001_00_11; // ror( M  +  0 + 0)

parameter
    //            +H h PC BS +X +1
    AB_ZPGX = 10'b00_1_00_00_11_0,  // AB=DI+XY                     HOLD
    AB_ABSX = 10'b10_1_00_10_01_0,  // AB=DIDR{D3}+XY               HOLD
    AB_PUSH = 10'b01_0_00_00_00_0,  // AB=S
    AB_PULL = 10'b01_0_00_00_00_1,  // AB=S+1
    AB_HOLD = 10'b10_0_00_11_00_0,  // AB=HOLD
    AB_NEXT = 10'b10_0_00_11_00_1,  // AB=HOLD+1    
    AB_OPER = 10'b00_0_01_01_00_0,  // AB=PC,           PC = AB+1
    AB_FWRD = 10'b10_0_01_01_10_0,  // AB=PC + DI       PC = AB+1
    AB_JMP1 = 10'b10_0_01_10_00_1,  // AB=DIDR{D3}+1,   PC = AB+1
    AB_JMP0 = 10'b10_0_01_10_01_0,  // AB=DIDR{D3}+XY,  PC = AB+1
    AB_BACK = 10'b11_0_01_01_10_0,  // AB=PC + FF,DI    PC = AB+1
    AB_NMIV = 10'b01_0_10_00_00_0,  // AB=S             PC = NMI Vector
    AB_IRQV = 10'b01_0_11_00_00_0;  // AB=S             PC = IRQ Vector

parameter
    WE___ = 2'b00,                  // read from this memory location
    WE_1_ = 2'b01,                  // write to this memory location
    WE_ST = 2'b10;                  // write only if opcode requires it

parameter
    DR___ = 1'b0,                   // do not load DR, keep old value
    DR_DI = 1'b1;                   // load DR with DI

parameter
    DO____ = 3'bxxx,
    DO_ALU = 3'b000,
    DO_PCL = 3'b001,
    DO_PCH = 3'b010,
    DO_PHP = 3'b011,
    DO_PC3 = 3'b100;

parameter
    S____  = 2'b00,
    S_INC  = 2'b01,
    S_DEC  = 2'b10;

parameter
    AB16  = 2'b00,
    AB24  = 2'b01,
    AB32  = 2'b10,
    AB48  = 2'b11;

parameter
    R08  = 2'b00,
    R16  = 2'b01,
    R24  = 2'b10,
    R32  = 2'b11;
