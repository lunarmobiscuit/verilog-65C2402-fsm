/*
 * (C) Arlet Ottens <arlet@c-scape.nl>
 * Vector changes (C) Luni Libes
 */

module ram( 
    input clk,
`ifdef ONEND
    input onend,
`endif
    input [23:0] AB,
    input [7:0] DO,
    output [7:0] DI,
    input WE,
    input RDY);

`ifdef SIM

wire OE = !WE;

reg [7:0] mem_read;

reg [7:0] mem[0:131071];
reg [7:0] vec[0:15];

initial begin
    $readmemh( "ram.hex", mem );  // load RAM file
    $readmemh( "vec.hex", vec );  // load vector file ($FFFFF0-FFFFFF)

`ifdef ONEND
    memStart = { vec[8'h0C], vec[8'h0B], vec[8'h0A] };
    for (integer i = memStart - 16; i <= memStart; i = i + 16) begin
        $display("%3h: %1h %1h %1h %1h %1h %1h %1h %1h %1h %1h %1h %1h %1h %1h %1h %1h",
            i, mem[i+0], mem[i+1], mem[i+2], mem[i+3], mem[i+4], mem[i+5], mem[i+6], mem[i+7], mem[i+8],
            mem[i+9], mem[i+10], mem[i+11], mem[i+12], mem[i+13], mem[i+14], mem[i+15]);
    end
`endif
end

// CPU read
always @(posedge clk) begin
    if( RDY ) begin
        if (AB >= 24'hFFFFF0) begin
            mem_read <= vec[AB[3:0]];
            //$display("%h: VEC[%h] = %h", AB, AB[3:0], vec[AB[3:0]]);
        end
        else begin
            mem_read <= mem[AB];
            //$display("RAM[%h] = %h", AB, mem[AB]);
        end
    end
end

// CPU write
always @(posedge clk) begin
    if( WE & RDY ) begin
        if (AB < 24'hFFFFF0) mem[AB] <= DO;
        //$display("WRITE RAM[%h] = %h", AB, DO);
    end
end

assign DI = mem_read;

`ifdef ONEND
    // Dump memory
    integer memStart;
    always @(posedge onend) begin
        memStart = { vec[8'h0C], vec[8'h0B], vec[8'h0A] };
        for (integer i = memStart - 16; i <= memStart; i = i + 16) begin
            $display("%3h: %1h %1h %1h %1h %1h %1h %1h %1h %1h %1h %1h %1h %1h %1h %1h %1h",
                i, mem[i+0], mem[i+1], mem[i+2], mem[i+3], mem[i+4], mem[i+5], mem[i+6], mem[i+7], mem[i+8],
                mem[i+9], mem[i+10], mem[i+11], mem[i+12], mem[i+13], mem[i+14], mem[i+15]);
        end
    end
`endif

`endif

endmodule
