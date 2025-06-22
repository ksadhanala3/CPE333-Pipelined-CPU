`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: California Polytechnic University, San Luis Obispo
// Engineer: Diego Renato Curiel
// Create Date: 02/25/2023 10:55:14 PM
// Module Name: TwoMux
//////////////////////////////////////////////////////////////////////////////////

module TwoMux(
    input logic SEL,
    input logic [31:0] ONE,
    input logic [31:0] TWO,
    output logic [31:0] OUT
    );
    
    //Create a generic two-to-one MUX to be used for the ALU.
    always_comb begin
        case(SEL)
            1'b0: begin OUT = ONE; end
            1'b1: begin OUT = TWO; end
        endcase
    end
    
endmodule
