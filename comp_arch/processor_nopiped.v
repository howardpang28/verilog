// This code was used for a course at the University of Waterloo.
// This is the non-pipelined implementation of a 5 stage pipelined processor.
// The processor supports an ISA with: add, addi, sub, subi, mov, movi, beq, blt, 
//
// name: Howard Pang
// compiler: iverilog
// pipelined?: no
module processor(mem_wr_data, mem_wr_addr, mem_wr_en, mem_rd_data1, mem_rd_addr1,
	mem_rd_data2, mem_rd_addr2, reset, clk);
	parameter n=8;
	output reg [n-1:0] mem_wr_data;
	output reg [n-1:0] mem_wr_addr;
	output reg mem_wr_en;
	input [n-1:0] mem_rd_data1;
	output [n-1:0] mem_rd_addr1;
	input [n-1:0] mem_rd_data2;
	output [n-1:0] mem_rd_addr2;
	input reset;
	input clk;

	// debug purposes
	wire [n-1:0] REG0, REG1, REG2, REG3, OPR0, OPR1;
	wire [2:0] instr;
	assign 	REG0 = REG[0], REG1 = REG[1],
		REG2 = REG[2], REG3 = REG[3],
		OPR0 = OPR[0], OPR1 = OPR[1],
		instr = IR[7:5];

	// states
	parameter [2:0]
		st_if  = 3'b000,
		st_id  = 3'b001,
		st_ex  = 3'b010,
		st_me  = 3'b011,
		st_wb  = 3'b100;

	reg[2:0] state;

	// processor regs
	reg[n-1:0] PC; // program counter
	reg[n-1:0] IR; // instruction register
	reg[n-1:0] REG [3:0]; // general-purpose registers r0-r3
	reg[n-1:0] OPR [1:0]; // operand regs to EX functional units

	assign mem_rd_addr1 = PC; // read port 1 is for instructions
	assign mem_rd_addr2 = REG[IR[ir_reg0_hi:ir_reg0_lo]]; // for indirect addressing

	// useful for indexing in IR - modify as needed...
	parameter ir_opcode_hi = 7;
	parameter ir_opcode_lo = 5;
	parameter ir_reg1_hi = 4;
	parameter ir_reg1_lo = 3;
	parameter ir_reg0_hi = 2;
	parameter ir_reg0_lo = 1;
	parameter ir_dst = 0;

	wire[2:0] op;
	assign op = IR[ir_opcode_hi:ir_opcode_lo];

	// ISA
	parameter [2:0]
		add  = 3'b000,
		addi = 3'b001,
		sub  = 3'b010,
		subi = 3'b011,
		mov  = 3'b100,
		movi = 3'b101,
		beq  = 3'b110,
		blt = 3'b111;

	always@(posedge clk or posedge reset) begin // stages
		if(reset) begin
			state <= st_if;
			PC <= 8'h 00;
		end
		else
		case(state)
			st_if: begin
				IR <= mem_rd_data1;
				PC <= PC + 1;
				state <= st_id;
			end

			st_id: begin
				// read in R1 and R2
				if (op == add || op == sub || op == beq || op == blt) begin
					OPR[1] <= REG[IR[4:3]];
					if (IR[2:1] == 2'b00) 
						begin OPR[0] <= mem_rd_data2; end else
						begin OPR[0] <= REG[IR[2:1]]; end

				// read R1
				end else if (op == addi || op == subi) begin
					OPR[1] <= REG[IR[4:3]];
					OPR[0] <= OPR[0];

				// read R2
				end else if (op == mov) begin
					OPR[1] <= OPR[1];
					if (IR[0]) begin
						if (IR[2:1] == 2'b00)
							begin OPR[0] <= mem_rd_data2; end else
							begin OPR[0] <= REG[IR[2:1]]; end
					end else begin
						OPR[0] <= REG[IR[4:3]];
					end
				end
				state <= st_ex;
			end

			st_ex: begin
				case(op)
				add:  begin OPR[1] <= OPR[1] + OPR[0]; 		end
				sub:  begin OPR[1] <= OPR[1] - OPR[0]; 		end
				addi: begin OPR[1] <= OPR[1] + mem_rd_data1; 	end
				subi: begin OPR[1] <= OPR[1] - mem_rd_data1; 	end
				mov:  begin OPR[1] <= OPR[0]; 			end
				movi: begin OPR[1] <= mem_rd_data1; 		end
				default: begin 					end
				endcase


				if( op == addi || op == subi || op == movi) begin // immediate
					PC <= PC + 1;
				end else if (op == beq) begin // branch
					if (OPR[1] == OPR[0]) begin 
						if (mem_rd_data1[7])
							begin PC <= PC + 1 - ~(mem_rd_data1-1);	end else
							begin PC <= PC + 1 + mem_rd_data1;	end
					end else begin
						PC <= PC + 1;
					end
				end else if (op == blt) begin // branch
					if (OPR[1] < OPR[0]) begin 
						if (mem_rd_data1[7])
							begin PC <= PC + 1 - ~(mem_rd_data1-1); end else
							begin PC <= PC + 1 + mem_rd_data1;	end
					end else begin
						PC <= PC + 1;
					end

				end
				state <= st_me;
			end

			st_me: begin
				if (IR[2:0] == 3'b000) begin
					mem_wr_addr <= REG[0];
					mem_wr_data <= OPR[1];
					mem_wr_en <= 1'b1;
				end
				state <= st_wb;
			end

			st_wb: begin
				if (IR[2:0] == 3'b000 || IR[7:6] == 2'b11) begin
				end else begin 
					if (IR[0] == 1'b0)
						begin REG[IR[2:1]] <= OPR[1]; end else
						begin REG[IR[4:3]] <= OPR[1]; end
				end
				mem_wr_en <= 0;
				state <= st_if;
			end

			default: state <= st_if;
		endcase
	end
endmodule
