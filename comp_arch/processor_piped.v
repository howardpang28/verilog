// This code was used for a course at the University of Waterloo.
// This is the pipelined implementation of a 5 stage pipelined processor.
// The processor supports an ISA with: add, addi, sub, subi, mov, movi, beq, blt, 
//
// name: Howard Pang
// compiler: iverilog
// pipelined?: yes
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
	assign 	REG0 = REG[0], REG1 = REG[1],
		REG2 = REG[2], REG3 = REG[3],
		OPR0 = OPR[0], OPR1 = OPR[1];
	wire [n-1:0] IR0, IR1, IR2, IR3;
	assign IR0=IR[0],IR1=IR[1],IR2=IR[2],IR3=IR[3];

	// states
	parameter [2:0]
		st_if  = 3'b000,
		st_id  = 3'b001,
		st_ex  = 3'b010,
		st_me  = 3'b011,
		st_wb  = 3'b100;

	reg[2:0] state;
	reg [n-1:0] mem_rd_addr2;
	
	// processor regs
	reg[n-1:0] PC; // program counter
	reg[n-1:0] IR [3:0]; // instruction register
	reg[n-1:0] REG [3:0]; // general-purpose registers r0-r3
	reg[n-1:0] OPR [1:0]; // operand regs to EX functional units

	assign mem_rd_addr1 = PC; // read port 1 is for instructions

	// useful for indexing in IR - modify as needed...
	parameter ir_opcode_hi = 7;
	parameter ir_opcode_lo = 5;
	parameter ir_reg1_hi = 4;
	parameter ir_reg1_lo = 3;
	parameter ir_reg0_hi = 2;
	parameter ir_reg0_lo = 1;
	parameter ir_dst = 0;

	//wire[2:0] op;
	//assign op = IR[ir_opcode_hi:ir_opcode_lo];

	// ISA
	parameter [2:0]
		add  = 3'b000,
		addi = 3'b001,
		sub  = 3'b010,
		subi = 3'b011,
		mov  = 3'b100,
		movi = 3'b101,
		beq  = 3'b110,
		blt  = 3'b111;
	parameter [n-1:0] nop = 8'h xx;

	wire wr_ops;
	assign wr_ops = (IR[0] !== nop && stall !== 1'b1);

	reg branch, stall; // branch?
	reg [n-1:0] br_off; // branch offset
	reg [n-1:0] id_r1, id_r2, ex_r1, me_r1, imm_val;
	wire wr, imm0, imm1, imm2, read2, read1, read0;

	// combinational assignments
	assign imm0 = (IR[0][7:5] == addi || IR[0][7:5] == subi || IR[0][7:5] == movi || IR[0][7:5] == blt || IR[0][7:5] == beq);
	assign imm1 = (IR[1][7:5] == addi || IR[1][7:5] == subi || IR[1][7:5] == movi || IR[1][7:5] == blt || IR[1][7:5] == beq);
	assign imm2 = (IR[2][7:5] == addi || IR[2][7:5] == subi || IR[2][7:5] == movi || IR[2][7:5] == blt || IR[2][7:5] == beq);
	assign imm3 = (IR[3][7:5] == addi || IR[3][7:5] == subi || IR[3][7:5] == movi || IR[3][7:5] == blt || IR[3][7:5] == beq);
	assign read2 = (IR[0][7:5] == add || IR[0][7:5] == sub || IR[0][7:5] == beq || IR[0][7:5] == blt);
	assign read1 = (IR[0][7:5] == addi || IR[0][7:5] == subi || (IR[0][7:5] == mov && ~IR[0][0]));
	assign read0 = (IR[0][7:5] == mov && IR[0][0]);
	
	// gets the destination of the IR and stall logic
	reg [1:0] wd_ex, wd_wb, wd_me; 
	wire df1_me, df2_me, df1_wb, df2_wb;
	assign wr  = ~(IR[0][7:5] == blt || IR[0][7:5] == beq);
	assign df1_me = (IR[0][2:1] == wd_me && IR[2][2:0] !== 3'b000);
	assign df2_me = (IR[0][4:3] == wd_me && IR[2][2:0] !== 3'b000);
	assign df1_wb = (IR[0][2:1] == wd_wb && IR[3][2:0] !== 3'b000);
	assign df2_wb = (IR[0][4:3] == wd_wb && IR[3][2:0] !== 3'b000); // the 2nd check ensures that indirect addressing cannot be forwarded

	// stall logic
	always@(*) begin
		if (read2) begin // the 2nd prevents stalling due to fake dependencies with ind. addressing
			if ((IR[0][2:1] == wd_ex || IR[0][4:3] == wd_ex) && IR[1][2:0] != 3'b000) stall <= 1'b1;
			else stall <= 1'b0;
		end else if (read1) begin
			if (IR[0][4:3] == wd_ex && IR[1][2:0] != 3'b000) stall <= 1'b1;
			else stall <= 1'b0;
		end else if (read0) begin
			if (IR[0][2:1] == wd_ex && IR[1][2:0] != 3'b000) stall <= 1'b1;
			else stall <= 1'b0;
		end else
			stall <= 1'b0;			
	end

	// data forward logic
	always@(*) begin
		if (IR[1][0] || imm1) wd_ex <= IR[1][4:3];
		else	              wd_ex <= IR[1][2:1];
		if (IR[2][0] || imm2) wd_me <= IR[2][4:3];
		else	              wd_me <= IR[2][2:1];
		if (IR[3][0] || imm3) wd_wb <= IR[3][4:3];
		else	              wd_wb <= IR[3][2:1];
	end

	// indirect addressing logic
	always@(*) begin
		if (IR[0][2:0] == 3'b001) begin
			if (df1_me) 	 mem_rd_addr2 <= ex_r1;
			else if (df1_wb) mem_rd_addr2 <= me_r1;
			else		 mem_rd_addr2 <= REG[IR[0][2:1]];
		end
	end

	// IF functional unit
	always@(posedge clk or posedge reset) begin
		if(reset) begin
			PC    <= 8'h 00;
			IR[0] <= 8'h xx;
		end else if(stall !== 1'b1) begin
			if(branch) begin 
				if (br_off[7]) PC <= PC - 1 - ~(br_off - 1);
				else	       PC <= PC - 1 + br_off;
			end else begin
				PC <= PC + 1;
			end

			if(imm0 || branch) IR[0] <= nop;
			else	 	   IR[0] <= mem_rd_data1;
		end
	end

	// ID functional unit
	always@(posedge clk or posedge reset) begin
		if(reset) begin
			IR[1] <= 8'hxx;
			id_r1 <= 8'h00;
			id_r2 <= 8'h00;
		end else begin
			if(IR[0] !== nop && stall !== 1'b1) begin
				if (IR[0][2:0] == 3'b001) begin
					id_r1 <= mem_rd_data2;
				end else begin
					if (df1_me) 	 id_r1 <= ex_r1;
					else if (df1_wb) id_r1 <= me_r1;
					else		 id_r1 <= REG[IR[0][2:1]];
				end
				if (df2_me) 	 id_r2 <= ex_r1;
				else if (df2_wb) id_r2 <= me_r1;
				else		 id_r2 <= REG[IR[0][4:3]];
			end

			if (imm0) imm_val <= mem_rd_data1;

			if (stall || branch) IR[1] <= nop;
			else 		     IR[1] <= IR[0];
		end
	end

	// EX functional unit
	always@(posedge clk or posedge reset) begin
		if (reset) begin
			IR[2] <= 8'hxx;
			ex_r1 <= 8'h00;
		end else begin
			// alu
			case(IR[1][7:5])
			add:  begin ex_r1 <= id_r2 + id_r1;	end
			sub:  begin ex_r1 <= id_r2 - id_r1;	end
			addi: begin ex_r1 <= id_r2 + imm_val;	end
			subi: begin ex_r1 <= id_r2 - imm_val;	end
			movi: begin ex_r1 <= imm_val;		end
			mov:  begin 
				if (IR[1][0]) ex_r1 <= id_r1;
				else 	      ex_r1 <= id_r2;
			end
			default: begin 				end
			endcase


			// branch resolution
			case(IR[1][7:5])
			beq: begin
				if (id_r2 == id_r1) begin
					branch <= 1'b1;
					br_off <= imm_val;
				end
			end
			blt: begin
				if (id_r2 < id_r1) begin
					branch <= 1'b1;
					br_off <= imm_val;
				end
			end
			default: begin branch <= 1'b0; end
			endcase

			if (branch) IR[2] <= nop;
			else 	    IR[2] <= IR[1];
		end
	end

	// ME functional unit
	always@(posedge clk or posedge reset) begin
		if(reset) begin
			IR[3] <= 8'hxx;
			me_r1  <= 8'h00;
			mem_wr_addr <= 8'h00;
			mem_wr_data <= 8'h00;
			mem_wr_en   <= 1'b0;
		end else begin
			if (IR[2][2:0] == 3'b000 && IR[2] !== nop) begin
				mem_wr_addr <= REG[0];
				mem_wr_data <= ex_r1;
				mem_wr_en <= 1'b1;
			end else
				mem_wr_en <= 1'b0;

			me_r1 <= ex_r1;
			IR[3] <= IR[2];
		end
	end

	// WB functional unit
	always@(posedge clk) begin
		if (IR[3][2:0] !== 3'b000 && IR[3][7:6] != 2'b11 && IR[3] !== nop) 
			REG[wd_wb] <= me_r1;
	end

endmodule
