
// H264 - Core Transform

module h264coretransform
(
    input logic CLK,	// fast io clock
    input logic RESET,
    output logic READY = '0,		//set when ready for ENABLE
    input logic ENABLE,				//values input only when this is 1
    input logic [35:0] XXIN,	 //4 x 9bit, first px is lsbs
    output logic VALID = '0,				//values output only when this is 1
    output logic [13:0] YNOUT = '0	//output (zigzag order)
);



logic [1:0] yny, ynx;
logic [8:0]  xx0, xx1, xx2, xx3;


assign xx0 = XXIN[8:0];
assign xx1 = XXIN[17:9];
assign xx2 = XXIN[26:18];
assign xx3 = XXIN[35:27];

logic [9:0] xt0 = 10'd0;
logic [9:0] xt1 = 10'd0;
logic [9:0] xt2 = 10'd0;
logic [9:0] xt3 = 10'd0;

logic [11:0] ff00 = 12'd0;
logic [11:0] ff01 = 12'd0;
logic [11:0] ff02 = 12'd0;
logic [11:0] ff03 = 12'd0;
logic [11:0] ff10 = 12'd0;
logic [11:0] ff11 = 12'd0;
logic [11:0] ff12 = 12'd0;
logic [11:0] ff13 = 12'd0;
logic [11:0] ff20 = 12'd0;
logic [11:0] ff21 = 12'd0;
logic [11:0] ff22 = 12'd0;
logic [11:0] ff23 = 12'd0;
logic [11:0] ffx0 = 12'd0;
logic [11:0] ffx1 = 12'd0;
logic [11:0] ffx2 = 12'd0;
logic [11:0] ffx3 = 12'd0;
logic [11:0] ff0p = 12'd0;
logic [11:0] ff1p = 12'd0;
logic [11:0] ff2p = 12'd0;
logic [11:0] ff3p = 12'd0;
logic [11:0] ff0pu = 12'd0;
logic [11:0] ff1pu = 12'd0;
logic [11:0] ff2pu = 12'd0;
logic [11:0] ff3pu = 12'd0;
logic [12:0] yt0 = 13'd0;
logic [12:0] yt1 = 13'd0;
logic [12:0] yt2 = 13'd0;
logic [12:0] yt3 = 13'd0;
logic valid1 = 1'd0;
logic valid2 = 1'd0;

logic [2:0] ixx = 3'd0;
logic [3:0] iyn = 4'd0;

logic [3:0] ynyx = 4'd0;

logic [1:0] yny1 = 2'd0;
logic [1:0] yny2 = 2'd0;

localparam ROW0 = 2'b00;
localparam ROW1 = 2'b01;
localparam ROW2 = 2'b10;
localparam ROW3 = 2'b11;
localparam COL0 = 2'b00;
localparam COL1 = 2'b01;
localparam COL2 = 2'b10;
localparam COL3 = 2'b11;

assign yny = ynyx[3:2];
assign ynx = ynyx [1:0];

// New Parameters

// For FSM-1
logic [2:0] c_state, n_state;
parameter IDLE = 3'b000, S0=3'b001, S1=3'b010, S2=3'b011, S3=3'b100, S4=3'b101, S5= 3'b110, S6 = 3'b111;

//next_state always block
always_comb
begin
    // Next State Logic
    in = ENABLE || (ixx != 0); // Input signal to progress states

    case (c_state)
        IDLE: begin 
                if (in) 
                    n_state = S0;
            end
        S0: begin 
                if (in) 
                    n_state = S1;
            end
        S1: begin 
                if (in) 
                    n_state = S2;
            end
        S2: begin 
                if (in) 
                    n_state = S3;
            end
        S3: begin 
                if (in) 
                    n_state = S4;
            end
        S4: begin 
                if (in) 
                    n_state = S5;
            end
        S5: begin 
                if (in) 
                    n_state = S6;
            end
        S6: begin 
                if (in) 
                    n_state = IDLE;
            end
        default: n_state = c_state;
    endcase

    // Current State Logic
    case (c_state)
    IDLE: begin // Clear all signals
            Enable1 = 1'b0;
            Enable2 = 1'b0;
            Enable3 = 1'b0;
            Enable4 = 1'b0;
        end
    S0: begin // Set Enable1
            Enable1 = 1'b1;
            Enable2 = 1'b0;
            Enable3 = 1'b0;
            Enable4 = 1'b0;
        end
    S1: begin // Set Enable1, Enable2
            Enable1 = 1'b1;
            Enable2 = 1'b1;
            Enable3 = 1'b0;
            Enable4 = 1'b0;
        end
    S2: begin // Set Enable1, Enable3
            Enable1 = 1'b1;
            Enable2 = 1'b0;
            Enable3 = 1'b1;
            Enable4 = 1'b0;
        end
    S3: begin // Set Enable1, Enable4
            Enable1 = 1'b1;
            Enable2 = 1'b0;
            Enable3 = 1'b0;
            Enable4 = 1'b1;
        end
    S4: begin // Clear all signals
            Enable1 = 1'b0;
            Enable2 = 1'b0;
            Enable3 = 1'b0;
            Enable4 = 1'b0;
        end
    S5: begin // Clear all signals
            Enable1 = 1'b0;
            Enable2 = 1'b0;
            Enable3 = 1'b0;
            Enable4 = 1'b0;
        end
    S6: begin // Clear all signals
            Enable1 = 1'b0;
            Enable2 = 1'b0;
            Enable3 = 1'b0;
            Enable4 = 1'b0;
        end
    default: begin // Clear all signals
            Enable1 = 1'b0;
            Enable2 = 1'b0;
            Enable3 = 1'b0;
            Enable4 = 1'b0;
        end
    endcase // End of FSM-1

    // en_2 Signal Generator for pipeline # 02
    if ((ixx == 5) || (iyn != 0))
        en_2 = 1;
    else
        en_2 = 0;

    // Mux below pipeline # 02
    if (en_2)
        valid1_in = 1'b1;
    else
        valid1_in = 1'b0;

    case(iyn)
        4'd15 : ynyx = {ROW0, COL0};
        4'd14 : ynyx = {ROW0, COL1};
        4'd13 : ynyx = {ROW1, COL0};
        4'd12 : ynyx = {ROW2, COL0};
        4'd11 : ynyx = {ROW1, COL1};
        4'd10 : ynyx = {ROW0, COL2};
        4'd9  : ynyx = {ROW0, COL3};
        4'd8  : ynyx = {ROW1, COL2};
        4'd7  : ynyx = {ROW2, COL1};
        4'd6  : ynyx = {ROW3, COL0};
        4'd5  : ynyx = {ROW3, COL1};
        4'd4  : ynyx = {ROW2, COL2};
        4'd3  : ynyx = {ROW1, COL3};
        4'd2  : ynyx = {ROW2, COL3};
        4'd1  : ynyx = {ROW3, COL2};
        default : ynyx = {ROW3, COL3};
    endcase

    case(ynx)
        2'd0:
        begin
            ff0pu = ff00;
            ff1pu = ff10;
            ff2pu = ff20;
            ff3pu = ffx0;
        end

        2'd1:
        begin
            ff0pu = ff01;
            ff1pu = ff11;
            ff2pu = ff21;
            ff3pu = ffx1;
        end

        2'd2:
        begin
            ff0pu = ff02;
            ff1pu = ff12;
            ff2pu = ff22;
            ff3pu = ffx2;
        end

        default:
        begin
            ff0pu = ff03;
            ff1pu = ff13;
            ff2pu = ff23;
            ff3pu = ffx3;
        end
    endcase

    // MUX before Pipeline # 04
    //--compute final YNOUT values (14bit from 13bit)
    if (yny2==0) 
    begin
        YNOUT_IN <= {yt0[12], yt0} + {yt1[12], yt1};	//-- yt0 + yt1
    end
    else if (yny2==1) 
    begin
        YNOUT_IN <= {yt2[12], yt2} + {yt3, 1'b0};		//-- yt2 + 2*yt3
    end
    else if (yny2==2) 
    begin
        YNOUT_IN <= {yt0[12], yt0} - {yt1[12], yt1};    //-- yt0 - yt1
    end	   
    else
    begin
        YNOUT_IN <= {yt3[12], yt3} - {yt2, 1'b0};	    //-- yt3 - 2*yt2
    end 
end


always_ff @(posedge CLK)
begin
    // Reset Signal
    if (reset)
    begin
        ixx <= 0;   // Reset counter
    end
    
    // Counter for ixx
    if (ENABLE || (ixx != 0))
    begin
        ixx <= ixx + 1;
    end 

    // READY Signal Generator
    if (ixx < 3 && (iyn >= 14 || iyn==0))
    begin
        READY <= 1;
    end
    else
    begin
        READY <= 0;
    end

    // FSM-1 : Moore Machine with 8 States; IDLE,S0,S1,S2,S3,S4,S5,S6
    //state register
    //reset is active low
    if (RESET)
        state <= IDLE;
    else
        state <= n_state;
    // Rest of the FSM-1 is in always_comb block

    // Pipeline Register # 1
    if (ENABLE)
    begin
        // --initial helpers (TT+1) (10bit from 9bit)
        xt0 <= {xx0[8], xx0} + {xx3[8], xx3};			//--xx0 + xx3
        xt1 <= {xx1[8], xx1} + {xx2[8], xx2};			//--xx1 + xx2
        xt2 <= {xx1[8], xx1} - {xx2[8], xx2};			//--xx1 - xx2
        xt3 <= {xx0[8], xx0} - {xx3[8], xx3};			//--xx0 - xx3
    end

    // Registers powered by FSM-1
    if (Enable1)
    begin
        // --now compute row of FF matrix at TT+2 (12bit from 10bit)
        ffx0 <= {xt0[9], xt0[9], xt0} + {xt1[9], xt1[9], xt1};	    //--xt0 + xt1
        ffx1 <= {xt2[9], xt2[9], xt2} + {xt3[9], xt3, 1'b0};	 	//--xt2 + 2*xt3
        ffx2 <= {xt0[9], xt0[9], xt0} - {xt1[9], xt1[9], xt1};	    //--xt0 - xt1
        ffx3 <= {xt3[9], xt3[9], xt3} - {xt2[9], xt2, 1'b0};		//--xt3 - 2*xt2
    end

    //--place rows 0,1,2 into slots at TT+3,4,5
    if (Enable2) 
    begin
        ff00 <= ffx0;
        ff01 <= ffx1;
        ff02 <= ffx2;
        ff03 <= ffx3;
    end

    if (Enable3)
    begin
        ff10 <= ffx0;
        ff11 <= ffx1;
        ff12 <= ffx2;
        ff13 <= ffx3;
    end 

    if (Enable4) 
    begin
        ff20 <= ffx0;
        ff21 <= ffx1;
        ff22 <= ffx2;
        ff23 <= ffx3;
    end

    // Pipeline #02
    // Signals used in Pipeline #02 are generated above and in always_comb block

    if (en_2)
    begin
        ff0p <= ff0pu;
        ff1p <= ff1pu;
        ff2p <= ff2pu;
        ff3p <= ff3pu;
        yny1 <= yny;
        iyn  <= iyn + 1;
    end

    // valid1 Register below Pipeline # 02
    // CLK dependent
    valid1 <= valid1_in;

    // Pipeline Register # 03
    if (valid1) 
    begin
        yt0 <= {ff0p[11], ff0p} + {ff3p[11], ff3p};	    //--ff0 + ff3
        yt1 <= {ff1p[11], ff1p} + {ff2p[11], ff2p};	    //--ff1 + ff2
        yt2 <= {ff1p[11], ff1p} - {ff2p[11], ff2p};	    //--ff1 - ff2
        yt3 <= {ff0p[11], ff0p} - {ff3p[11], ff3p};	    //--ff0 - ff3
        yny2 <= yny1;
    end

    // Valid2
    valid2 <= valid1

    // Pipeline # 04 (ENDING)
    // OUTPUT "YNOUT_IN"is computed at the end of always_comb block

    if (valid2)
    begin
        YNOUT <= YNOUT_IN;
    end

    // VALID
    VALID <= valid2

end
endmodule

