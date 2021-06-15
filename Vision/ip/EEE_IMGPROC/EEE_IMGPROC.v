module EEE_IMGPROC(
	// global clock & reset
	clk,
	reset_n,
	
	// mm slave
	s_chipselect,
	s_read,
	s_write,
	s_readdata,
	s_writedata,
	s_address,

	// stream sink
	sink_data,
	sink_valid,
	sink_ready,
	sink_sop,
	sink_eop,
	
	// streaming source
	source_data,
	source_valid,
	source_ready,
	source_sop,
	source_eop,
	
	// conduit
	mode
	
);


// global clock & reset
input	clk;
input	reset_n;

// mm slave
input							s_chipselect;
input							s_read;
input							s_write;
output	reg	[31:0]	s_readdata;
input	[31:0]				s_writedata;
input	[2:0]					s_address;


// streaming sink
input	[23:0]            	sink_data;
input								sink_valid;
output							sink_ready;
input								sink_sop;
input								sink_eop;

// streaming source
output	[23:0]			  	   source_data;
output								source_valid;
input									source_ready;
output								source_sop;
output								source_eop;

// conduit export
input                         mode;

////////////////////////////////////////////////////////////////////////
//
parameter IMAGE_W = 11'd640;
parameter IMAGE_H = 11'd480;
parameter MESSAGE_BUF_MAX = 256;
parameter MSG_INTERVAL = 10;
parameter BB_COL_DEFAULT = 24'h00ff00;


wire [7:0]   red, green, blue, grey;
wire [7:0]   red_out, green_out, blue_out;

wire         sop, eop, in_valid, out_ready;
////////////////////////////////////////////////////////////////////////

// Detect red areas
//wire red_detect;
//assign red_detect = red[7] & ~green[7] & ~blue[7];

// Find boundary of cursor box

// Highlight detected areas
//wire [23:0] red_high;
assign grey = green[7:1] + red[7:2] + blue[7:2]; //Grey = green/2 + red/4 + blue/4
//assign red_high  =  red_detect ? {8'hff, 8'h0, 8'h0} : {grey, grey, grey};

// Show bounding box
wire [23:0] new_image;
//wire bb_active;
//assign bb_active = (x == left) | (x == right) | (y == top) | (y == bottom);
//assign new_image = bb_active ? bb_col : red_high;

// Switch output pixels depending on mode switch
// Don't modify the start-of-packet word - it's a packet discriptor
// Don't modify data in non-video packets
assign {red_out, green_out, blue_out} = (mode & ~sop & packet_video) ? new_image : {red,green,blue};

//Method 1: Edge Detection
//Count valid pixels to tget the image coordinates. Reset and detect packet type on Start of Packet.
//reg [5119:0] buffer_0, buffer_1, buffer_2;
//reg [1:0] current_row;
//wire x_index, l_index, r_index;
//assign x_index = x<<3;
//assign l_index = (x-1)<<3;
//assign r_index = (x+1)<<3;
//
//always @(posedge clk) begin
//	if(current_row == 0) begin
//		buffer_0[x_index+:8] <= grey;
//	end else if(current_row == 1) begin
//		buffer_1[x_index+:8] <= grey;
//	end else begin
//		buffer_2[x_index+:8] <= grey;	
//	end
//end
//
//wire [15:0] intensity;
//assign intensity = (current_row == 0) ? (buffer_2[r_index+:8] - buffer_2[l_index+:8])**2 + (grey - buffer_1[x_index+:8])**2:
//                         (current_row == 1) ? (buffer_0[r_index+:8] - buffer_0[l_index+:8])**2 + (grey - buffer_2[x_index+:8])**2:
//                         (buffer_1[r_index+:8] - buffer_1[l_index+:8])**2 + (grey - buffer_0[x_index+:8])**2;
//wire [7:0] edge_detect;
//assign edge_detect = intensity[7:0];
//
//assign new_image = 
//(x > 2 && x < IMAGE_W-1 && y > 2 && y < IMAGE_H-1) ? {edge_detect, edge_detect, edge_detect}:
//24'hff00ff;

//Method 2: RGB to HSV
//wire [15:0] r_prime, g_prime, b_prime;
//assign r_prime = red/255;
//assign g_prime = green/255;
//assign b_prime = blue/255;
//
//wire [15:0] max, min;
//assign max = ((r_prime > g_prime) && (g_prime > b_prime)) ? r_prime :
//				((g_prime > r_prime) && (g_prime > b_prime)) ? g_prime :
//				r_prime;
//
//wire [15:0] light;
//assign light = (max+min)/2;
//
//wire [15:0] saturation;
//assign saturation = (max == min) ? 0 : 
//						(2*light <= 1'b1) ? (max-min)/(max+min) :
//						(max-min)/(2-max-min);
//						
//reg [15:0] hue;
//always@(*) begin
//	hue <= ((r_prime > g_prime) && (g_prime > b_prime)) ? (g_prime - b_prime)/(max-min) :
//				((g_prime > r_prime) && (g_prime > b_prime)) ? 2+(b_prime - r_prime)/(max-min) :
//				4+(r_prime - g_prime)/(max-min);
//				
//	hue <= hue*60;
//	
//	if(hue < 0) begin
//		hue <= hue + 360;
//	end
//end
//
//wire red_detect, yellow_detect, blue_detect, pink_detect, teal_detect;
//assign red_detect =  (hue < 10 || hue > 350);
//assign yellow_detect = (hue > 40 && hue < 70);
//assign blue_detect = (hue > 220 && hue < 250);
//assign pink_detect = (hue > 290 && hue < 330);
//assign teal_detect = (hue > 130 && hue < 180);
//
//wire red_bb_active, yellow_bb_active, blue_bb_active, pink_bb_active, teal_bb_active;
//assign red_bb_active = (x == red_left) | (x == red_right) | (y == red_top) | (y == red_bottom);
//assign yellow_bb_active = (x == yellow_left) | (x == yellow_right) | (y == yellow_top) | (y == yellow_bottom);
//assign blue_bb_active = (x == blue_left) | (x == blue_right) | (y == blue_top) | (y == blue_bottom);
//assign pink_bb_active = (x == pink_left) | (x == pink_right) | (y == pink_top) | (y == pink_bottom);
//assign teal_bb_active = (x == teal_left) | (x == teal_right) | (y == teal_top) | (y == teal_bottom);
//
//assign new_image = 	red_bb_active ? 24'hff0000:
//							yellow_bb_active ? 24'hffff00:
//							blue_bb_active ? 24'h0000ff:
//							pink_bb_active ? 24'hff00ff:
//							teal_bb_active ? 24'h008080:
//							red_detect ? 24'hff0000:
//							yellow_detect ? 24'hffff00:
//							blue_detect ? 24'h0000ff:
//							pink_detect ? 24'hff00ff:
//							teal_detect ? 24'h008080:
//							{grey, grey, grey};
							
							
//Method 3: Blur and colour detect
//reg [15359:0] buffer_0, buffer_1, buffer_2;
//reg [1:0] current_row;
//wire x_index, l_index, c_index;
//assign x_index = (x<<3)*3;
//assign c_index = ((x-1)<<3)*3;
//assign l_index = ((x-2)<<3)*3;
//
//always @(posedge clk) begin
//	if(current_row == 0) begin
//		buffer_0[x_index+:24] <= {red,green,blue};
//	end else if(current_row == 1) begin
//		buffer_1[x_index+:24] <= {red,green,blue};
//	end else begin
//		buffer_2[x_index+:24] <= {red,green,blue};	
//	end
//end
//
//wire [23:0] blur;
//assign blur = (current_row == 0) ? ((buffer_1[l_index+:24] + buffer_1[c_index+:24] + buffer_1[x_index+:24] + buffer_2[l_index+:24] + buffer_2[c_index+:24] + buffer_2[x_index+:24] + buffer_0[l_index+:24] + buffer_0[c_index+:24] + {red, green, blue})/9) :
//(current_row == 1) ? ((buffer_1[l_index+:24] + buffer_1[c_index+:24] + {red, green, blue} + buffer_2[l_index+:24] + buffer_2[c_index+:24] + buffer_2[x_index+:24] + buffer_0[l_index+:24] + buffer_0[c_index+:24] + buffer_0[x_index+:24])/9) :
//((buffer_1[l_index+:24] + buffer_1[c_index+:24] + buffer_1[x_index+:24] + buffer_2[l_index+:24] + buffer_2[c_index+:24] + {red, green, blue} + buffer_0[l_index+:24] + buffer_0[c_index+:24] + buffer_0[x_index+:24])/9);
//
//assign new_image = (x > 1  && x < IMAGE_W-1 && y> 1 && y < IMAGE_H-1) ? blur : {red,green,blue};

//Method 4: Just colour detect
wire red_detect, yellow_detect, blue_detect, pink_detect, teal_detect;
assign red_detect = (red >= 8'd90 && red <= 8'd150) && (green >= 8'd30 && green <= 8'd90) && (blue >= 8'd20 && blue <= 8'd50) && (red > green) && (green > blue) && (red + green + blue < 350);
assign yellow_detect = (red >= 8'd90 && red <= 8'd150) && (green >= 8'd160 && green <= 8'd230) && (blue >= 8'd55 && blue <= 8'd100) && (green > red) && (red > blue);
assign blue_detect = (red >= 8'd0 && red <= 8'd40) && (green >= 8'd20 && green <= 8'd60) && (blue >= 8'd20 && blue <= 8'd100) && (blue > green) && (green > red);
assign pink_detect = (red >= 8'd90 && red <= 8'd200) && (green >= 8'd75 && green <= 8'd200) && (blue >= 8'd65 && blue <= 8'd170) && (red > green) && (green > blue) && (red+green+blue > 350) ;
assign teal_detect = (red>= 8'd20 && red <=8'd60) && (green >= 8'd60 && green <= 8'd140) && (blue >= 8'd30 && blue <= 8'd150) && (green > blue) && (blue > red);

wire red_bb_active, yellow_bb_active, blue_bb_active, pink_bb_active, teal_bb_active;
assign red_bb_active = (x == red_left) | (x == red_right) | (y == red_top) | (y == red_bottom);
assign yellow_bb_active = (x == yellow_left) | (x == yellow_right) | (y == yellow_top) | (y == yellow_bottom);
assign blue_bb_active = (x == blue_left) | (x == blue_right) | (y == blue_top) | (y == blue_bottom);
assign pink_bb_active = (x == pink_left) | (x == pink_right) | (y == pink_top) | (y == pink_bottom);
assign teal_bb_active = (x == teal_left) | (x == teal_right) | (y == teal_top) | (y == teal_bottom);

assign new_image = 	//red_bb_active ? 24'hff0000:
							//yellow_bb_active ? 24'hffff00:
							//blue_bb_active ? 24'h0000ff:
							//pink_bb_active ? 24'hff00ff:
							//teal_bb_active ? 24'h008080:
							red_detect ? 24'hff0000:
							yellow_detect ? 24'hffff00:
							blue_detect ? 24'h0000ff:
							pink_detect ? 24'hff00ff:
							teal_detect ? 24'h008080:
							{grey, grey, grey};

//end of ball detects  
reg [10:0] x, y;
reg packet_video;
always@(posedge clk) begin
	if (sop) begin
		x <= 11'h0;
		y <= 11'h0;
		//current_row <= 0;
		packet_video <= (blue[3:0] == 3'h0);
	end
	else if (in_valid) begin
		if (x == IMAGE_W-1) begin
			x <= 11'h0;
			y <= y + 11'h1;
			
//			if(current_row == 0) begin
//				current_row <= 1;
//			end else if(current_row == 1) begin
//				current_row <= 2;
//			end else begin
//				current_row <= 0;
//			end
		end
		else begin
			x <= x + 11'h1;
		end
		
	end
end

//Find first and last red pixels
reg [10:0] r_x_min, r_y_min, r_x_max, r_y_max;
reg [10:0] y_x_min, y_y_min, y_x_max, y_y_max;
reg [10:0] b_x_min, b_y_min, b_x_max, b_y_max;
reg [10:0] p_x_min, p_y_min, p_x_max, p_y_max;
reg [10:0] t_x_min, t_y_min, t_x_max, t_y_max;
always@(posedge clk) begin
	if (red_detect & in_valid) begin	//Update bounds when the pixel is red
		if (x < r_x_min) r_x_min <= x;
		if (x > r_x_max) r_x_max <= x;
		if (y < r_y_min) r_y_min <= y;
		r_y_max <= y;
	end
	if (yellow_detect & in_valid) begin	//Update bounds when the pixel is yellow
		if (x < y_x_min) y_x_min <= x;
		if (x > y_x_max) y_x_max <= x;
		if (y < y_y_min) y_y_min <= y;
		y_y_max <= y;
	end
	if (blue_detect & in_valid) begin	//Update bounds when the pixel is blue
		if (x < b_x_min) b_x_min <= x;
		if (x > b_x_max) b_x_max <= x;
		if (y < b_y_min) b_y_min <= y;
		b_y_max <= y;
	end
	if (pink_detect & in_valid) begin	//Update bounds when the pixel is pink
		if (x < p_x_min) p_x_min <= x;
		if (x > p_x_max) p_x_max <= x;
		if (y < p_y_min) p_y_min <= y;
		p_y_max <= y;
	end
	if (teal_detect & in_valid) begin	//Update bounds when the pixel is teal
		if (x < t_x_min) t_x_min <= x;
		if (x > t_x_max) t_x_max <= x;
		if (y < t_y_min) t_y_min <= y;
		t_y_max <= y;
	end
	if (sop & in_valid) begin	//Reset bounds on start of packet
		r_x_min <= IMAGE_W-11'h1;
		r_x_max <= 0;
		r_y_min <= IMAGE_H-11'h1;
		r_y_max <= 0;
		y_x_min <= IMAGE_W-11'h1;
		y_x_max <= 0;
		y_y_min <= IMAGE_H-11'h1;
		y_y_max <= 0;
		b_x_min <= IMAGE_W-11'h1;
		b_x_max <= 0;
		b_y_min <= IMAGE_H-11'h1;
		b_y_max <= 0;
		p_x_min <= IMAGE_W-11'h1;
		p_x_max <= 0;
		p_y_min <= IMAGE_H-11'h1;
		p_y_max <= 0;
		t_x_min <= IMAGE_W-11'h1;
		t_x_max <= 0;
		t_y_min <= IMAGE_H-11'h1;
		t_y_max <= 0;
	end
end

//Process bounding box at the end of the frame.
reg [4:0] msg_state;
reg [10:0] red_left, red_right, red_top, red_bottom;
reg [10:0] yellow_left, yellow_right, yellow_top, yellow_bottom;
reg [10:0] blue_left, blue_right, blue_top, blue_bottom;
reg [10:0] pink_left, pink_right, pink_top, pink_bottom;
reg [10:0] teal_left, teal_right, teal_top, teal_bottom;

reg [7:0] frame_count;
always@(posedge clk) begin
	if (eop & in_valid & packet_video) begin  //Ignore non-video packets
		
		//Latch edges for display overlay on next frame
		red_left <= r_x_min;
		red_right <= r_x_max;
		red_top <= r_y_min;
		red_bottom <= r_y_max;
		
		yellow_left <= y_x_min;
		yellow_right <= y_x_max;
		yellow_top <= y_y_min;
		yellow_bottom <= y_y_max;
		
		blue_left <= b_x_min;
		blue_right <= b_x_max;
		blue_top <= b_y_min;
		blue_bottom <= b_y_max;
		
		pink_left <= p_x_min;
		pink_right <= p_x_max;
		pink_top <= p_y_min;
		pink_bottom <= p_y_max;
		
		teal_left <= t_x_min;
		teal_right <= t_x_max;
		teal_top <= t_y_min;
		teal_bottom <= t_y_max;
		
		
		//Start message writer FSM once every MSG_INTERVAL frames, if there is room in the FIFO
		frame_count <= frame_count - 1;
		
		if (frame_count == 0 && msg_buf_size < MESSAGE_BUF_MAX - 3) begin
			msg_state <= 3'b001;
			frame_count <= MSG_INTERVAL-1;
		end
	end
	
	//Cycle through message writer states once started
	if (msg_state != 3'b000) msg_state <= msg_state + 3'b001;

end
	
//Generate output messages for CPU
reg [31:0] msg_buf_in; 
wire [31:0] msg_buf_out;
reg msg_buf_wr;
wire msg_buf_rd, msg_buf_flush;
wire [7:0] msg_buf_size;
wire msg_buf_empty;
wire collision_detect;
`define RED_BOX_MSG_ID "RBB"

always@(*) begin	//Write words to FIFO as state machine advances
	case(msg_state)
		4'b0000: begin
			msg_buf_in = 32'b0;
			msg_buf_wr = 1'b0;
		end
		4'b0001: begin
			msg_buf_in = 32'd1;	//Red identifier
			msg_buf_wr = 1'b1;
		end
		4'b0010: begin
			msg_buf_in = {5'd0, r_x_min, 5'd0, r_y_max};	//Red x1, y2
			msg_buf_wr = 1'b1;
		end
		4'b0011: begin
			msg_buf_in = {5'd0, r_x_max, 5'd0, r_y_min}; 	//Red x2, y1
			msg_buf_wr = 1'b1;
		end
		4'b0100: begin
			msg_buf_in = 32'd4; //Yellow identifier
			msg_buf_wr = 1'b1;
		end
		4'b0101: begin
			msg_buf_in = {5'd0, y_x_min, 5'd0, y_y_max}; //yellow x1
			msg_buf_wr = 1'b1;
		end
		4'b0110: begin
			msg_buf_in = {5'd0, y_x_max, 5'd0, y_y_min}; //yellow x2
			msg_buf_wr = 1'b1;
		end
		4'b0111: begin
			msg_buf_in = 32'd7; //Blue identifier
			msg_buf_wr = 1'b1;
		end
		4'b1000: begin
			msg_buf_in = {5'd0, b_x_min, 5'd0, b_y_max};	//blue x1
			msg_buf_wr = 1'b1;
		end
		4'b1001: begin
			msg_buf_in = {5'd0, b_x_max, 5'd0, b_y_min};	//blue x2
			msg_buf_wr = 1'b1;
		end
		4'b1010: begin
			msg_buf_in = 32'd10;	//Pink identifier
			msg_buf_wr = 1'b1;
		end
		4'b1011: begin
			msg_buf_in = {5'd0, p_x_min, 5'd0, p_y_max}; //pink x1
			msg_buf_wr = 1'b1;
		end
		4'b1100: begin
			msg_buf_in = {5'd0, p_x_max, 5'd0, p_y_min}; //pink x2
			msg_buf_wr = 1'b1;
		end
		4'b1101: begin
			msg_buf_in = 32'd13; //Teal identifier
			msg_buf_wr = 1'b1;
		end
		4'b1110: begin
			msg_buf_in = {5'd0, t_x_min, 5'd0, t_y_max}; //teal x1
			msg_buf_wr = 1'b1;
		end
		4'b1111: begin
			msg_buf_in = {5'd0, t_x_max, 5'd0, t_y_min}; //teal x2
			msg_buf_wr = 1'b1;
		end
	endcase
end


//Output message FIFO
MSG_FIFO	MSG_FIFO_inst (
	.clock (clk),
	.data (msg_buf_in),
	.rdreq (msg_buf_rd),
	.sclr (~reset_n | msg_buf_flush),
	.wrreq (msg_buf_wr),
	.q (msg_buf_out),
	.usedw (msg_buf_size),
	.empty (msg_buf_empty)
	);


//Streaming registers to buffer video signal
STREAM_REG #(.DATA_WIDTH(26)) in_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(sink_ready),
	.valid_out(in_valid),
	.data_out({red,green,blue,sop,eop}),
	.ready_in(out_ready),
	.valid_in(sink_valid),
	.data_in({sink_data,sink_sop,sink_eop})
);

STREAM_REG #(.DATA_WIDTH(26)) out_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(out_ready),
	.valid_out(source_valid),
	.data_out({source_data,source_sop,source_eop}),
	.ready_in(source_ready),
	.valid_in(in_valid),
	.data_in({red_out, green_out, blue_out, sop, eop})
);


/////////////////////////////////
/// Memory-mapped port		 /////
/////////////////////////////////

// Addresses
`define REG_STATUS    			0
`define READ_MSG    				1
`define READ_ID    				2
`define REG_BBCOL					3

//Status register bits
// 31:16 - unimplemented
// 15:8 - number of words in message buffer (read only)
// 7:5 - unused
// 4 - flush message buffer (write only - read as 0)
// 3:0 - unused


// Process write

reg  [7:0]   reg_status;
reg	[23:0]	bb_col;

always @ (posedge clk)
begin
	if (~reset_n)
	begin
		reg_status <= 8'b0;
		bb_col <= BB_COL_DEFAULT;
	end
	else begin
		if(s_chipselect & s_write) begin
		   if      (s_address == `REG_STATUS)	reg_status <= s_writedata[7:0];
		   if      (s_address == `REG_BBCOL)	bb_col <= s_writedata[23:0];
		end
	end
end


//Flush the message buffer if 1 is written to status register bit 4
assign msg_buf_flush = (s_chipselect & s_write & (s_address == `REG_STATUS) & s_writedata[4]);


// Process reads
reg read_d; //Store the read signal for correct updating of the message buffer

// Copy the requested word to the output port when there is a read.
always @ (posedge clk)
begin
   if (~reset_n) begin
	   s_readdata <= {32'b0};
		read_d <= 1'b0;
	end
	
	else if (s_chipselect & s_read) begin
		if   (s_address == `REG_STATUS) s_readdata <= {16'b0,msg_buf_size,reg_status};
		if   (s_address == `READ_MSG) s_readdata <= {msg_buf_out};
		if   (s_address == `READ_ID) s_readdata <= 32'h1234EEE2;
		if   (s_address == `REG_BBCOL) s_readdata <= {8'h0, bb_col};
	end
	
	read_d <= s_read;
end

//Fetch next word from message buffer after read from READ_MSG
assign msg_buf_rd = s_chipselect & s_read & ~read_d & ~msg_buf_empty & (s_address == `READ_MSG);
						


endmodule