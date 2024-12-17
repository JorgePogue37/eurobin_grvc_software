Register	RAM Address Range	Buffer Position			Notes
--------	-----------------	---------------			-----
Voltage				[54]			[9]					1 unit = 0.1 [V]
Temperature			[55]			[10]				1 unit = 1 ºC
Control Mode		[56]			[11]				0: position ; 1: turn/velocity
Tick				[57]			[12]				1 unit = 11.2 [ms] (1 byte time stamp)
Calib. Pos.			[58, 59]		[13, 14]
Abs. Pos.			[60, 61]		[LSB:15, MSB:16]	Offset: 16384
Diff. Pos.			[62, 63]		[17, 18]			Codified as an int16_t
PWM					[64, 65]		[19, 20]			Codified as an int16_t
Abs. 2nd Pos.		[66, 67]		[LSB:21, MSB:22]	Offset: 1024
Abs. Goal Pos.		[68, 69]		[LSB:23, MSB:24]	Offset: 16384

