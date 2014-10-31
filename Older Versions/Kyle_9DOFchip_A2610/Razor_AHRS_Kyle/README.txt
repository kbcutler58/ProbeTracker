Notes on arduino code

3/6/14

Output of Razor_AHRS_Kyle

timestamp in ms , x mouse, y mouse, tracking quality,
"YPR" indicates yaw pitch roll output,
Yaw, Pitch, Roll (degrees -179.99:179.99 between 4 and 7 characters)

Changes to original Razor_AHRS Code

Added A2610 Section
	A2610_Init()
		keeps mouse always in high power mode
	A2610_read_sensor()
		reads registers for x, y, and squal
	A2610_output()
		outputs x, y, and squal to serial
	A2610_read_reg
		reads register on A2610
	A2610_write_reg
		writes register on 2610

Editted Output Section
	Changed #YPR= to YPR,
	Added output_timestamp()
		outputs millis() to serial

Editted Razor_AHRS_Kyle
	Added output_timestamp call in loop
	Added A2610_output call in loop
	Added A2610_read_sensor call in read_sensors function

