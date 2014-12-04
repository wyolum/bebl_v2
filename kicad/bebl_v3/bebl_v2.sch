EESchema Schematic File Version 2
LIBS:bebl_v2
LIBS:power
LIBS:bebl_v2-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "BEBL_v2 (bar end brake light)"
Date "2 may 2012"
Rev "1"
Comp "WyoLum"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	550  3050 750  3050
Connection ~ 900  3275
Wire Wire Line
	900  3250 900  3625
Wire Wire Line
	900  3625 700  3625
Wire Wire Line
	700  3625 700  3575
Connection ~ 700  3050
Wire Wire Line
	1500 2950 1400 2950
Wire Wire Line
	1400 2950 1400 2800
Wire Wire Line
	1400 2800 900  2800
Wire Wire Line
	1500 2250 1075 2250
Wire Notes Line
	550  7750 550  6300
Wire Notes Line
	1400 7750 1400 6300
Wire Wire Line
	1650 6800 1975 6800
Wire Wire Line
	1975 6800 1975 6750
Wire Wire Line
	1975 6750 2000 6750
Wire Wire Line
	2550 7000 1650 7000
Wire Wire Line
	2350 7300 2350 7200
Wire Wire Line
	900  6900 625  6900
Wire Wire Line
	900  7100 625  7100
Wire Wire Line
	625  7300 900  7300
Connection ~ 2350 7300
Connection ~ 1075 800 
Wire Wire Line
	1075 1175 1075 800 
Connection ~ 3775 800 
Wire Wire Line
	3775 725  3775 1175
Connection ~ 1075 4975
Connection ~ 1300 2450
Wire Wire Line
	1300 4975 1300 1650
Connection ~ 2350 800 
Wire Wire Line
	975  2650 1500 2650
Wire Wire Line
	975  2650 975  800 
Wire Wire Line
	3450 4350 3400 4350
Wire Wire Line
	2350 4850 2350 5100
Wire Wire Line
	2350 800  2350 1950
Wire Wire Line
	2450 800  2450 1950
Connection ~ 2450 800 
Wire Wire Line
	3450 4450 3400 4450
Wire Wire Line
	3450 4550 3400 4550
Wire Wire Line
	1500 2450 1300 2450
Connection ~ 2350 4975
Wire Wire Line
	1300 800  1300 1200
Connection ~ 1300 800 
Wire Wire Line
	1075 4650 1075 4975
Connection ~ 1300 4975
Wire Wire Line
	3975 800  3975 1175
Wire Wire Line
	3400 3250 3625 3250
Wire Wire Line
	3400 3150 3625 3150
Wire Wire Line
	2350 7200 2550 7200
Wire Wire Line
	900  7200 625  7200
Wire Wire Line
	900  7000 625  7000
Wire Wire Line
	900  6800 625  6800
Wire Wire Line
	2550 7100 1650 7100
Wire Wire Line
	975  800  4475 800 
Wire Wire Line
	1650 7300 2550 7300
Wire Wire Line
	2550 6900 1650 6900
Wire Wire Line
	2450 6750 2475 6750
Wire Wire Line
	2475 6750 2475 6800
Wire Wire Line
	2475 6800 2550 6800
Wire Notes Line
	3100 6300 1500 6300
Wire Notes Line
	3100 6300 3100 7750
Wire Notes Line
	3100 7750 1500 7750
Wire Notes Line
	1500 7750 1500 6300
Wire Wire Line
	1075 1675 1075 4050
Connection ~ 1075 2250
Wire Wire Line
	900  3275 1400 3275
Wire Wire Line
	1400 3275 1400 3150
Wire Wire Line
	1400 3150 1500 3150
Wire Wire Line
	700  2975 700  3125
Wire Wire Line
	700  2525 700  2475
Wire Wire Line
	700  2475 900  2475
Wire Wire Line
	900  2475 900  2850
Connection ~ 900  2800
Wire Wire Line
	550  3050 550  4975
Wire Wire Line
	550  4975 2450 4975
Wire Wire Line
	2450 4975 2450 4850
Text Label 1400 3275 0    50   ~ 0
PB7
Text Label 1400 2800 0    50   ~ 0
PB6
$Comp
L GND #PWR01
U 1 1 4F5AFC62
P 2350 5100
F 0 "#PWR01" H 2350 5100 30  0001 C CNN
F 1 "GND" H 2350 5030 30  0001 C CNN
F 2 "" H 2350 5100 60  0001 C CNN
F 3 "" H 2350 5100 60  0001 C CNN
	1    2350 5100
	1    0    0    -1  
$EndComp
$Comp
L C_NPOL C2
U 1 1 4EC1290E
P 700 3350
F 0 "C2" H 725 3225 50  0000 L CNN
F 1 "22p" H 725 3475 50  0000 L CNN
F 2 "bebl_v2_pretty:c_0805" H 700 3350 60  0001 C CNN
F 3 "" H 700 3350 60  0001 C CNN
F 4 "490-1734-1-ND" H 700 3550 60  0001 C CNN "Digikey"
	1    700  3350
	1    0    0    -1  
$EndComp
$Comp
L C_NPOL C1
U 1 1 4EC128EB
P 700 2750
F 0 "C1" H 725 2875 50  0000 L CNN
F 1 "22p" H 725 2625 50  0000 L CNN
F 2 "bebl_v2_pretty:c_0805" H 700 2750 60  0001 C CNN
F 3 "" H 700 2750 60  0001 C CNN
F 4 "490-1734-1-ND" H 700 2950 60  0001 C CNN "Digikey"
	1    700  2750
	1    0    0    -1  
$EndComp
Text Notes 1500 6300 0    60   Italic 12
FTDI Header
Text Notes 550  6300 0    60   Italic 12
ISP/PROG Header
Text Label 625  7300 0    60   ~ 0
GND
$Comp
L +3.3V #PWR02
U 1 1 4E1EF3E1
P 3775 725
F 0 "#PWR02" H 3775 685 30  0001 C CNN
F 1 "+3.3V" H 3775 835 30  0000 C CNN
F 2 "" H 3775 725 60  0001 C CNN
F 3 "" H 3775 725 60  0001 C CNN
	1    3775 725 
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 P3
U 1 1 4E1ED2A7
P 7450 3775
F 0 "P3" H 7450 3875 50  0000 C CNN
F 1 "Ext_LiPo" H 7450 3975 50  0000 C CNN
F 2 "bebl_v2_pretty:pin_strip_2" H 7450 3775 60  0001 C CNN
F 3 "" H 7450 3775 60  0001 C CNN
	1    7450 3775
	-1   0    0    -1  
$EndComp
Text Label 1650 7100 0    60   ~ 0
3V3
Text Notes 3300 6575 0    60   Italic 12
Mounting Holes
Text Label 625  6800 0    60   ~ 0
MISO
Text Label 625  6900 0    60   ~ 0
3V3
Text Label 625  7000 0    60   ~ 0
SCK
Text Label 625  7100 0    60   ~ 0
MOSI
Text Label 625  7200 0    60   ~ 0
RESET
$Comp
L ISP P1
U 1 1 4DCD1997
P 1250 7050
F 0 "P1" H 1250 7275 60  0000 C CNN
F 1 "ISP" H 1250 7400 60  0000 C CNN
F 2 "bebl_v2_pretty:Header_3x2_ISP" H 1250 7050 60  0001 C CNN
F 3 "" H 1250 7050 60  0001 C CNN
	1    1250 7050
	1    0    0    -1  
$EndComp
$Comp
L SCREW SC1
U 1 1 4DCCE748
P 3425 6700
F 0 "SC1" H 3425 6600 40  0000 C CNN
F 1 "SCREW" H 3425 6800 40  0001 C CNN
F 2 "bebl_v2_pretty:screw_3mm" H 3425 6700 60  0001 C CNN
F 3 "" H 3425 6700 60  0001 C CNN
	1    3425 6700
	1    0    0    -1  
$EndComp
Text Label 1650 7300 0    60   ~ 0
GND
Text Label 1650 6900 0    60   ~ 0
TXD
Text Label 1650 7000 0    60   ~ 0
RXD
Text Label 1650 6800 0    60   ~ 0
RESET
$Comp
L CONN_6 P2
U 1 1 4DCB971F
P 2900 7050
F 0 "P2" H 2900 6825 60  0000 C CNN
F 1 "FTDI" H 2900 6700 60  0000 C CNN
F 2 "bebl_v2_pretty:PIN_ARRAY-6X1" H 2900 7050 60  0001 C CNN
F 3 "" H 2900 7050 60  0001 C CNN
	1    2900 7050
	1    0    0    1   
$EndComp
Text Label 1075 2250 1    60   ~ 0
RESET
Text Label 3625 3150 0    40   ~ 0
RXD
Text Label 3625 3250 0    40   ~ 0
TXD
$Comp
L GND #PWR03
U 1 1 4DCAC327
P 8250 4350
F 0 "#PWR03" H 8250 4350 30  0001 C CNN
F 1 "GND" H 8250 4280 30  0001 C CNN
F 2 "" H 8250 4350 60  0001 C CNN
F 3 "" H 8250 4350 60  0001 C CNN
	1    8250 4350
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 4DCAC172
P 3975 1425
F 0 "R3" H 4050 1225 50  0000 C CNN
F 1 "10k" V 3975 1425 50  0000 C CNN
F 2 "bebl_v2_pretty:r_0805" H 3975 1425 60  0001 C CNN
F 3 "" H 3975 1425 60  0001 C CNN
	1    3975 1425
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 4DCAC161
P 3775 1425
F 0 "R2" H 3850 1225 50  0000 C CNN
F 1 "10k" V 3775 1425 50  0000 C CNN
F 2 "bebl_v2_pretty:r_0805" H 3775 1425 60  0001 C CNN
F 3 "" H 3775 1425 60  0001 C CNN
	1    3775 1425
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW1
U 1 1 4DCA7D67
P 1075 4350
F 0 "SW1" V 1075 4550 50  0000 C CNN
F 1 "RST" V 975 4550 50  0000 C CNN
F 2 "bebl_v2_pretty:Switch_SMT" H 1075 4350 60  0001 C CNN
F 3 "" H 1075 4350 60  0001 C CNN
F 4 "" H 1325 4560 60  0001 C CNN "DigiKey"
	1    1075 4350
	0    -1   -1   0   
$EndComp
$Comp
L R R1
U 1 1 4DCA7BFC
P 1075 1425
F 0 "R1" H 1125 1225 50  0000 C CNN
F 1 "10k" V 1075 1425 50  0000 C CNN
F 2 "bebl_v2_pretty:r_0805" H 1075 1425 60  0001 C CNN
F 3 "" H 1075 1425 60  0001 C CNN
	1    1075 1425
	1    0    0    -1  
$EndComp
$Comp
L C_NPOL C3
U 1 1 4DCA7BAC
P 1300 1425
F 0 "C3" H 1350 1550 50  0000 L CNN
F 1 "100n" H 1350 1275 50  0000 L CNN
F 2 "bebl_v2_pretty:c_0805" H 1300 1425 60  0001 C CNN
F 3 "" H 1300 1425 60  0001 C CNN
	1    1300 1425
	1    0    0    -1  
$EndComp
Text Label 3450 2650 0    40   ~ 0
SDA
Text Label 3450 2750 0    40   ~ 0
SCL
Text Label 3450 4350 0    40   ~ 0
MOSI
Text Label 3450 4550 0    40   ~ 0
SCK
Text Label 3450 4450 0    40   ~ 0
MISO
$Comp
L ATMEGA8-AI U1
U 1 1 4DCA5EC5
P 2400 3250
F 0 "U1" H 1700 4400 50  0000 L BNN
F 1 "ATMEGA328" H 1750 2500 50  0000 L BNN
F 2 "bebl_v2_pretty:TQFP32" H 2925 1675 50  0001 C CNN
F 3 "" H 2400 3250 60  0001 C CNN
	1    2400 3250
	1    0    0    -1  
$EndComp
Wire Notes Line
	1400 6300 550  6300
Wire Notes Line
	1400 7750 550  7750
$Comp
L CRYSTAL X1
U 1 1 5313C3B2
P 900 3050
F 0 "X1" V 975 3300 60  0000 C CNN
F 1 "8MHz" V 875 3375 60  0000 C CNN
F 2 "bebl_v2_pretty:Xtal_SMD4" H 900 3050 60  0001 C CNN
F 3 "" H 900 3050 60  0000 C CNN
	1    900  3050
	0    1    1    0   
$EndComp
Wire Wire Line
	1500 2550 1400 2550
Wire Wire Line
	3775 1675 3775 2650
Wire Wire Line
	3775 2650 3400 2650
Wire Wire Line
	3400 2750 3975 2750
Wire Wire Line
	3975 2750 3975 1675
$Comp
L TPS61221DCK U5
U 1 1 5415CD74
P 9650 3450
F 0 "U5" H 9650 3450 50  0000 C CNN
F 1 "TPS61221DCK" H 9650 3150 50  0000 C CNN
F 2 "bebl_v2_pretty:SOT23-6_LDO" H 9650 3450 60  0001 C CNN
F 3 "" H 9650 3450 60  0000 C CNN
	1    9650 3450
	1    0    0    -1  
$EndComp
$Comp
L INDUCTOR2 L2
U 1 1 5415D384
P 8625 3300
F 0 "L2" H 8625 3475 50  0000 C CNN
F 1 "4u7" H 8625 3300 50  0000 C CNN
F 2 "bebl_v2_pretty:Inductor_LQH3NPN" H 8625 3300 60  0001 C CNN
F 3 "" H 8625 3300 60  0000 C CNN
	1    8625 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 3600 9050 3600
Wire Wire Line
	8250 3300 8250 3725
Wire Wire Line
	8250 3450 9050 3450
Wire Wire Line
	7000 3300 8375 3300
Connection ~ 8250 3450
Wire Wire Line
	8875 3300 9050 3300
Wire Wire Line
	10250 3300 10700 3300
Wire Wire Line
	10325 3300 10325 3450
Wire Wire Line
	10325 3450 10250 3450
$Comp
L C_POL C7
U 1 1 5415D639
P 8250 3925
F 0 "C7" H 8150 4025 50  0000 L CNN
F 1 "10uF" H 8250 3775 50  0000 L CNN
F 2 "bebl_v2_pretty:c_0805" H 8250 3925 60  0001 C CNN
F 3 "" H 8250 3925 60  0000 C CNN
	1    8250 3925
	1    0    0    -1  
$EndComp
$Comp
L C_POL C10
U 1 1 5415D64B
P 10475 3925
F 0 "C10" H 10375 4025 50  0000 L CNN
F 1 "10uF" H 10475 3775 50  0000 L CNN
F 2 "bebl_v2_pretty:c_0805" H 10475 3925 60  0001 C CNN
F 3 "" H 10475 3925 60  0000 C CNN
	1    10475 3925
	1    0    0    -1  
$EndComp
Wire Wire Line
	10475 3300 10475 3725
Connection ~ 10325 3300
Wire Wire Line
	10250 3600 10325 3600
Wire Wire Line
	10325 3600 10325 4250
Wire Wire Line
	7000 4250 10475 4250
Wire Wire Line
	10475 4250 10475 4175
Wire Wire Line
	8250 4175 8250 4350
Connection ~ 10325 4250
Connection ~ 8250 3600
Connection ~ 8250 3300
Connection ~ 8250 4250
Connection ~ 10475 3300
Text Label 7925 4250 0    50   ~ 0
GND
Text Label 10700 3300 0    50   ~ 0
3V3
Text Label 7925 3300 0    50   ~ 0
BAT
Wire Wire Line
	7925 3675 7925 3300
Wire Wire Line
	7925 3875 7925 4250
Wire Wire Line
	7800 3675 7925 3675
Wire Wire Line
	7925 3875 7800 3875
Text Label 3500 800  0    50   ~ 0
3V3
$Comp
L MCP73831 U4
U 1 1 5415E477
P 9650 1125
F 0 "U4" H 9650 1125 50  0000 C CNN
F 1 "MCP73831" H 9650 825 50  0000 C CNN
F 2 "bebl_v2_pretty:SOT23-5" H 9650 1125 60  0001 C CNN
F 3 "" H 9650 1125 60  0000 C CNN
	1    9650 1125
	1    0    0    -1  
$EndComp
$Comp
L C_POL C8
U 1 1 5415E493
P 8500 1550
F 0 "C8" H 8400 1650 50  0000 L CNN
F 1 "4u7" H 8500 1400 50  0000 L CNN
F 2 "bebl_v2_pretty:c_0805" H 8500 1550 60  0001 C CNN
F 3 "" H 8500 1550 60  0000 C CNN
	1    8500 1550
	1    0    0    -1  
$EndComp
$Comp
L DIODE_LED D8
U 1 1 5415E4A0
P 10275 2400
F 0 "D8" V 10175 2450 50  0000 C CNN
F 1 "CHG" V 10400 2325 40  0000 C CNN
F 2 "bebl_v2_pretty:LED-0805" H 10275 2400 60  0001 C CNN
F 3 "" H 10275 2400 60  0000 C CNN
	1    10275 2400
	0    -1   -1   0   
$EndComp
$Comp
L R2 R6
U 1 1 5415E4B4
P 8975 1575
F 0 "R6" H 8900 1750 50  0000 C CNN
F 1 "150E" V 8975 1575 50  0000 C CNN
F 2 "bebl_v2_pretty:r_0805" H 8975 1575 60  0001 C CNN
F 3 "" H 8975 1575 60  0000 C CNN
	1    8975 1575
	-1   0    0    1   
$EndComp
$Comp
L C_POL C12
U 1 1 5415E4EE
P 10750 1550
F 0 "C12" H 10775 1700 50  0000 L CNN
F 1 "4u7" H 10750 1400 50  0000 L CNN
F 2 "bebl_v2_pretty:c_0805" H 10750 1550 60  0001 C CNN
F 3 "" H 10750 1550 60  0000 C CNN
	1    10750 1550
	1    0    0    -1  
$EndComp
$Comp
L R2 R7
U 1 1 5415E508
P 10475 1575
F 0 "R7" H 10400 1750 50  0000 C CNN
F 1 "2k" V 10475 1575 50  0000 C CNN
F 2 "bebl_v2_pretty:r_0805" H 10475 1575 60  0001 C CNN
F 3 "" H 10475 1575 60  0000 C CNN
	1    10475 1575
	-1   0    0    1   
$EndComp
Wire Wire Line
	10250 975  10750 975 
Wire Wire Line
	10750 975  10750 1350
Wire Wire Line
	10250 1125 10475 1125
Wire Wire Line
	10475 1125 10475 1350
Wire Wire Line
	9050 1275 8975 1275
Wire Wire Line
	8975 1275 8975 1350
Wire Wire Line
	8125 975  9050 975 
Wire Wire Line
	8500 975  8500 1350
Wire Wire Line
	8800 900  8800 2525
Connection ~ 8800 975 
Wire Wire Line
	8500 1950 8500 1800
Wire Wire Line
	6925 1950 10750 1950
Wire Wire Line
	10750 1950 10750 1800
Wire Wire Line
	10475 1800 10475 1950
Connection ~ 10475 1950
Wire Wire Line
	10250 1275 10325 1275
Wire Wire Line
	10325 1275 10325 1950
Connection ~ 10325 1950
Connection ~ 8500 975 
Connection ~ 8500 1950
Text Label 8300 1950 0    50   ~ 0
GND
Text Label 8225 975  0    50   ~ 0
VUSB
Text Label 10400 975  2    50   ~ 0
BAT
$Comp
L USB-Micro P4
U 1 1 5415F287
P 7575 1275
F 0 "P4" H 7000 1350 50  0000 C CNN
F 1 "USB-Micro" H 6850 1275 50  0000 C CNN
F 2 "bebl_v2_pretty:USB_B_Micro" H 7575 1275 60  0001 C CNN
F 3 "" H 7575 1275 60  0000 C CNN
	1    7575 1275
	1    0    0    -1  
$EndComp
Wire Wire Line
	8125 1575 8225 1575
Wire Wire Line
	8225 1575 8225 1950
Wire Wire Line
	7275 1950 7275 1875
Connection ~ 8225 1950
Wire Wire Line
	7425 1875 7425 1950
Connection ~ 7425 1950
Wire Wire Line
	7575 1875 7575 1950
Connection ~ 7575 1950
Wire Wire Line
	7725 1875 7725 1950
Connection ~ 7725 1950
NoConn ~ 8125 1125
NoConn ~ 8125 1275
NoConn ~ 8125 1425
$Comp
L C_NPOL C4
U 1 1 5415F8B1
P 2225 6750
F 0 "C4" V 2175 6850 50  0000 L CNN
F 1 "100n" V 2175 6475 50  0000 L CNN
F 2 "bebl_v2_pretty:c_0805" H 2225 6775 60  0001 C CNN
F 3 "" H 2225 6775 60  0000 C CNN
	1    2225 6750
	0    1    1    0   
$EndComp
Text Label 1400 4975 0    50   ~ 0
GND
Wire Wire Line
	1400 2550 1400 2650
Connection ~ 1400 2650
$Comp
L ADXL345 U3
U 1 1 542E7DA5
P 9550 5450
F 0 "U3" H 9525 6000 50  0000 C CNN
F 1 "ADXL345" H 9550 4900 50  0000 C CNN
F 2 "bebl_v2_pretty:ADXL345" H 9550 5100 60  0001 C CNN
F 3 "" H 9550 5100 60  0000 C CNN
	1    9550 5450
	-1   0    0    -1  
$EndComp
NoConn ~ 10100 5300
NoConn ~ 10100 5400
Wire Wire Line
	10175 5700 10175 6250
Wire Wire Line
	10100 5900 10175 5900
Connection ~ 10175 5900
Wire Wire Line
	10575 5800 10100 5800
Connection ~ 10175 5800
Wire Wire Line
	10100 5700 10175 5700
$Comp
L GND #PWR04
U 1 1 542E7DB5
P 10175 6250
F 0 "#PWR04" H 10175 6250 30  0001 C CNN
F 1 "GND" H 10175 6180 30  0001 C CNN
F 2 "" H 10175 6250 60  0001 C CNN
F 3 "" H 10175 6250 60  0001 C CNN
	1    10175 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	10175 5100 10100 5100
Wire Wire Line
	10175 4775 10175 5100
Wire Wire Line
	10300 5100 10300 5175
Wire Wire Line
	10100 5000 10575 5000
Connection ~ 10175 5000
$Comp
L C_NPOL C11
U 1 1 542E7DC9
P 10575 5400
F 0 "C11" H 10600 5525 50  0000 L CNN
F 1 "100nF" V 10625 5100 50  0000 L CNN
F 2 "bebl_v2_pretty:c_0805" H 10575 5400 50  0001 C CNN
F 3 "" H 10575 5400 60  0001 C CNN
F 4 "CAP FILM 0.1UF 63VDC RADIAL" H 10575 5600 60  0001 L CNN "Field4"
F 5 "100nF, 63V" H 10575 5700 60  0001 L CNN "Field5"
F 6 "R82" H 10575 5800 60  0001 L CNN "Field6"
F 7 "Kemet" H 10575 5900 60  0001 L CNN "Field7"
F 8 "R82DC3100AA50J" H 10575 6000 60  0001 L CNN "Field8"
F 9 "Digikey" H 10575 6100 60  0001 L CNN "Field9"
F 10 "399-5863-ND" H 10575 6200 60  0001 L CNN "Field10"
F 11 "http://www.digikey.com/product-detail/en/R82DC3100AA50J/399-5863-ND/2571298?cur=USD" H 10575 6300 60  0001 L CNN "Field11"
	1    10575 5400
	1    0    0    -1  
$EndComp
$Comp
L C_NPOL C13
U 1 1 542E7DD7
P 10850 5400
F 0 "C13" H 10875 5525 50  0000 L CNN
F 1 "100nF" V 10900 5100 50  0000 L CNN
F 2 "bebl_v2_pretty:c_0805" H 10850 5400 50  0001 C CNN
F 3 "" H 10850 5400 60  0001 C CNN
F 4 "CAP FILM 0.1UF 63VDC RADIAL" H 10850 5600 60  0001 L CNN "Field4"
F 5 "100nF, 63V" H 10850 5700 60  0001 L CNN "Field5"
F 6 "R82" H 10850 5800 60  0001 L CNN "Field6"
F 7 "Kemet" H 10850 5900 60  0001 L CNN "Field7"
F 8 "R82DC3100AA50J" H 10850 6000 60  0001 L CNN "Field8"
F 9 "Digikey" H 10850 6100 60  0001 L CNN "Field9"
F 10 "399-5863-ND" H 10850 6200 60  0001 L CNN "Field10"
F 11 "http://www.digikey.com/product-detail/en/R82DC3100AA50J/399-5863-ND/2571298?cur=USD" H 10850 6300 60  0001 L CNN "Field11"
	1    10850 5400
	1    0    0    -1  
$EndComp
$Comp
L C_POL C9
U 1 1 542E7DE5
P 10300 5375
F 0 "C9" H 10200 5475 50  0000 L CNN
F 1 "10uF" V 10350 5075 40  0000 L CNN
F 2 "bebl_v2_pretty:c_0805" H 9900 5350 50  0001 C CNN
F 3 "" H 10300 5375 60  0001 C CNN
F 4 "CAP ALUM 100UF 16V 20% RADIAL" H 10300 5575 60  0001 L CNN "Field4"
F 5 "100u,16V" H 10300 5675 60  0001 L CNN "Field5"
F 6 "Radial, Can, 6.3mm dia" H 10300 5775 60  0001 L CNN "Field6"
F 7 "Panasonic Electronic Components" H 10300 5875 60  0001 L CNN "Field7"
F 8 "ECE-A1CKA101" H 10300 5975 60  0001 L CNN "Field8"
F 9 "Digikey" H 10300 6075 60  0001 L CNN "Field9"
F 10 "P833-ND" H 10300 6175 60  0001 L CNN "Field10"
F 11 "http://www.digikey.com/product-detail/en/ECE-A1CKA101/P833-ND/44757?cur=USD" H 10300 6275 60  0001 L CNN "Field11"
	1    10300 5375
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 5625 10300 5700
Wire Wire Line
	10300 5700 10850 5700
Wire Wire Line
	10850 5700 10850 5625
Wire Wire Line
	10575 5625 10575 5800
Connection ~ 10575 5700
Wire Wire Line
	10850 5100 10850 5175
Wire Wire Line
	10300 5100 10850 5100
Wire Wire Line
	10575 5000 10575 5175
Connection ~ 10575 5100
Wire Wire Line
	8725 5600 9000 5600
Wire Wire Line
	8725 5700 9000 5700
Wire Wire Line
	9000 5400 8725 5400
Wire Wire Line
	8725 5300 9000 5300
Wire Wire Line
	8625 5200 9000 5200
Wire Wire Line
	8625 5100 9000 5100
Text Label 8725 5400 0    40   ~ 0
SCL
Text Label 8725 5600 0    40   ~ 0
ACL_INT1
Text Label 8725 5700 0    40   ~ 0
ACL_INT2
Wire Wire Line
	8400 4775 10175 4775
Wire Wire Line
	8625 5100 8625 4775
Connection ~ 8625 4775
Wire Wire Line
	8625 6250 8625 5200
$Comp
L GND #PWR05
U 1 1 542E7E3D
P 8625 6250
F 0 "#PWR05" H 8625 6250 30  0001 C CNN
F 1 "GND" H 8625 6180 30  0001 C CNN
F 2 "" H 8625 6250 60  0001 C CNN
F 3 "" H 8625 6250 60  0001 C CNN
	1    8625 6250
	1    0    0    -1  
$EndComp
Text Label 10175 6100 0    50   ~ 0
GND
Text Label 8725 5300 0    40   ~ 0
SDA
Text Label 8400 4775 0    50   ~ 0
3V3
$Comp
L SCREW SC2
U 1 1 542E814F
P 3425 6900
F 0 "SC2" H 3425 6800 40  0000 C CNN
F 1 "SCREW" H 3425 7000 40  0001 C CNN
F 2 "bebl_v2_pretty:screw_3mm" H 3425 6900 60  0001 C CNN
F 3 "" H 3425 6900 60  0001 C CNN
	1    3425 6900
	1    0    0    -1  
$EndComp
$Comp
L SCREW SC3
U 1 1 542E815A
P 3425 7100
F 0 "SC3" H 3425 7000 40  0000 C CNN
F 1 "SCREW" H 3425 7200 40  0001 C CNN
F 2 "bebl_v2_pretty:screw_3mm" H 3425 7100 60  0001 C CNN
F 3 "" H 3425 7100 60  0001 C CNN
	1    3425 7100
	1    0    0    -1  
$EndComp
$Comp
L SCREW SC4
U 1 1 542E8165
P 3425 7300
F 0 "SC4" H 3425 7200 40  0000 C CNN
F 1 "SCREW" H 3425 7400 40  0001 C CNN
F 2 "bebl_v2_pretty:screw_3mm" H 3425 7300 60  0001 C CNN
F 3 "" H 3425 7300 60  0001 C CNN
	1    3425 7300
	1    0    0    -1  
$EndComp
NoConn ~ 10100 5600
$Comp
L I/O B1
U 1 1 5431218A
P 3625 2250
F 0 "B1" H 3705 2250 40  0000 L CNN
F 1 "I/O" H 3625 2305 30  0001 C CNN
F 2 "bebl_v2_pretty:breakout_smd" H 3625 2250 60  0001 C CNN
F 3 "" H 3625 2250 60  0000 C CNN
	1    3625 2250
	1    0    0    -1  
$EndComp
$Comp
L I/O B2
U 1 1 5431219E
P 3625 2350
F 0 "B2" H 3705 2350 40  0000 L CNN
F 1 "I/O" H 3625 2405 30  0001 C CNN
F 2 "bebl_v2_pretty:breakout_smd" H 3625 2350 60  0001 C CNN
F 3 "" H 3625 2350 60  0000 C CNN
	1    3625 2350
	1    0    0    -1  
$EndComp
$Comp
L I/O B3
U 1 1 543121B2
P 3625 2450
F 0 "B3" H 3705 2450 40  0000 L CNN
F 1 "I/O" H 3625 2505 30  0001 C CNN
F 2 "bebl_v2_pretty:breakout_smd" H 3625 2450 60  0001 C CNN
F 3 "" H 3625 2450 60  0000 C CNN
	1    3625 2450
	1    0    0    -1  
$EndComp
$Comp
L I/O B4
U 1 1 543121C6
P 3625 2550
F 0 "B4" H 3705 2550 40  0000 L CNN
F 1 "I/O" H 3625 2605 30  0001 C CNN
F 2 "bebl_v2_pretty:breakout_smd" H 3625 2550 60  0001 C CNN
F 3 "" H 3625 2550 60  0000 C CNN
	1    3625 2550
	1    0    0    -1  
$EndComp
$Comp
L I/O B5
U 1 1 543121DA
P 3625 2850
F 0 "B5" H 3705 2850 40  0000 L CNN
F 1 "I/O" H 3625 2905 30  0001 C CNN
F 2 "bebl_v2_pretty:breakout_smd" H 3625 2850 60  0001 C CNN
F 3 "" H 3625 2850 60  0000 C CNN
	1    3625 2850
	1    0    0    -1  
$EndComp
$Comp
L I/O B6
U 1 1 543121EE
P 3625 2950
F 0 "B6" H 3705 2950 40  0000 L CNN
F 1 "I/O" H 3625 3005 30  0001 C CNN
F 2 "bebl_v2_pretty:breakout_smd" H 3625 2950 60  0001 C CNN
F 3 "" H 3625 2950 60  0000 C CNN
	1    3625 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 2950 3475 2950
Wire Wire Line
	3400 2850 3475 2850
Wire Wire Line
	3400 2250 3475 2250
Wire Wire Line
	3400 2350 3475 2350
Wire Wire Line
	3400 2450 3475 2450
Wire Wire Line
	3400 2550 3475 2550
$Comp
L PWR_FLAG #FLG06
U 1 1 54312EEF
P 8800 900
F 0 "#FLG06" H 8800 1170 30  0001 C CNN
F 1 "PWR_FLAG" H 8800 1130 30  0000 C CNN
F 2 "" H 8800 900 60  0000 C CNN
F 3 "" H 8800 900 60  0000 C CNN
	1    8800 900 
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG07
U 1 1 5431352D
P 6925 1875
F 0 "#FLG07" H 6925 2145 30  0001 C CNN
F 1 "PWR_FLAG" H 6925 2105 30  0000 C CNN
F 2 "" H 6925 1875 60  0000 C CNN
F 3 "" H 6925 1875 60  0000 C CNN
	1    6925 1875
	1    0    0    -1  
$EndComp
Wire Wire Line
	6925 1950 6925 1875
Connection ~ 7275 1950
Wire Notes Line
	6600 600  11000 600 
Wire Notes Line
	11000 2850 6600 2850
Wire Notes Line
	6600 3050 11000 3050
Wire Notes Line
	11000 3050 11000 4450
Wire Notes Line
	11000 4450 6600 4450
Wire Notes Line
	6600 4450 6600 3050
Wire Notes Line
	6600 4650 11000 4650
Wire Notes Line
	11000 6500 6600 6500
Text Notes 5900 6150 0    50   Italic 10
LED DRIVER
Text Notes 6650 4775 0    50   Italic 10
ACCELEROMETER, I2C
Text Notes 6650 3175 0    50   Italic 10
VOLTAGE REGULATOR
Text Notes 7550 700  0    50   Italic 10
LiPo CHARGER
Text Label 10300 1125 0    50   ~ 0
PROG
Text Label 8975 3300 0    40   ~ 0
L
Wire Wire Line
	3400 3650 3900 3650
$Comp
L R R4
U 1 1 54315771
P 4150 3650
F 0 "R4" V 4230 3650 50  0000 C CNN
F 1 "55E" V 4150 3650 50  0000 C CNN
F 2 "bebl_v2_pretty:r_0805" H 4150 3650 60  0001 C CNN
F 3 "" H 4150 3650 60  0000 C CNN
	1    4150 3650
	0    1    1    0   
$EndComp
$Comp
L DIODE_LED D1
U 1 1 5431586D
P 6075 2875
F 0 "D1" H 6075 2975 40  0000 C CNN
F 1 "DRL" H 6075 2775 40  0000 C CNN
F 2 "bebl_v2_pretty:LED-5MM" H 6075 2875 60  0001 C CNN
F 3 "" H 6075 2875 60  0000 C CNN
	1    6075 2875
	1    0    0    -1  
$EndComp
Wire Wire Line
	4625 3650 4400 3650
Text Label 4800 3075 0    40   ~ 0
GND
Text Notes 5100 3250 0    40   ~ 0
1x LED 20mA, 5mm, 9000mcd\n(On Indicator, Day time running light - DRL)
$Comp
L DIODE_LED D2
U 1 1 54487929
P 4625 6975
F 0 "D2" H 4775 6925 40  0000 C CNN
F 1 "LED_R" H 4450 6925 40  0000 C CNN
F 2 "bebl_v2_pretty:LED-5MM" H 4625 6975 60  0001 C CNN
F 3 "" H 4625 6975 60  0000 C CNN
	1    4625 6975
	0    1    1    0   
$EndComp
$Comp
L DIODE_LED D3
U 1 1 54487F12
P 4925 6975
F 0 "D3" H 5075 6925 40  0000 C CNN
F 1 "LED_R" H 4750 6925 40  0000 C CNN
F 2 "bebl_v2_pretty:LED-5MM" H 4925 6975 60  0001 C CNN
F 3 "" H 4925 6975 60  0000 C CNN
	1    4925 6975
	0    1    1    0   
$EndComp
$Comp
L DIODE_LED D4
U 1 1 5448813C
P 5225 6975
F 0 "D4" H 5375 6925 40  0000 C CNN
F 1 "LED_R" H 5050 6925 40  0000 C CNN
F 2 "bebl_v2_pretty:LED-5MM" H 5225 6975 60  0001 C CNN
F 3 "" H 5225 6975 60  0000 C CNN
	1    5225 6975
	0    1    1    0   
$EndComp
$Comp
L DIODE_LED D5
U 1 1 5448836F
P 5525 6975
F 0 "D5" H 5675 6925 40  0000 C CNN
F 1 "LED_R" H 5350 6925 40  0000 C CNN
F 2 "bebl_v2_pretty:LED-5MM" H 5525 6975 60  0001 C CNN
F 3 "" H 5525 6975 60  0000 C CNN
	1    5525 6975
	0    1    1    0   
$EndComp
$Comp
L DIODE_LED D6
U 1 1 544885AB
P 5825 6975
F 0 "D6" H 5975 6925 40  0000 C CNN
F 1 "LED_R" H 5650 6925 40  0000 C CNN
F 2 "bebl_v2_pretty:LED-5MM" H 5825 6975 60  0001 C CNN
F 3 "" H 5825 6975 60  0000 C CNN
	1    5825 6975
	0    1    1    0   
$EndComp
Text Label 3450 3650 0    40   ~ 0
LED_D1
Text Label 4425 3650 0    40   ~ 0
DRL
$Comp
L I/O B14
U 1 1 54491858
P 7000 3500
F 0 "B14" V 7000 3550 40  0000 L CNN
F 1 "LiPo+" V 7075 3500 30  0000 C CNN
F 2 "bebl_v2_pretty:batt_clip" H 7000 3500 60  0001 C CNN
F 3 "" H 7000 3500 60  0000 C CNN
	1    7000 3500
	0    1    1    0   
$EndComp
$Comp
L I/O B15
U 1 1 54491E64
P 7000 4050
F 0 "B15" V 7000 3875 40  0000 L CNN
F 1 "LiPo-" V 7075 4050 30  0000 C CNN
F 2 "bebl_v2_pretty:batt_clip" H 7000 4050 60  0001 C CNN
F 3 "" H 7000 4050 60  0000 C CNN
	1    7000 4050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7000 4250 7000 4200
Connection ~ 7925 4250
Wire Wire Line
	7000 3350 7000 3300
Connection ~ 7925 3300
Wire Wire Line
	7425 625  7425 675 
Wire Wire Line
	7100 625  7425 625 
Wire Wire Line
	7100 625  7100 1950
Connection ~ 7100 1950
Wire Wire Line
	7275 675  7275 625 
Connection ~ 7275 625 
Wire Notes Line
	6400 3800 6400 7500
Wire Notes Line
	6400 7500 4200 7500
Wire Notes Line
	4200 7500 4200 3800
$Comp
L SW_PUSH SW2
U 1 1 544EC7EE
P 4475 2225
F 0 "SW2" V 4475 2425 50  0000 C CNN
F 1 "MODE" V 4375 2425 50  0000 C CNN
F 2 "bebl_v2_pretty:Switch_SMT" H 4475 2225 60  0001 C CNN
F 3 "" H 4475 2225 60  0001 C CNN
F 4 "" H 4725 2435 60  0001 C CNN "DigiKey"
	1    4475 2225
	0    -1   -1   0   
$EndComp
$Comp
L R R8
U 1 1 544EC93F
P 4475 1425
F 0 "R8" H 4550 1225 50  0000 C CNN
F 1 "10k" V 4475 1425 50  0000 C CNN
F 2 "bebl_v2_pretty:r_0805" H 4475 1425 60  0001 C CNN
F 3 "" H 4475 1425 60  0001 C CNN
	1    4475 1425
	1    0    0    -1  
$EndComp
Wire Wire Line
	4475 800  4475 1175
Connection ~ 3975 800 
Wire Wire Line
	4475 1675 4475 1925
Wire Wire Line
	4475 2525 4475 2750
$Comp
L GND #PWR08
U 1 1 544ED2FD
P 4475 2750
F 0 "#PWR08" H 4475 2750 30  0001 C CNN
F 1 "GND" H 4475 2680 30  0001 C CNN
F 2 "" H 4475 2750 60  0001 C CNN
F 3 "" H 4475 2750 60  0001 C CNN
	1    4475 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4125 1800 4125 3550
Wire Wire Line
	4125 1800 4475 1800
Connection ~ 4475 1800
$Comp
L CONN_2 P10
U 1 1 544F01FA
P 5425 2975
F 0 "P10" V 5375 2975 40  0000 C CNN
F 1 "LED_DRL" V 5475 2975 40  0000 C CNN
F 2 "bebl_v2_pretty:pin_strip_2" H 5425 2975 60  0001 C CNN
F 3 "" H 5425 2975 60  0000 C CNN
	1    5425 2975
	-1   0    0    -1  
$EndComp
$Comp
L CONN_2 P8
U 1 1 544F0628
P 5200 2975
F 0 "P8" V 5150 2975 40  0000 C CNN
F 1 "DRL_LED" V 5250 2975 40  0000 C CNN
F 2 "bebl_v2_pretty:pin_socket_2" H 5200 2975 60  0001 C CNN
F 3 "" H 5200 2975 60  0000 C CNN
	1    5200 2975
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 2875 5775 2875
Wire Wire Line
	6300 2875 6350 2875
Wire Wire Line
	6350 2875 6350 3075
Wire Wire Line
	6350 3075 5775 3075
Wire Wire Line
	4850 3075 4800 3075
Wire Wire Line
	4800 3075 4800 3225
$Comp
L GND #PWR09
U 1 1 544F1460
P 4800 3225
F 0 "#PWR09" H 4800 3225 30  0001 C CNN
F 1 "GND" H 4800 3155 30  0001 C CNN
F 2 "" H 4800 3225 60  0001 C CNN
F 3 "" H 4800 3225 60  0001 C CNN
	1    4800 3225
	1    0    0    -1  
$EndComp
Wire Wire Line
	4625 3650 4625 2875
Wire Wire Line
	4625 2875 4850 2875
Text Label 5800 2875 0    39   ~ 0
D1A
Text Label 5800 3075 0    39   ~ 0
LED_GND
$Comp
L SCREW SC5
U 1 1 544E7E14
P 3625 6700
F 0 "SC5" H 3625 6600 40  0000 C CNN
F 1 "SCREW" H 3625 6800 40  0001 C CNN
F 2 "bebl_v2_pretty:screw_3mm" H 3625 6700 60  0001 C CNN
F 3 "" H 3625 6700 60  0001 C CNN
	1    3625 6700
	1    0    0    -1  
$EndComp
$Comp
L SCREW SC6
U 1 1 544E7E1A
P 3625 6900
F 0 "SC6" H 3625 6800 40  0000 C CNN
F 1 "SCREW" H 3625 7000 40  0001 C CNN
F 2 "bebl_v2_pretty:screw_3mm" H 3625 6900 60  0001 C CNN
F 3 "" H 3625 6900 60  0001 C CNN
	1    3625 6900
	1    0    0    -1  
$EndComp
$Comp
L SCREW SC7
U 1 1 544E7E20
P 3625 7100
F 0 "SC7" H 3625 7000 40  0000 C CNN
F 1 "SCREW" H 3625 7200 40  0001 C CNN
F 2 "bebl_v2_pretty:screw_3mm" H 3625 7100 60  0001 C CNN
F 3 "" H 3625 7100 60  0001 C CNN
	1    3625 7100
	1    0    0    -1  
$EndComp
$Comp
L SCREW SC8
U 1 1 544E7E26
P 3625 7300
F 0 "SC8" H 3625 7200 40  0000 C CNN
F 1 "SCREW" H 3625 7400 40  0001 C CNN
F 2 "bebl_v2_pretty:screw_3mm" H 3625 7300 60  0001 C CNN
F 3 "" H 3625 7300 60  0001 C CNN
	1    3625 7300
	1    0    0    -1  
$EndComp
Wire Notes Line
	11000 4650 11000 6500
Wire Notes Line
	6600 6500 6600 4650
Wire Notes Line
	6600 2850 6600 600 
Wire Notes Line
	11000 600  11000 2850
$Comp
L CONN_2 P13
U 1 1 544EF97E
P 9675 2425
F 0 "P13" V 9625 2425 40  0000 C CNN
F 1 "LED_CHG" V 9725 2425 40  0000 C CNN
F 2 "bebl_v2_pretty:pin_strip_2" H 9675 2425 60  0001 C CNN
F 3 "" H 9675 2425 60  0000 C CNN
	1    9675 2425
	-1   0    0    1   
$EndComp
Wire Wire Line
	10275 2175 10275 2125
Wire Wire Line
	10275 2125 10125 2125
Wire Wire Line
	10125 2125 10125 2325
Wire Wire Line
	10125 2325 10025 2325
Wire Wire Line
	10025 2525 10125 2525
Wire Wire Line
	10125 2525 10125 2675
Wire Wire Line
	10125 2675 10275 2675
Wire Wire Line
	10275 2675 10275 2625
$Comp
L CONN_2 P12
U 1 1 544F00AA
P 9450 2425
F 0 "P12" V 9400 2425 40  0000 C CNN
F 1 "CHG_LED" V 9500 2425 40  0000 C CNN
F 2 "bebl_v2_pretty:pin_socket_2" H 9450 2425 60  0001 C CNN
F 3 "" H 9450 2425 60  0000 C CNN
	1    9450 2425
	1    0    0    1   
$EndComp
Wire Wire Line
	8800 2525 9100 2525
Wire Wire Line
	8975 1800 8975 2325
Wire Wire Line
	8975 2325 9100 2325
Wire Wire Line
	4125 3550 3400 3550
Wire Wire Line
	3400 3350 3625 3350
Wire Wire Line
	3400 3450 3625 3450
Text Label 3450 3550 0    40   ~ 0
MODE
Text Label 3625 3450 0    40   ~ 0
ACL_INT1
Text Label 3625 3350 0    40   ~ 0
ACL_INT2
Wire Wire Line
	4625 6150 4625 6750
Wire Wire Line
	4725 6150 4725 6700
Wire Wire Line
	4725 6700 4925 6700
Wire Wire Line
	4925 6700 4925 6750
Wire Wire Line
	4825 6150 4825 6650
Wire Wire Line
	4825 6650 5225 6650
Wire Wire Line
	5225 6650 5225 6750
Wire Wire Line
	4925 6150 4925 6600
Wire Wire Line
	4925 6600 5525 6600
Wire Wire Line
	5525 6600 5525 6750
Wire Wire Line
	5025 6150 5025 6550
Wire Wire Line
	5025 6550 5825 6550
Wire Wire Line
	5825 6550 5825 6750
Wire Wire Line
	4625 7200 4625 7250
Wire Wire Line
	4625 7250 6025 7250
Wire Wire Line
	5825 7250 5825 7200
Wire Wire Line
	6025 7250 6025 6500
Wire Wire Line
	6025 6500 5125 6500
Wire Wire Line
	5125 6500 5125 6150
Connection ~ 5825 7250
Wire Wire Line
	5525 7200 5525 7250
Connection ~ 5525 7250
Wire Wire Line
	5225 7200 5225 7250
Connection ~ 5225 7250
Wire Wire Line
	4925 7200 4925 7250
Connection ~ 4925 7250
$Comp
L R R12
U 1 1 5466729A
P 5025 4875
F 0 "R12" V 5075 4675 50  0000 C CNN
F 1 "55E" V 5025 4875 50  0000 C CNN
F 2 "bebl_v2_pretty:r_0805" H 5025 4875 60  0001 C CNN
F 3 "" H 5025 4875 60  0000 C CNN
	1    5025 4875
	-1   0    0    1   
$EndComp
$Comp
L R R11
U 1 1 546675B9
P 4925 4875
F 0 "R11" V 4975 4675 50  0000 C CNN
F 1 "55E" V 4925 4875 50  0000 C CNN
F 2 "bebl_v2_pretty:r_0805" H 4925 4875 60  0001 C CNN
F 3 "" H 4925 4875 60  0000 C CNN
	1    4925 4875
	-1   0    0    1   
$EndComp
$Comp
L R R10
U 1 1 5466767C
P 4825 4875
F 0 "R10" V 4875 4675 50  0000 C CNN
F 1 "55E" V 4825 4875 50  0000 C CNN
F 2 "bebl_v2_pretty:r_0805" H 4825 4875 60  0001 C CNN
F 3 "" H 4825 4875 60  0000 C CNN
	1    4825 4875
	-1   0    0    1   
$EndComp
$Comp
L R R9
U 1 1 546676D2
P 4725 4875
F 0 "R9" V 4775 4675 50  0000 C CNN
F 1 "55E" V 4725 4875 50  0000 C CNN
F 2 "bebl_v2_pretty:r_0805" H 4725 4875 60  0001 C CNN
F 3 "" H 4725 4875 60  0000 C CNN
	1    4725 4875
	-1   0    0    1   
$EndComp
$Comp
L R R5
U 1 1 54667729
P 4625 4875
F 0 "R5" V 4675 4675 50  0000 C CNN
F 1 "55E" V 4625 4875 50  0000 C CNN
F 2 "bebl_v2_pretty:r_0805" H 4625 4875 60  0001 C CNN
F 3 "" H 4625 4875 60  0000 C CNN
	1    4625 4875
	-1   0    0    1   
$EndComp
Wire Wire Line
	4625 5200 4625 5125
Wire Wire Line
	4725 5125 4725 5200
Wire Wire Line
	4825 5200 4825 5125
Wire Wire Line
	4925 5125 4925 5200
Wire Wire Line
	5025 5200 5025 5125
Wire Wire Line
	4625 4625 4625 4250
Wire Wire Line
	4625 4250 3400 4250
Wire Wire Line
	3400 4150 4725 4150
Wire Wire Line
	4725 4150 4725 4625
Wire Wire Line
	3400 4050 4825 4050
Wire Wire Line
	4825 4050 4825 4625
Wire Wire Line
	3400 3850 4925 3850
Wire Wire Line
	4925 3850 4925 4625
Wire Wire Line
	3400 3750 5025 3750
Wire Wire Line
	5025 3750 5025 4625
Wire Wire Line
	5125 5200 5125 5125
Wire Wire Line
	5125 5125 5500 5125
Wire Wire Line
	5500 5125 5500 5600
$Comp
L GND #PWR010
U 1 1 54669052
P 5500 5600
F 0 "#PWR010" H 5500 5600 30  0001 C CNN
F 1 "GND" H 5500 5530 30  0001 C CNN
F 2 "" H 5500 5600 60  0001 C CNN
F 3 "" H 5500 5600 60  0001 C CNN
	1    5500 5600
	1    0    0    -1  
$EndComp
Text Label 6025 7250 0    39   ~ 0
LED_GND
Text Label 5125 5125 0    50   ~ 0
GND
Text Label 4425 4250 0    40   ~ 0
LED_D2
Text Label 4425 4150 0    40   ~ 0
LED_D3
Text Label 4425 4050 0    40   ~ 0
LED_D4
Text Label 4425 3850 0    40   ~ 0
LED_D5
Text Label 4425 3750 0    40   ~ 0
LED_D6
$Comp
L CONN_3 P5
U 1 1 5466321F
P 4725 5550
F 0 "P5" V 4675 5550 40  0000 C CNN
F 1 "DRV_1" V 4775 5550 40  0000 C CNN
F 2 "bebl_v2_pretty:pin_socket_3" H 4725 5400 60  0001 C CNN
F 3 "" H 4725 5400 60  0000 C CNN
	1    4725 5550
	0    1    1    0   
$EndComp
$Comp
L CONN_3 P7
U 1 1 54663387
P 5025 5550
F 0 "P7" V 4975 5550 40  0000 C CNN
F 1 "DRV_2" V 5075 5550 40  0000 C CNN
F 2 "bebl_v2_pretty:pin_socket_3" H 5025 5400 60  0001 C CNN
F 3 "" H 5025 5400 60  0000 C CNN
	1    5025 5550
	0    1    1    0   
$EndComp
$Comp
L CONN_3 P6
U 1 1 5466340D
P 4725 5800
F 0 "P6" V 4675 5800 40  0000 C CNN
F 1 "LED_1" V 4775 5800 40  0000 C CNN
F 2 "bebl_v2_pretty:PIN_ARRAY-3X1" H 4725 5650 60  0001 C CNN
F 3 "" H 4725 5650 60  0000 C CNN
	1    4725 5800
	0    1    -1   0   
$EndComp
$Comp
L CONN_3 P9
U 1 1 5466347A
P 5025 5800
F 0 "P9" V 4975 5800 40  0000 C CNN
F 1 "LED_2" V 5075 5800 40  0000 C CNN
F 2 "bebl_v2_pretty:PIN_ARRAY-3X1" H 5025 5650 60  0001 C CNN
F 3 "" H 5025 5650 60  0000 C CNN
	1    5025 5800
	0    1    -1   0   
$EndComp
$EndSCHEMATC