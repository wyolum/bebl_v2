EESchema Schematic File Version 2
LIBS:bebl_v2
LIBS:power
LIBS:device
LIBS:bebl_v2-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 2
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
	1075 1450 1075 800 
Connection ~ 3775 800 
Wire Wire Line
	3775 725  3775 1200
Connection ~ 1075 4975
Connection ~ 1250 2450
Wire Wire Line
	1250 4975 1250 1450
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
	1500 2450 1250 2450
Connection ~ 2350 4975
Wire Wire Line
	1250 800  1250 1000
Connection ~ 1250 800 
Wire Wire Line
	1075 4650 1075 4975
Connection ~ 1250 4975
Wire Wire Line
	3975 800  3975 1200
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
	975  800  3975 800 
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
	1075 1950 1075 4050
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
F 2 "ArthurC Lib:c_0805" H 700 3350 60  0001 C CNN
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
F 2 "ArthurC Lib:c_0805" H 700 2750 60  0001 C CNN
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
L CONN_2 P4
U 1 1 4E1ED2A7
P 7450 2925
F 0 "P4" H 7450 3025 50  0000 C CNN
F 1 "LiPo" H 7450 3125 50  0000 C CNN
F 2 "ArthurC Lib:pin_strip_2" H 7450 2925 60  0001 C CNN
F 3 "" H 7450 2925 60  0001 C CNN
	1    7450 2925
	-1   0    0    -1  
$EndComp
Text Label 1650 7100 0    60   ~ 0
3V3
Text Notes 6200 6725 0    60   Italic 12
Mounting Hole
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
F 2 "ArthurC Lib:ISP" H 1250 7050 60  0001 C CNN
F 3 "" H 1250 7050 60  0001 C CNN
	1    1250 7050
	1    0    0    -1  
$EndComp
$Comp
L SCREW SC1
U 1 1 4DCCE748
P 6525 6850
F 0 "SC1" H 6525 6750 40  0000 C CNN
F 1 "SCREW" H 6525 6950 40  0001 C CNN
F 2 "ArthurC Lib:vite_3mm" H 6525 6850 60  0001 C CNN
F 3 "" H 6525 6850 60  0001 C CNN
	1    6525 6850
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
F 2 "ArthurC Lib:PIN_ARRAY-6X1" H 2900 7050 60  0001 C CNN
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
P 8250 3500
F 0 "#PWR03" H 8250 3500 30  0001 C CNN
F 1 "GND" H 8250 3430 30  0001 C CNN
F 2 "" H 8250 3500 60  0001 C CNN
F 3 "" H 8250 3500 60  0001 C CNN
	1    8250 3500
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 4DCAC172
P 3975 1450
F 0 "R3" H 4050 1250 50  0000 C CNN
F 1 "10k" V 3975 1450 50  0000 C CNN
F 2 "ArthurC Lib:r_0805" H 3975 1450 60  0001 C CNN
F 3 "" H 3975 1450 60  0001 C CNN
	1    3975 1450
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 4DCAC161
P 3775 1450
F 0 "R2" H 3850 1250 50  0000 C CNN
F 1 "10k" V 3775 1450 50  0000 C CNN
F 2 "ArthurC Lib:r_0805" H 3775 1450 60  0001 C CNN
F 3 "" H 3775 1450 60  0001 C CNN
	1    3775 1450
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW1
U 1 1 4DCA7D67
P 1075 4350
F 0 "SW1" V 1075 4550 50  0000 C CNN
F 1 "RST" V 975 4550 50  0000 C CNN
F 2 "ArthurC Lib:SW_SMD" H 1075 4350 60  0001 C CNN
F 3 "" H 1075 4350 60  0001 C CNN
F 4 "" H 1325 4560 60  0001 C CNN "DigiKey"
	1    1075 4350
	0    -1   -1   0   
$EndComp
$Comp
L R R1
U 1 1 4DCA7BFC
P 1075 1700
F 0 "R1" H 1125 1900 50  0000 C CNN
F 1 "10k" V 1075 1700 50  0000 C CNN
F 2 "ArthurC Lib:r_0805" H 1075 1700 60  0001 C CNN
F 3 "" H 1075 1700 60  0001 C CNN
	1    1075 1700
	1    0    0    -1  
$EndComp
$Comp
L C_NPOL C3
U 1 1 4DCA7BAC
P 1250 1225
F 0 "C3" H 1300 1350 50  0000 L CNN
F 1 "100n" H 1300 1075 50  0000 L CNN
F 2 "ArthurC Lib:c_0805" H 1250 1225 60  0001 C CNN
F 3 "" H 1250 1225 60  0001 C CNN
	1    1250 1225
	1    0    0    -1  
$EndComp
Text Label 3450 2650 0    40   ~ 0
SDA
Text Label 3450 2750 0    40   ~ 0
SCL
Text Label 3450 4250 0    40   ~ 0
SS
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
F 2 "ArthurC Lib:TQFP32" H 2925 1675 50  0001 C CNN
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
F 2 "ArthurC Lib:Xtal_SMD4" H 900 3050 60  0001 C CNN
F 3 "" H 900 3050 60  0000 C CNN
	1    900  3050
	0    1    1    0   
$EndComp
Wire Wire Line
	1500 2550 1400 2550
Wire Wire Line
	3775 1700 3775 2650
Wire Wire Line
	3775 2650 3400 2650
Wire Wire Line
	3400 2750 3975 2750
Wire Wire Line
	3975 2750 3975 1700
$Comp
L TPS61221DCK U4
U 1 1 5415CD74
P 9650 2600
F 0 "U4" H 9650 2600 50  0000 C CNN
F 1 "TPS61221DCK" H 9650 2300 50  0000 C CNN
F 2 "" H 9650 2600 60  0000 C CNN
F 3 "" H 9650 2600 60  0000 C CNN
	1    9650 2600
	1    0    0    -1  
$EndComp
$Comp
L INDUCTOR L1
U 1 1 5415D384
P 8625 2450
F 0 "L1" V 8575 2450 50  0000 C CNN
F 1 "4u7" V 8725 2450 50  0000 C CNN
F 2 "" H 8625 2450 60  0000 C CNN
F 3 "" H 8625 2450 60  0000 C CNN
	1    8625 2450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8250 2750 9050 2750
Wire Wire Line
	8250 2450 8250 2875
Wire Wire Line
	8250 2600 9050 2600
Wire Wire Line
	7925 2450 8325 2450
Connection ~ 8250 2600
Wire Wire Line
	8925 2450 9050 2450
Wire Wire Line
	10250 2450 10700 2450
Wire Wire Line
	10325 2450 10325 2600
Wire Wire Line
	10325 2600 10250 2600
$Comp
L C_POL C5
U 1 1 5415D639
P 8250 3075
F 0 "C5" H 8150 3175 50  0000 L CNN
F 1 "10uF" H 8250 2925 50  0000 L CNN
F 2 "" H 8250 3075 60  0000 C CNN
F 3 "" H 8250 3075 60  0000 C CNN
	1    8250 3075
	1    0    0    -1  
$EndComp
$Comp
L C_POL C8
U 1 1 5415D64B
P 10475 3075
F 0 "C8" H 10375 3175 50  0000 L CNN
F 1 "10uF" H 10475 2925 50  0000 L CNN
F 2 "" H 10475 3075 60  0000 C CNN
F 3 "" H 10475 3075 60  0000 C CNN
	1    10475 3075
	1    0    0    -1  
$EndComp
Wire Wire Line
	10475 2450 10475 2875
Connection ~ 10325 2450
Wire Wire Line
	10250 2750 10325 2750
Wire Wire Line
	10325 2750 10325 3400
Wire Wire Line
	7925 3400 10475 3400
Wire Wire Line
	10475 3400 10475 3325
Wire Wire Line
	8250 3325 8250 3500
Connection ~ 10325 3400
Connection ~ 8250 2750
Connection ~ 8250 2450
Connection ~ 8250 3400
Connection ~ 10475 2450
Text Label 7925 3400 0    50   ~ 0
GND
Text Label 10700 2450 0    50   ~ 0
3V3
Text Label 7925 2450 0    50   ~ 0
BAT
Wire Wire Line
	7925 2825 7925 2450
Wire Wire Line
	7925 3025 7925 3400
Wire Wire Line
	7800 2825 7925 2825
Wire Wire Line
	7925 3025 7800 3025
Text Label 3500 800  0    50   ~ 0
3V3
$Comp
L MCP73831 U3
U 1 1 5415E477
P 9650 1125
F 0 "U3" H 9650 1125 50  0000 C CNN
F 1 "MCP73831" H 9650 825 50  0000 C CNN
F 2 "SOT-23-5" H 9650 1125 60  0001 C CNN
F 3 "" H 9650 1125 60  0000 C CNN
	1    9650 1125
	1    0    0    -1  
$EndComp
$Comp
L C_POL C6
U 1 1 5415E493
P 8500 1550
F 0 "C6" H 8400 1650 50  0000 L CNN
F 1 "4u7" H 8500 1400 50  0000 L CNN
F 2 "" H 8500 1550 60  0000 C CNN
F 3 "" H 8500 1550 60  0000 C CNN
	1    8500 1550
	1    0    0    -1  
$EndComp
$Comp
L DIODE_LED D2
U 1 1 5415E4A0
P 8800 1275
F 0 "D2" V 8700 1325 50  0000 C CNN
F 1 "CHG" V 8925 1200 40  0000 C CNN
F 2 "" H 8800 1275 60  0000 C CNN
F 3 "" H 8800 1275 60  0000 C CNN
	1    8800 1275
	0    1    1    0   
$EndComp
$Comp
L R2 R6
U 1 1 5415E4B4
P 8975 1575
F 0 "R6" H 8900 1750 50  0000 C CNN
F 1 "470" V 8975 1575 50  0000 C CNN
F 2 "" H 8975 1575 60  0000 C CNN
F 3 "" H 8975 1575 60  0000 C CNN
	1    8975 1575
	-1   0    0    1   
$EndComp
$Comp
L C_POL C10
U 1 1 5415E4EE
P 10750 1550
F 0 "C10" H 10775 1700 50  0000 L CNN
F 1 "4u7" H 10750 1400 50  0000 L CNN
F 2 "" H 10750 1550 60  0000 C CNN
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
F 2 "" H 10475 1575 60  0000 C CNN
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
	8800 900  8800 1050
Connection ~ 8800 975 
Wire Wire Line
	8500 1950 8500 1800
Wire Wire Line
	6925 1950 10750 1950
Wire Wire Line
	10750 1950 10750 1800
Wire Wire Line
	8975 1800 8975 1875
Wire Wire Line
	8975 1875 8800 1875
Wire Wire Line
	8800 1875 8800 1500
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
Vusb
Text Label 10400 975  2    50   ~ 0
BAT
$Comp
L USB-Micro P5
U 1 1 5415F287
P 7575 1275
F 0 "P5" H 7125 1625 50  0000 C CNN
F 1 "USB-Micro" H 6950 1500 50  0000 C CNN
F 2 "" H 7575 1275 60  0000 C CNN
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
F 2 "" H 2225 6775 60  0000 C CNN
F 3 "" H 2225 6775 60  0000 C CNN
	1    2225 6750
	0    1    1    0   
$EndComp
Text Label 1400 4975 0    50   ~ 0
GND
Text Label 5000 6225 0    40   ~ 0
GND
Wire Wire Line
	1400 2550 1400 2650
Connection ~ 1400 2650
$Comp
L ADXL345 U2
U 1 1 542E7DA5
P 9550 5125
F 0 "U2" H 9525 5675 50  0000 C CNN
F 1 "ADXL345" H 9550 4575 50  0000 C CNN
F 2 "ADXL345" H 9550 4775 60  0001 C CNN
F 3 "" H 9550 4775 60  0000 C CNN
	1    9550 5125
	-1   0    0    -1  
$EndComp
NoConn ~ 10100 4975
NoConn ~ 10100 5075
Wire Wire Line
	10175 5375 10175 5925
Wire Wire Line
	10100 5575 10175 5575
Connection ~ 10175 5575
Wire Wire Line
	10575 5475 10100 5475
Connection ~ 10175 5475
Wire Wire Line
	10100 5375 10175 5375
$Comp
L GND #PWR04
U 1 1 542E7DB5
P 10175 5925
F 0 "#PWR04" H 10175 5925 30  0001 C CNN
F 1 "GND" H 10175 5855 30  0001 C CNN
F 2 "" H 10175 5925 60  0001 C CNN
F 3 "" H 10175 5925 60  0001 C CNN
	1    10175 5925
	1    0    0    -1  
$EndComp
Wire Wire Line
	10175 4775 10100 4775
Wire Wire Line
	10175 4150 10175 4775
Wire Wire Line
	10300 4775 10300 4850
Wire Wire Line
	10100 4675 10575 4675
Connection ~ 10175 4675
$Comp
L C_NPOL C9
U 1 1 542E7DC9
P 10575 5075
F 0 "C9" H 10600 5200 50  0000 L CNN
F 1 "100nF" V 10625 4775 50  0000 L CNN
F 2 "c_0805" H 10575 5075 50  0001 C CNN
F 3 "" H 10575 5075 60  0001 C CNN
F 4 "CAP FILM 0.1UF 63VDC RADIAL" H 10575 5275 60  0001 L CNN "Field4"
F 5 "100nF, 63V" H 10575 5375 60  0001 L CNN "Field5"
F 6 "R82" H 10575 5475 60  0001 L CNN "Field6"
F 7 "Kemet" H 10575 5575 60  0001 L CNN "Field7"
F 8 "R82DC3100AA50J" H 10575 5675 60  0001 L CNN "Field8"
F 9 "Digikey" H 10575 5775 60  0001 L CNN "Field9"
F 10 "399-5863-ND" H 10575 5875 60  0001 L CNN "Field10"
F 11 "http://www.digikey.com/product-detail/en/R82DC3100AA50J/399-5863-ND/2571298?cur=USD" H 10575 5975 60  0001 L CNN "Field11"
	1    10575 5075
	1    0    0    -1  
$EndComp
$Comp
L C_NPOL C11
U 1 1 542E7DD7
P 10850 5075
F 0 "C11" H 10875 5200 50  0000 L CNN
F 1 "100nF" V 10900 4775 50  0000 L CNN
F 2 "c_0805" H 10850 5075 50  0001 C CNN
F 3 "" H 10850 5075 60  0001 C CNN
F 4 "CAP FILM 0.1UF 63VDC RADIAL" H 10850 5275 60  0001 L CNN "Field4"
F 5 "100nF, 63V" H 10850 5375 60  0001 L CNN "Field5"
F 6 "R82" H 10850 5475 60  0001 L CNN "Field6"
F 7 "Kemet" H 10850 5575 60  0001 L CNN "Field7"
F 8 "R82DC3100AA50J" H 10850 5675 60  0001 L CNN "Field8"
F 9 "Digikey" H 10850 5775 60  0001 L CNN "Field9"
F 10 "399-5863-ND" H 10850 5875 60  0001 L CNN "Field10"
F 11 "http://www.digikey.com/product-detail/en/R82DC3100AA50J/399-5863-ND/2571298?cur=USD" H 10850 5975 60  0001 L CNN "Field11"
	1    10850 5075
	1    0    0    -1  
$EndComp
$Comp
L C_POL C7
U 1 1 542E7DE5
P 10300 5050
F 0 "C7" H 10200 5150 50  0000 L CNN
F 1 "10uF" V 10350 4750 40  0000 L CNN
F 2 "C_ELCO_SMD" H 9900 5025 50  0001 C CNN
F 3 "" H 10300 5050 60  0001 C CNN
F 4 "CAP ALUM 100UF 16V 20% RADIAL" H 10300 5250 60  0001 L CNN "Field4"
F 5 "100u,16V" H 10300 5350 60  0001 L CNN "Field5"
F 6 "Radial, Can, 6.3mm dia" H 10300 5450 60  0001 L CNN "Field6"
F 7 "Panasonic Electronic Components" H 10300 5550 60  0001 L CNN "Field7"
F 8 "ECE-A1CKA101" H 10300 5650 60  0001 L CNN "Field8"
F 9 "Digikey" H 10300 5750 60  0001 L CNN "Field9"
F 10 "P833-ND" H 10300 5850 60  0001 L CNN "Field10"
F 11 "http://www.digikey.com/product-detail/en/ECE-A1CKA101/P833-ND/44757?cur=USD" H 10300 5950 60  0001 L CNN "Field11"
	1    10300 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 5300 10300 5375
Wire Wire Line
	10300 5375 10850 5375
Wire Wire Line
	10850 5375 10850 5300
Wire Wire Line
	10575 5300 10575 5475
Connection ~ 10575 5375
Wire Wire Line
	10850 4775 10850 4850
Wire Wire Line
	10300 4775 10850 4775
Wire Wire Line
	10575 4675 10575 4850
Connection ~ 10575 4775
Text Label 8725 4875 0    40   ~ 0
ACL_SDO
Wire Wire Line
	8300 5275 9000 5275
Wire Wire Line
	8650 5375 9000 5375
Wire Wire Line
	9000 5075 8725 5075
Wire Wire Line
	8725 4975 9000 4975
Wire Wire Line
	7950 4875 9000 4875
Wire Wire Line
	8675 4775 9000 4775
Text Label 8725 5075 0    40   ~ 0
SCL
Text Label 8725 5275 0    40   ~ 0
ACL_INT1
Text Label 8725 5375 0    40   ~ 0
ACL_INT2
Wire Wire Line
	7875 4150 10175 4150
Wire Wire Line
	8675 4775 8675 4150
Connection ~ 8675 4150
$Comp
L CONN_3 P6
U 1 1 542E7E0B
P 7950 6225
F 0 "P6" V 7900 6225 50  0000 C CNN
F 1 "Alt_Add" V 8000 6225 50  0000 C CNN
F 2 "ShortLink2" H 7950 6225 60  0001 C CNN
F 3 "" H 7950 6225 60  0000 C CNN
F 4 "CONN HEADER 50POS .100\" SGL GOLD" H 7950 6325 60  0001 L CNN "Field4"
F 5 "Header, Unshrouded, Male pin," H 7950 6425 60  0001 L CNN "Field5"
F 6 "0.1\" pitch x 50 nos" H 7950 6525 60  0001 L CNN "Field6"
F 7 "Samtec Inc" H 7950 6625 60  0001 L CNN "Field7"
F 8 "TSW-150-07-L-S" H 7950 6725 60  0001 L CNN "Field8"
F 9 "Digikey" H 7950 6825 60  0001 L CNN "Field9"
F 10 "SAM1031-50-ND" H 7950 6925 60  0001 L CNN "Field10"
F 11 "http://www.digikey.com/scripts/DkSearch/dksus.dll?WT.z_header=search_go&lang=en&keywords=SAM1031-50-ND&x=15&y=16&cur=USD" H 7950 7025 60  0001 L CNN "Field11"
	1    7950 6225
	0    1    1    0   
$EndComp
$Comp
L CONN_3 P7
U 1 1 542E7E19
P 8300 6225
F 0 "P7" V 8250 6225 50  0000 C CNN
F 1 "Int1" V 8350 6225 50  0000 C CNN
F 2 "ShortLink2" H 8300 6225 60  0001 C CNN
F 3 "" H 8300 6225 60  0000 C CNN
F 4 "CONN HEADER 50POS .100\" SGL GOLD" H 8300 6325 60  0001 L CNN "Field4"
F 5 "Header, Unshrouded, Male pin," H 8300 6425 60  0001 L CNN "Field5"
F 6 "0.1\" pitch x 50 nos" H 8300 6525 60  0001 L CNN "Field6"
F 7 "Samtec Inc" H 8300 6625 60  0001 L CNN "Field7"
F 8 "TSW-150-07-L-S" H 8300 6725 60  0001 L CNN "Field8"
F 9 "Digikey" H 8300 6825 60  0001 L CNN "Field9"
F 10 "SAM1031-50-ND" H 8300 6925 60  0001 L CNN "Field10"
F 11 "http://www.digikey.com/scripts/DkSearch/dksus.dll?WT.z_header=search_go&lang=en&keywords=SAM1031-50-ND&x=15&y=16&cur=USD" H 8300 7025 60  0001 L CNN "Field11"
	1    8300 6225
	0    1    1    0   
$EndComp
$Comp
L CONN_3 P8
U 1 1 542E7E27
P 8650 6225
F 0 "P8" V 8600 6225 50  0000 C CNN
F 1 "Int2" V 8700 6225 50  0000 C CNN
F 2 "ShortLink2" H 8650 6225 60  0001 C CNN
F 3 "" H 8650 6225 60  0000 C CNN
F 4 "CONN HEADER 50POS .100\" SGL GOLD" H 8650 6325 60  0001 L CNN "Field4"
F 5 "Header, Unshrouded, Male pin," H 8650 6425 60  0001 L CNN "Field5"
F 6 "0.1\" pitch x 50 nos" H 8650 6525 60  0001 L CNN "Field6"
F 7 "Samtec Inc" H 8650 6625 60  0001 L CNN "Field7"
F 8 "TSW-150-07-L-S" H 8650 6725 60  0001 L CNN "Field8"
F 9 "Digikey" H 8650 6825 60  0001 L CNN "Field9"
F 10 "SAM1031-50-ND" H 8650 6925 60  0001 L CNN "Field10"
F 11 "http://www.digikey.com/scripts/DkSearch/dksus.dll?WT.z_header=search_go&lang=en&keywords=SAM1031-50-ND&x=15&y=16&cur=USD" H 8650 7025 60  0001 L CNN "Field11"
	1    8650 6225
	0    1    1    0   
$EndComp
Wire Wire Line
	8650 5875 8650 5375
Wire Wire Line
	8300 5275 8300 5875
Wire Wire Line
	7950 5875 7950 4875
Wire Wire Line
	8550 5650 8550 5875
Wire Wire Line
	7850 5650 8550 5650
Wire Wire Line
	7850 5650 7850 5875
Wire Wire Line
	8200 4150 8200 5875
Connection ~ 8200 5650
Connection ~ 8200 4150
Wire Wire Line
	8050 5875 8050 5750
Wire Wire Line
	8050 5750 8925 5750
Wire Wire Line
	8750 5750 8750 5875
Wire Wire Line
	8400 5875 8400 5750
Connection ~ 8400 5750
Wire Wire Line
	8925 5750 8925 6350
Connection ~ 8750 5750
$Comp
L GND #PWR05
U 1 1 542E7E3D
P 8925 6350
F 0 "#PWR05" H 8925 6350 30  0001 C CNN
F 1 "GND" H 8925 6280 30  0001 C CNN
F 2 "" H 8925 6350 60  0001 C CNN
F 3 "" H 8925 6350 60  0001 C CNN
	1    8925 6350
	1    0    0    -1  
$EndComp
Text Label 8925 5750 2    40   ~ 0
GND
Text Label 10175 5775 0    50   ~ 0
GND
Text Label 8725 4975 0    40   ~ 0
SDA
Text Label 7875 4150 0    50   ~ 0
3V3
$Comp
L SCREW SC2
U 1 1 542E814F
P 6525 7050
F 0 "SC2" H 6525 6950 40  0000 C CNN
F 1 "SCREW" H 6525 7150 40  0001 C CNN
F 2 "ArthurC Lib:vite_3mm" H 6525 7050 60  0001 C CNN
F 3 "" H 6525 7050 60  0001 C CNN
	1    6525 7050
	1    0    0    -1  
$EndComp
$Comp
L SCREW SC3
U 1 1 542E815A
P 6525 7250
F 0 "SC3" H 6525 7150 40  0000 C CNN
F 1 "SCREW" H 6525 7350 40  0001 C CNN
F 2 "ArthurC Lib:vite_3mm" H 6525 7250 60  0001 C CNN
F 3 "" H 6525 7250 60  0001 C CNN
	1    6525 7250
	1    0    0    -1  
$EndComp
$Comp
L SCREW SC4
U 1 1 542E8165
P 6525 7450
F 0 "SC4" H 6525 7350 40  0000 C CNN
F 1 "SCREW" H 6525 7550 40  0001 C CNN
F 2 "ArthurC Lib:vite_3mm" H 6525 7450 60  0001 C CNN
F 3 "" H 6525 7450 60  0001 C CNN
	1    6525 7450
	1    0    0    -1  
$EndComp
NoConn ~ 10100 5275
$Comp
L NPN Q1
U 1 1 542EACD6
P 5000 5525
F 0 "Q1" H 5025 5325 50  0000 C CNN
F 1 "2N5088BU" V 5225 5525 40  0000 C CNN
F 2 "TO92-EBC" H 5190 5525 30  0001 C CNN
F 3 "" H 5000 5525 60  0000 C CNN
	1    5000 5525
	-1   0    0    -1  
$EndComp
$Comp
L MOSFET_N Q2
U 1 1 542EADCE
P 5300 5175
F 0 "Q2" H 5300 5400 50  0000 R CNN
F 1 "MGSF1N03L" V 5475 5350 40  0000 R CNN
F 2 "" H 5300 5175 60  0000 C CNN
F 3 "" H 5300 5175 60  0000 C CNN
	1    5300 5175
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 542EADE2
P 5400 5825
F 0 "R5" H 5450 5625 50  0000 C CNN
F 1 "Rled" V 5400 5825 50  0000 C CNN
F 2 "" H 5400 5825 60  0000 C CNN
F 3 "" H 5400 5825 60  0000 C CNN
	1    5400 5825
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 542EADF4
P 4475 4875
F 0 "R4" H 4550 4675 50  0000 C CNN
F 1 "22k" V 4475 4875 50  0000 C CNN
F 2 "" H 4475 4875 60  0000 C CNN
F 3 "" H 4475 4875 60  0000 C CNN
	1    4475 4875
	1    0    0    -1  
$EndComp
$Comp
L ZENER D1
U 1 1 542EAE1C
P 4475 5875
F 0 "D1" V 4575 5825 50  0000 C CNN
F 1 "ZENER_2V7" H 4325 5975 40  0000 C CNN
F 2 "" H 4475 5875 60  0000 C CNN
F 3 "" H 4475 5875 60  0000 C CNN
	1    4475 5875
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4475 5125 4475 5675
Wire Wire Line
	3950 5175 5100 5175
Connection ~ 4475 5175
Wire Wire Line
	4900 5325 4900 5175
Connection ~ 4900 5175
Wire Wire Line
	5400 5375 5400 5575
Wire Wire Line
	5200 5525 5400 5525
Connection ~ 5400 5525
Wire Wire Line
	5400 6225 5400 6075
Wire Wire Line
	4475 6225 5400 6225
Wire Wire Line
	4475 6225 4475 6075
Wire Wire Line
	4475 4625 4475 4550
Wire Wire Line
	4475 4550 5400 4550
Wire Wire Line
	4900 5725 4900 6225
Connection ~ 4900 6225
Text Label 4825 4550 0    50   ~ 0
3V3
$Comp
L CONN_2 P3
U 1 1 542EB92D
P 5750 4775
F 0 "P3" H 5750 4875 40  0000 C CNN
F 1 "LED" H 5750 4975 40  0000 C CNN
F 2 "" H 5750 4775 60  0000 C CNN
F 3 "" H 5750 4775 60  0000 C CNN
	1    5750 4775
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 4550 5400 4675
Wire Wire Line
	5400 4975 5400 4875
Text Notes 5850 6225 1    40   ~ 0
Rled = 0.5/Iled\nW(Rled) = 0.25/Rled\n5 Ohms for 100mA : 50mW\n1.56 Ohms for 320mA : 160mW\n\n
Wire Wire Line
	3950 3750 3950 5175
Text Notes 4500 4375 0    40   ~ 0
4x LED 80mA, 10mm, 12000mcd, (48000mcd, 320mA)\nor\n5x LED 20mA, 5mm, 9000mcd, (45000mcd, 100mA)
$Comp
L I/O B1
U 1 1 5431218A
P 3625 2250
F 0 "B1" H 3705 2250 40  0000 L CNN
F 1 "I/O" H 3625 2305 30  0001 C CNN
F 2 "" H 3625 2250 60  0000 C CNN
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
F 2 "" H 3625 2350 60  0000 C CNN
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
F 2 "" H 3625 2450 60  0000 C CNN
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
F 2 "" H 3625 2550 60  0000 C CNN
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
F 2 "" H 3625 2850 60  0000 C CNN
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
F 2 "" H 3625 2950 60  0000 C CNN
F 3 "" H 3625 2950 60  0000 C CNN
	1    3625 2950
	1    0    0    -1  
$EndComp
$Comp
L I/O B7
U 1 1 54312202
P 3625 3350
F 0 "B7" H 3705 3350 40  0000 L CNN
F 1 "I/O" H 3625 3405 30  0001 C CNN
F 2 "" H 3625 3350 60  0000 C CNN
F 3 "" H 3625 3350 60  0000 C CNN
	1    3625 3350
	1    0    0    -1  
$EndComp
$Comp
L I/O B8
U 1 1 54312216
P 3625 3450
F 0 "B8" H 3705 3450 40  0000 L CNN
F 1 "I/O" H 3625 3505 30  0001 C CNN
F 2 "" H 3625 3450 60  0000 C CNN
F 3 "" H 3625 3450 60  0000 C CNN
	1    3625 3450
	1    0    0    -1  
$EndComp
$Comp
L I/O B9
U 1 1 5431222A
P 3625 3550
F 0 "B9" H 3705 3550 40  0000 L CNN
F 1 "I/O" H 3625 3605 30  0001 C CNN
F 2 "" H 3625 3550 60  0000 C CNN
F 3 "" H 3625 3550 60  0000 C CNN
	1    3625 3550
	1    0    0    -1  
$EndComp
$Comp
L I/O B10
U 1 1 5431223E
P 3625 3850
F 0 "B10" H 3705 3850 40  0000 L CNN
F 1 "I/O" H 3625 3905 30  0001 C CNN
F 2 "" H 3625 3850 60  0000 C CNN
F 3 "" H 3625 3850 60  0000 C CNN
	1    3625 3850
	1    0    0    -1  
$EndComp
$Comp
L I/O B11
U 1 1 54312252
P 3625 4050
F 0 "B11" H 3705 4050 40  0000 L CNN
F 1 "I/O" H 3625 4105 30  0001 C CNN
F 2 "" H 3625 4050 60  0000 C CNN
F 3 "" H 3625 4050 60  0000 C CNN
	1    3625 4050
	1    0    0    -1  
$EndComp
$Comp
L I/O B12
U 1 1 54312266
P 3625 4150
F 0 "B12" H 3705 4150 40  0000 L CNN
F 1 "I/O" H 3625 4205 30  0001 C CNN
F 2 "" H 3625 4150 60  0000 C CNN
F 3 "" H 3625 4150 60  0000 C CNN
	1    3625 4150
	1    0    0    -1  
$EndComp
$Comp
L I/O B13
U 1 1 5431227A
P 3625 4250
F 0 "B13" H 3705 4250 40  0000 L CNN
F 1 "I/O" H 3625 4305 30  0001 C CNN
F 2 "" H 3625 4250 60  0000 C CNN
F 3 "" H 3625 4250 60  0000 C CNN
	1    3625 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 4150 3475 4150
Wire Wire Line
	3475 4050 3400 4050
Wire Wire Line
	3400 3550 3475 3550
Wire Wire Line
	3400 3850 3475 3850
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
Wire Wire Line
	3400 4250 3475 4250
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
	11000 600  11000 2100
Wire Notes Line
	11000 2100 6600 2100
Wire Notes Line
	6600 2100 6600 600 
Wire Notes Line
	6600 2200 11000 2200
Wire Notes Line
	11000 2200 11000 3600
Wire Notes Line
	11000 3600 6600 3600
Wire Notes Line
	6600 3600 6600 2200
Wire Notes Line
	6600 3800 11000 3800
Wire Notes Line
	11000 3800 11000 6500
Wire Notes Line
	11000 6500 6600 6500
Wire Notes Line
	6600 6500 6600 3800
Wire Notes Line
	6400 3800 4200 3800
Wire Notes Line
	4200 3800 4200 6500
Wire Notes Line
	4200 6500 6400 6500
Wire Notes Line
	6400 6500 6400 3800
Text Notes 4225 3900 0    50   Italic 10
LED DRIVER
Text Notes 6650 3900 0    50   Italic 10
ACCELEROMETER, I2C
Text Notes 6650 2325 0    50   Italic 10
VOLTAGE REGULATOR
Text Notes 6650 725  0    50   Italic 10
LiPo CHARGER
Wire Wire Line
	3400 3350 3475 3350
Text Label 10300 1125 0    50   ~ 0
PROG
Text Label 8975 2450 0    40   ~ 0
L
Text Label 4000 5175 0    50   ~ 0
LED
Wire Wire Line
	3400 3750 3950 3750
Wire Wire Line
	3475 3450 3400 3450
Wire Wire Line
	3400 3650 4450 3650
$Comp
L R R8
U 1 1 54315771
P 4700 3650
F 0 "R8" V 4780 3650 50  0000 C CNN
F 1 "56E" V 4700 3650 50  0000 C CNN
F 2 "" H 4700 3650 60  0000 C CNN
F 3 "" H 4700 3650 60  0000 C CNN
	1    4700 3650
	0    1    1    0   
$EndComp
$Comp
L DIODE_LED D3
U 1 1 5431586D
P 5425 3650
F 0 "D3" H 5425 3750 40  0000 C CNN
F 1 "IND" H 5425 3550 40  0000 C CNN
F 2 "" H 5425 3650 60  0000 C CNN
F 3 "" H 5425 3650 60  0000 C CNN
	1    5425 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 3650 4950 3650
Wire Wire Line
	5650 3650 6000 3650
Text Label 6000 3650 0    40   ~ 0
GND
Text Notes 5225 3450 0    40   ~ 0
1x LED 20mA, 5mm, 9000mcd\n(On Indicator, Drive Light DRL)
$Sheet
S 5850 7250 500  150 
U 543A53BD
F0 "LED_Driver" 60
F1 "LED_Driver.sch" 60
$EndSheet
$EndSCHEMATC
