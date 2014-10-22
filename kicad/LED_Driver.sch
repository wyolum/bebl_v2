EESchema Schematic File Version 2
LIBS:bebl_v2
LIBS:power
LIBS:device
LIBS:bebl_v2-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 2
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Label 3300 4075 0    40   ~ 0
GND
$Comp
L MOSFET_N Q201
U 1 1 543A5558
P 3600 3025
F 0 "Q201" H 3600 3250 50  0000 R CNN
F 1 "MGSF1N03L" V 3775 3200 40  0000 R CNN
F 2 "" H 3600 3025 60  0001 C CNN
F 3 "" H 3600 3025 60  0000 C CNN
	1    3600 3025
	1    0    0    -1  
$EndComp
$Comp
L R R202
U 1 1 543A555F
P 3700 3675
F 0 "R202" H 3800 3450 50  0000 C CNN
F 1 "Rled" V 3700 3675 50  0000 C CNN
F 2 "" H 3700 3675 60  0001 C CNN
F 3 "" H 3700 3675 60  0000 C CNN
	1    3700 3675
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 3225 3700 3425
Wire Wire Line
	3700 4075 3700 3925
Wire Wire Line
	1575 4075 3700 4075
Wire Wire Line
	1575 2100 3700 2100
Text Label 3125 2100 0    50   ~ 0
3V3
Text Label 5325 3275 0    40   ~ 0
GND
$Comp
L MOSFET_N Q202
U 1 1 543A56E5
P 7200 3075
F 0 "Q202" H 7200 3300 50  0000 R CNN
F 1 "MGSF1N03L" V 7375 3250 40  0000 R CNN
F 2 "" H 7200 3075 60  0001 C CNN
F 3 "" H 7200 3075 60  0000 C CNN
	1    7200 3075
	1    0    0    -1  
$EndComp
$Comp
L R R204
U 1 1 543A56EB
P 7300 3725
F 0 "R204" H 7400 3500 50  0000 C CNN
F 1 "Rled" V 7300 3725 50  0000 C CNN
F 2 "" H 7300 3725 60  0001 C CNN
F 3 "" H 7300 3725 60  0000 C CNN
	1    7300 3725
	1    0    0    -1  
$EndComp
$Comp
L R R203
U 1 1 543A56F1
P 6000 3075
F 0 "R203" V 5900 3075 50  0000 C CNN
F 1 "100R" V 6000 3075 50  0000 C CNN
F 2 "" H 6000 3075 60  0001 C CNN
F 3 "" H 6000 3075 60  0000 C CNN
	1    6000 3075
	0    1    1    0   
$EndComp
Wire Wire Line
	6250 3075 7000 3075
Wire Wire Line
	7300 3275 7300 3475
Wire Wire Line
	7300 4125 7300 3975
Wire Wire Line
	5650 4125 7300 4125
Wire Wire Line
	5650 2100 7300 2100
Text Label 5325 2875 0    50   ~ 0
3V3
Wire Wire Line
	7300 2100 7300 2250
Wire Wire Line
	7300 2875 7300 2700
$Comp
L DIODE_LED D201
U 1 1 543A5887
P 3700 2475
F 0 "D201" V 3575 2550 40  0000 C CNN
F 1 "DIODE_LED" V 3675 2750 40  0000 C CNN
F 2 "" H 3700 2475 60  0001 C CNN
F 3 "" H 3700 2475 60  0000 C CNN
	1    3700 2475
	0    1    1    0   
$EndComp
Wire Wire Line
	3700 2100 3700 2250
Wire Wire Line
	3700 2825 3700 2700
$Comp
L R R201
U 1 1 543A5A71
P 3100 3025
F 0 "R201" V 3000 3000 50  0000 C CNN
F 1 "100R" V 3100 3025 50  0000 C CNN
F 2 "bebl_v2_legacy:r_0805" H 3100 3025 60  0001 C CNN
F 3 "" H 3100 3025 60  0000 C CNN
	1    3100 3025
	0    1    1    0   
$EndComp
Wire Wire Line
	3350 3025 3400 3025
$Comp
L SWITCH_INV SW201
U 1 1 543A5C10
P 2225 3025
F 0 "SW201" H 2025 3175 50  0000 C CNN
F 1 "SWITCH" H 2075 2875 50  0000 C CNN
F 2 "" H 2225 3025 60  0001 C CNN
F 3 "" H 2225 3025 60  0000 C CNN
	1    2225 3025
	-1   0    0    1   
$EndComp
Wire Wire Line
	2725 3025 2850 3025
Wire Wire Line
	1725 2925 1575 2925
Wire Wire Line
	1575 2925 1575 2100
Wire Wire Line
	1725 3125 1575 3125
Wire Wire Line
	1575 3125 1575 4075
Text Notes 3800 3775 0    60   ~ 0
Between\n11 to 25 ohms
Text Notes 7350 3775 0    60   ~ 0
Between\n11 to 25 ohms
$Comp
L DIODE_LED D202
U 1 1 543A61D5
P 7300 2475
F 0 "D202" V 7175 2550 40  0000 C CNN
F 1 "DIODE_LED" V 7275 2750 40  0000 C CNN
F 2 "" H 7300 2475 60  0001 C CNN
F 3 "" H 7300 2475 60  0000 C CNN
	1    7300 2475
	0    1    1    0   
$EndComp
Wire Wire Line
	5650 2100 5650 2875
Wire Wire Line
	5650 2875 5325 2875
Wire Wire Line
	5325 3075 5750 3075
Wire Wire Line
	5325 3275 5650 3275
Wire Wire Line
	5650 3275 5650 4125
Text Label 5325 3075 0    50   ~ 0
ATMEGA
$EndSCHEMATC
