EESchema Schematic File Version 4
LIBS:cicada-micro-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Drone flight controller - Cicada Micro"
Date "2024-04-16"
Rev "2"
Comp "anton.sysoev.ru68@gmail.com"
Comment1 ""
Comment2 "Anton Sysoev"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:C C3
U 1 1 5FF0A617
P 8550 2400
F 0 "C3" H 8665 2446 50  0000 L CNN
F 1 "20 uF" H 8665 2355 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 8588 2250 50  0001 C CNN
F 3 "~" H 8550 2400 50  0001 C CNN
	1    8550 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 1900 8550 2250
$Comp
L power:GNDD #PWR017
U 1 1 5FF0A629
P 8550 2950
F 0 "#PWR017" H 8550 2700 50  0001 C CNN
F 1 "GNDD" H 8554 2795 50  0000 C CNN
F 2 "" H 8550 2950 50  0001 C CNN
F 3 "" H 8550 2950 50  0001 C CNN
	1    8550 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 2950 8550 2550
$Comp
L Regulator_Linear:LM1117-3.3 D1
U 1 1 5FF0A630
P 7900 1900
F 0 "D1" H 7900 2050 50  0000 C CNN
F 1 "LDL1117SR33" H 7600 1600 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 7900 1900 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm1117.pdf" H 7900 1900 50  0001 C CNN
	1    7900 1900
	-1   0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR015
U 1 1 5FF0A637
P 7300 2950
F 0 "#PWR015" H 7300 2700 50  0001 C CNN
F 1 "GNDD" H 7304 2795 50  0000 C CNN
F 2 "" H 7300 2950 50  0001 C CNN
F 3 "" H 7300 2950 50  0001 C CNN
	1    7300 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5FF0A63D
P 7300 2400
F 0 "C2" H 7415 2446 50  0000 L CNN
F 1 "20 uF" H 7415 2355 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 7338 2250 50  0001 C CNN
F 3 "~" H 7300 2400 50  0001 C CNN
	1    7300 2400
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR014
U 1 1 5FF0A644
P 7300 1800
F 0 "#PWR014" H 7300 1650 50  0001 C CNN
F 1 "+3V3" H 7315 1973 50  0000 C CNN
F 2 "" H 7300 1800 50  0001 C CNN
F 3 "" H 7300 1800 50  0001 C CNN
	1    7300 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR016
U 1 1 5FF0A64A
P 7900 2950
F 0 "#PWR016" H 7900 2700 50  0001 C CNN
F 1 "GNDD" H 7904 2795 50  0000 C CNN
F 2 "" H 7900 2950 50  0001 C CNN
F 3 "" H 7900 2950 50  0001 C CNN
	1    7900 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 2200 7900 2950
Wire Wire Line
	8200 1900 8550 1900
Wire Wire Line
	7300 1800 7300 1900
Wire Wire Line
	7300 1900 7600 1900
Wire Wire Line
	7300 2250 7300 1900
Connection ~ 7300 1900
Wire Wire Line
	7300 2550 7300 2950
$Comp
L RF_Module:ESP-07 U1
U 1 1 5FF1BA92
P 3350 2500
AR Path="/5FF1BA92" Ref="U1"  Part="1" 
AR Path="/5FEF9D0A/5FF1BA92" Ref="U?"  Part="1" 
F 0 "U1" H 3500 3300 50  0000 C CNN
F 1 "ESP-07" H 3700 1800 50  0000 C CNN
F 2 "custom:ESP-07-ThroughMount" H 3350 2500 50  0001 C CNN
F 3 "http://wiki.ai-thinker.com/_media/esp8266/esp8266_series_modules_user_manual_v1.1.pdf" H 3000 2600 50  0001 C CNN
	1    3350 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5FF1BA99
P 2650 1150
AR Path="/5FF1BA99" Ref="R4"  Part="1" 
AR Path="/5FEF9D0A/5FF1BA99" Ref="R?"  Part="1" 
F 0 "R4" H 2720 1196 50  0000 L CNN
F 1 "3k3" H 2720 1105 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2580 1150 50  0001 C CNN
F 3 "~" H 2650 1150 50  0001 C CNN
	1    2650 1150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5FF1BAA0
P 2350 1150
AR Path="/5FF1BAA0" Ref="R3"  Part="1" 
AR Path="/5FEF9D0A/5FF1BAA0" Ref="R?"  Part="1" 
F 0 "R3" H 2420 1196 50  0000 L CNN
F 1 "3k3" H 2420 1105 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2280 1150 50  0001 C CNN
F 3 "~" H 2350 1150 50  0001 C CNN
	1    2350 1150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 5FF1BAA7
P 4050 3150
AR Path="/5FF1BAA7" Ref="R6"  Part="1" 
AR Path="/5FEF9D0A/5FF1BAA7" Ref="R?"  Part="1" 
F 0 "R6" H 4120 3196 50  0000 L CNN
F 1 "3k3" H 4120 3105 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3980 3150 50  0001 C CNN
F 3 "~" H 4050 3150 50  0001 C CNN
	1    4050 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5FF1BAAE
P 3600 1150
AR Path="/5FF1BAAE" Ref="C1"  Part="1" 
AR Path="/5FEF9D0A/5FF1BAAE" Ref="C?"  Part="1" 
F 0 "C1" H 3715 1196 50  0000 L CNN
F 1 "100 nF" H 3650 1050 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3638 1000 50  0001 C CNN
F 3 "~" H 3600 1150 50  0001 C CNN
	1    3600 1150
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR05
U 1 1 5FF1BAB5
P 3600 1400
AR Path="/5FF1BAB5" Ref="#PWR05"  Part="1" 
AR Path="/5FEF9D0A/5FF1BAB5" Ref="#PWR?"  Part="1" 
F 0 "#PWR05" H 3600 1150 50  0001 C CNN
F 1 "GNDD" H 3604 1245 50  0000 C CNN
F 2 "" H 3600 1400 50  0001 C CNN
F 3 "" H 3600 1400 50  0001 C CNN
	1    3600 1400
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR04
U 1 1 5FF1BABB
P 3350 3400
AR Path="/5FF1BABB" Ref="#PWR04"  Part="1" 
AR Path="/5FEF9D0A/5FF1BABB" Ref="#PWR?"  Part="1" 
F 0 "#PWR04" H 3350 3150 50  0001 C CNN
F 1 "GNDD" H 3354 3245 50  0000 C CNN
F 2 "" H 3350 3400 50  0001 C CNN
F 3 "" H 3350 3400 50  0001 C CNN
	1    3350 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5FF1BAC1
P 4050 1150
AR Path="/5FF1BAC1" Ref="R5"  Part="1" 
AR Path="/5FEF9D0A/5FF1BAC1" Ref="R?"  Part="1" 
F 0 "R5" H 4120 1196 50  0000 L CNN
F 1 "3k3" H 4120 1105 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3980 1150 50  0001 C CNN
F 3 "~" H 4050 1150 50  0001 C CNN
	1    4050 1150
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR06
U 1 1 5FF1BAC8
P 4050 3400
AR Path="/5FF1BAC8" Ref="#PWR06"  Part="1" 
AR Path="/5FEF9D0A/5FF1BAC8" Ref="#PWR?"  Part="1" 
F 0 "#PWR06" H 4050 3150 50  0001 C CNN
F 1 "GNDD" H 4054 3245 50  0000 C CNN
F 2 "" H 4050 3400 50  0001 C CNN
F 3 "" H 4050 3400 50  0001 C CNN
	1    4050 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 1900 2650 1900
Wire Wire Line
	2650 1900 2650 1300
Wire Wire Line
	2750 2100 2350 2100
Wire Wire Line
	2350 2100 2350 1300
Wire Wire Line
	3600 1300 3600 1400
Wire Wire Line
	2350 1000 2350 900 
Wire Wire Line
	2350 900  2650 900 
Wire Wire Line
	2650 900  2650 1000
Wire Wire Line
	2650 900  3350 900 
Wire Wire Line
	3600 900  3600 1000
Connection ~ 2650 900 
Wire Wire Line
	3350 1700 3350 900 
Connection ~ 3350 900 
Wire Wire Line
	3350 900  3600 900 
Wire Wire Line
	3600 900  4050 900 
Wire Wire Line
	4050 900  4050 1000
Connection ~ 3600 900 
Wire Wire Line
	3950 2100 4050 2100
Wire Wire Line
	4050 2100 4050 1300
Wire Wire Line
	3950 2800 4050 2800
Wire Wire Line
	4050 2800 4050 3000
Wire Wire Line
	3350 3200 3350 3400
Wire Wire Line
	4050 3300 4050 3400
$Comp
L power:+3V3 #PWR03
U 1 1 5FF1BAE5
P 3350 800
AR Path="/5FF1BAE5" Ref="#PWR03"  Part="1" 
AR Path="/5FEF9D0A/5FF1BAE5" Ref="#PWR?"  Part="1" 
F 0 "#PWR03" H 3350 650 50  0001 C CNN
F 1 "+3V3" H 3365 973 50  0000 C CNN
F 2 "" H 3350 800 50  0001 C CNN
F 3 "" H 3350 800 50  0001 C CNN
	1    3350 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 800  3350 900 
Entry Wire Line
	4950 2000 5050 2100
Entry Wire Line
	4950 2200 5050 2300
Entry Wire Line
	4950 1900 5050 2000
Wire Wire Line
	3950 2000 4950 2000
Wire Wire Line
	3950 2200 4950 2200
Text Label 4900 2200 2    50   ~ 0
RXD
Text Label 4900 2000 2    50   ~ 0
TXD
Entry Wire Line
	4950 2300 5050 2400
Entry Wire Line
	4950 2400 5050 2500
Text Label 4900 2400 2    50   ~ 0
SCL
Text Label 4900 2300 2    50   ~ 0
SDA
Entry Wire Line
	4950 2500 5050 2600
Entry Wire Line
	4950 2600 5050 2700
Entry Wire Line
	4950 2700 5050 2800
Entry Wire Line
	4950 2900 5050 3000
Wire Wire Line
	3950 2500 4950 2500
Wire Wire Line
	3950 2600 4950 2600
Wire Wire Line
	3950 2700 4950 2700
Wire Wire Line
	3950 2900 4950 2900
Entry Wire Line
	4950 2800 5050 2900
Entry Wire Line
	4950 2100 5050 2200
Wire Wire Line
	4050 2800 4950 2800
Connection ~ 4050 2800
Wire Wire Line
	4050 2100 4950 2100
Connection ~ 4050 2100
Text Label 4900 2900 2    50   ~ 0
M4
Text Label 4900 1900 2    50   ~ 0
M3
Text Label 4900 2100 2    50   ~ 0
M2
Text Label 4900 2800 2    50   ~ 0
M1
$Comp
L Device:R R7
U 1 1 5FF20D01
P 5550 1400
F 0 "R7" H 5620 1446 50  0000 L CNN
F 1 "3k3" H 5620 1355 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5480 1400 50  0001 C CNN
F 3 "~" H 5550 1400 50  0001 C CNN
	1    5550 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R8
U 1 1 5FF20D08
P 5550 2400
F 0 "R8" H 5620 2446 50  0000 L CNN
F 1 "3k3" H 5620 2355 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5480 2400 50  0001 C CNN
F 3 "~" H 5550 2400 50  0001 C CNN
	1    5550 2400
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR09
U 1 1 5FF20D0F
P 5550 2650
F 0 "#PWR09" H 5550 2400 50  0001 C CNN
F 1 "GNDD" H 5554 2495 50  0000 C CNN
F 2 "" H 5550 2650 50  0001 C CNN
F 3 "" H 5550 2650 50  0001 C CNN
	1    5550 2650
	1    0    0    -1  
$EndComp
$Comp
L Jumper:Jumper_3_Bridged12 JP1
U 1 1 5FF20D20
P 5550 1900
F 0 "JP1" V 5550 1967 50  0000 L CNN
F 1 "Jumper_3_Bridged12" H 5595 1967 50  0001 L CNN
F 2 "Jumper:SolderJumper-3_P1.3mm_Open_Pad1.0x1.5mm" H 5550 1900 50  0001 C CNN
F 3 "~" H 5550 1900 50  0001 C CNN
	1    5550 1900
	0    1    1    0   
$EndComp
Text Notes 5650 1750 0    50   ~ 0
Norm
Text Notes 5650 2100 0    50   ~ 0
Prog
Wire Wire Line
	5550 2550 5550 2650
$Comp
L power:GNDD #PWR011
U 1 1 5FF20D30
P 5550 3950
F 0 "#PWR011" H 5550 3700 50  0001 C CNN
F 1 "GNDD" H 5554 3795 50  0000 C CNN
F 2 "" H 5550 3950 50  0001 C CNN
F 3 "" H 5550 3950 50  0001 C CNN
	1    5550 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 3300 5550 3300
Wire Wire Line
	5550 3300 5550 3150
$Comp
L power:+3V3 #PWR08
U 1 1 5FF20D3A
P 5550 1150
F 0 "#PWR08" H 5550 1000 50  0001 C CNN
F 1 "+3V3" H 5565 1323 50  0000 C CNN
F 2 "" H 5550 1150 50  0001 C CNN
F 3 "" H 5550 1150 50  0001 C CNN
	1    5550 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 1250 5550 1150
Wire Wire Line
	5550 1650 5550 1550
Wire Wire Line
	5550 2250 5550 2150
Entry Wire Line
	5050 2000 5150 1900
Entry Wire Line
	5050 3500 5150 3400
Entry Wire Line
	5050 3600 5150 3500
Text Label 5200 1900 0    50   ~ 0
M3
Text Label 5200 3400 0    50   ~ 0
TXD
Text Label 5200 3500 0    50   ~ 0
RXD
Entry Wire Line
	5050 4950 5150 4850
Entry Wire Line
	5050 5050 5150 4950
Text Label 5200 4850 0    50   ~ 0
SCL
Text Label 5200 4950 0    50   ~ 0
SDA
Entry Wire Line
	5050 5650 5150 5550
Entry Wire Line
	5050 5750 5150 5650
Entry Wire Line
	5050 5850 5150 5750
Entry Wire Line
	5050 5950 5150 5850
Wire Wire Line
	5150 5550 5600 5550
Wire Wire Line
	5150 5650 5600 5650
Wire Wire Line
	5150 5750 5600 5750
Wire Wire Line
	5150 5850 5600 5850
Text Label 5200 5550 0    50   ~ 0
M4
Text Label 5200 5650 0    50   ~ 0
M3
Text Label 5200 5750 0    50   ~ 0
M2
Text Label 5200 5850 0    50   ~ 0
M1
$Comp
L power:+3V3 #PWR012
U 1 1 63A2FD35
P 5550 4450
F 0 "#PWR012" H 5550 4300 50  0001 C CNN
F 1 "+3V3" H 5550 4600 50  0000 C CNN
F 2 "" H 5550 4450 50  0001 C CNN
F 3 "" H 5550 4450 50  0001 C CNN
	1    5550 4450
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR013
U 1 1 63D5FF8B
P 5950 5150
F 0 "#PWR013" H 5950 4900 50  0001 C CNN
F 1 "GNDD" H 5954 4995 50  0000 C CNN
F 2 "" H 5950 5150 50  0001 C CNN
F 3 "" H 5950 5150 50  0001 C CNN
	1    5950 5150
	0    -1   -1   0   
$EndComp
$Comp
L power:+BATT #PWR07
U 1 1 63D5FFE8
P 5300 5350
F 0 "#PWR07" H 5300 5200 50  0001 C CNN
F 1 "+BATT" H 5300 5500 50  0000 C CNN
F 2 "" H 5300 5350 50  0001 C CNN
F 3 "" H 5300 5350 50  0001 C CNN
	1    5300 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 5450 5300 5450
Wire Wire Line
	5300 5450 5300 5350
Wire Wire Line
	5600 5350 5550 5350
Wire Wire Line
	5550 5350 5550 5150
Wire Wire Line
	5550 5150 5950 5150
Connection ~ 5550 5150
$Comp
L power:PWR_FLAG #FLG02
U 1 1 63D91D66
P 10250 5900
F 0 "#FLG02" H 10250 5975 50  0001 C CNN
F 1 "PWR_FLAG" H 10250 6074 50  0000 C CNN
F 2 "" H 10250 5900 50  0001 C CNN
F 3 "~" H 10250 5900 50  0001 C CNN
	1    10250 5900
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR022
U 1 1 63DA18DB
P 10750 6150
F 0 "#PWR022" H 10750 5900 50  0001 C CNN
F 1 "GNDD" H 10750 6000 50  0000 C CNN
F 2 "" H 10750 6150 50  0001 C CNN
F 3 "" H 10750 6150 50  0001 C CNN
	1    10750 6150
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG03
U 1 1 63DAEF47
P 10750 5900
F 0 "#FLG03" H 10750 5975 50  0001 C CNN
F 1 "PWR_FLAG" H 10750 6074 50  0000 C CNN
F 2 "" H 10750 5900 50  0001 C CNN
F 3 "~" H 10750 5900 50  0001 C CNN
	1    10750 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	10750 5900 10750 6150
$Comp
L power:+BATT #PWR020
U 1 1 63DC81BC
P 10250 6150
F 0 "#PWR020" H 10250 6000 50  0001 C CNN
F 1 "+BATT" H 10250 6300 50  0000 C CNN
F 2 "" H 10250 6150 50  0001 C CNN
F 3 "" H 10250 6150 50  0001 C CNN
	1    10250 6150
	-1   0    0    1   
$EndComp
Wire Wire Line
	10250 5900 10250 6150
Wire Wire Line
	3950 2300 4950 2300
Wire Wire Line
	3950 2400 4950 2400
Wire Wire Line
	5600 4550 5550 4550
Wire Wire Line
	5550 4550 5550 4450
NoConn ~ 5600 4650
Wire Wire Line
	5550 4750 5600 4750
$Comp
L Connector_Generic:Conn_01x05 X2
U 1 1 658F513F
P 5800 4750
F 0 "X2" H 5800 5050 50  0000 C CNN
F 1 "GY-91 Module" V 5950 4750 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 5800 4750 50  0001 C CNN
F 3 "~" H 5800 4750 50  0001 C CNN
	1    5800 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 4850 5600 4850
Wire Wire Line
	5150 4950 5600 4950
Wire Wire Line
	5150 3400 5600 3400
Wire Wire Line
	5150 3500 5600 3500
Wire Wire Line
	4950 1900 3950 1900
Wire Wire Line
	5150 1900 5400 1900
$Comp
L Device:R R1
U 1 1 6591A968
P 2000 2100
F 0 "R1" H 2070 2146 50  0000 L CNN
F 1 "390k" H 2070 2055 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1930 2100 50  0001 C CNN
F 3 "~" H 2000 2100 50  0001 C CNN
	1    2000 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 6591AA56
P 2000 2500
F 0 "R2" H 2070 2546 50  0000 L CNN
F 1 "30k" H 2070 2455 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 1930 2500 50  0001 C CNN
F 3 "~" H 2000 2500 50  0001 C CNN
	1    2000 2500
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR02
U 1 1 6591AB58
P 2000 2750
AR Path="/6591AB58" Ref="#PWR02"  Part="1" 
AR Path="/5FEF9D0A/6591AB58" Ref="#PWR?"  Part="1" 
F 0 "#PWR02" H 2000 2500 50  0001 C CNN
F 1 "GNDD" H 2004 2595 50  0000 C CNN
F 2 "" H 2000 2750 50  0001 C CNN
F 3 "" H 2000 2750 50  0001 C CNN
	1    2000 2750
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR01
U 1 1 6591ABAE
P 2000 1850
F 0 "#PWR01" H 2000 1700 50  0001 C CNN
F 1 "+BATT" H 2015 2023 50  0000 C CNN
F 2 "" H 2000 1850 50  0001 C CNN
F 3 "" H 2000 1850 50  0001 C CNN
	1    2000 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 2750 2000 2650
Wire Wire Line
	2000 2350 2000 2300
Wire Wire Line
	2000 1950 2000 1850
Wire Wire Line
	2000 2300 2750 2300
Connection ~ 2000 2300
Wire Wire Line
	2000 2300 2000 2250
Wire Wire Line
	5550 4750 5550 5150
$Comp
L power:+BATT #PWR0101
U 1 1 658F0590
P 9050 1600
F 0 "#PWR0101" H 9050 1450 50  0001 C CNN
F 1 "+BATT" H 9050 1750 50  0000 C CNN
F 2 "" H 9050 1600 50  0001 C CNN
F 3 "" H 9050 1600 50  0001 C CNN
	1    9050 1600
	1    0    0    -1  
$EndComp
Text Label 4900 2600 2    50   ~ 0
TX2
Text Label 4900 2500 2    50   ~ 0
RX2
Entry Wire Line
	5050 6600 5150 6500
Entry Wire Line
	5050 6700 5150 6600
Wire Wire Line
	5600 6500 5150 6500
Wire Wire Line
	5600 6600 5150 6600
Text Label 5200 6600 0    50   ~ 0
RX2
Text Label 5200 6500 0    50   ~ 0
TX2
NoConn ~ 5600 5950
Text Notes 5350 5950 0    50   ~ 0
CURN
$Comp
L Connector_Generic:Conn_01x02 X6
U 1 1 661E5843
P 9700 2250
F 0 "X6" H 9700 2350 50  0000 C CNN
F 1 "Ext Pwr 5V" H 10000 2200 50  0000 C CNN
F 2 "Connector_Molex:Molex_PicoBlade_53047-0410_1x02_P1.25mm_Vertical" H 9700 2250 50  0001 C CNN
F 3 "~" H 9700 2250 50  0001 C CNN
	1    9700 2250
	1    0    0    -1  
$EndComp
$Comp
L Jumper:Jumper_3_Bridged12 JP2
U 1 1 661E5BFD
P 9050 1900
F 0 "JP2" V 9050 1967 50  0000 L CNN
F 1 "Jumper_3_Bridged12" H 9095 1967 50  0001 L CNN
F 2 "Jumper:SolderJumper-3_P1.3mm_Open_Pad1.0x1.5mm" H 9050 1900 50  0001 C CNN
F 3 "~" H 9050 1900 50  0001 C CNN
	1    9050 1900
	0    1    1    0   
$EndComp
$Comp
L power:GNDD #PWR0102
U 1 1 661E60E9
P 9400 2950
F 0 "#PWR0102" H 9400 2700 50  0001 C CNN
F 1 "GNDD" H 9404 2795 50  0000 C CNN
F 2 "" H 9400 2950 50  0001 C CNN
F 3 "" H 9400 2950 50  0001 C CNN
	1    9400 2950
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0103
U 1 1 661E6299
P 9400 1600
F 0 "#PWR0103" H 9400 1450 50  0001 C CNN
F 1 "+5V" H 9400 1750 50  0000 C CNN
F 2 "" H 9400 1600 50  0001 C CNN
F 3 "" H 9400 1600 50  0001 C CNN
	1    9400 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 1900 8550 1900
Connection ~ 8550 1900
Wire Wire Line
	9500 2250 9400 2250
Wire Wire Line
	9050 2250 9050 2150
Wire Wire Line
	9400 1600 9400 2250
Connection ~ 9400 2250
Wire Wire Line
	9400 2250 9050 2250
Wire Wire Line
	9400 2950 9400 2350
Wire Wire Line
	9400 2350 9500 2350
$Comp
L power:+5V #PWR0104
U 1 1 661ED811
P 9750 6150
F 0 "#PWR0104" H 9750 6000 50  0001 C CNN
F 1 "+5V" H 9750 6300 50  0000 C CNN
F 2 "" H 9750 6150 50  0001 C CNN
F 3 "" H 9750 6150 50  0001 C CNN
	1    9750 6150
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 661ED9D4
P 9750 5900
F 0 "#FLG0101" H 9750 5975 50  0001 C CNN
F 1 "PWR_FLAG" H 9750 6074 50  0000 C CNN
F 2 "" H 9750 5900 50  0001 C CNN
F 3 "~" H 9750 5900 50  0001 C CNN
	1    9750 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9750 6150 9750 5900
Wire Wire Line
	9050 1650 9050 1600
Text Notes 7000 5250 0    98   ~ 0
1. Sold pin1+pin2 of JP2 to power from 1S LiPo\n2. Sold pin2+pin3 of JP2 to power from external 5V\n3. Don't mount LM1117 or AMS1117 - they have big\n    dropout voltage\n4. Mount LDL1117SR33 from STMicroelectronics\n5. Don't mount pull up resistors bigger 3k3\n    it causes a problem to upload firmware by UART
$Comp
L Connector_Generic:Conn_01x06 X1
U 1 1 661F5D2B
P 5800 3500
F 0 "X1" H 5800 3800 50  0000 C CNN
F 1 "GPS" H 5800 3100 50  0000 C CNN
F 2 "Connector_Molex:Molex_PicoBlade_53047-1210_1x06_P1.25mm_Vertical" H 5800 3500 50  0001 C CNN
F 3 "~" H 5800 3500 50  0001 C CNN
	1    5800 3500
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0105
U 1 1 661F5DD8
P 5550 3150
F 0 "#PWR0105" H 5550 3000 50  0001 C CNN
F 1 "+5V" H 5550 3300 50  0000 C CNN
F 2 "" H 5550 3150 50  0001 C CNN
F 3 "" H 5550 3150 50  0001 C CNN
	1    5550 3150
	1    0    0    -1  
$EndComp
Entry Wire Line
	5050 3700 5150 3600
Entry Wire Line
	5050 3800 5150 3700
Wire Wire Line
	5600 3600 5150 3600
Wire Wire Line
	5600 3700 5150 3700
Text Label 5200 3600 0    50   ~ 0
SCL
Text Label 5200 3700 0    50   ~ 0
SDA
Wire Wire Line
	5550 3950 5550 3800
Wire Wire Line
	5550 3800 5600 3800
$Comp
L Connector_Generic:Conn_01x08 X3
U 1 1 6620290F
P 5800 5650
F 0 "X3" H 5800 6050 50  0000 C CNN
F 1 "ESC" H 5800 5100 50  0000 C CNN
F 2 "Connector_Molex:Molex_PicoBlade_53047-1610_1x08_P1.25mm_Vertical" H 5800 5650 50  0001 C CNN
F 3 "~" H 5800 5650 50  0001 C CNN
	1    5800 5650
	1    0    0    -1  
$EndComp
Entry Wire Line
	5050 6150 5150 6050
Wire Wire Line
	5600 6050 5150 6050
Text Label 4900 2700 2    50   ~ 0
OESERVO
Text Label 5200 6050 0    50   ~ 0
OESERVO
$Comp
L Connector_Generic:Conn_01x02 X4
U 1 1 66205E0B
P 5800 6500
F 0 "X4" H 5800 6600 50  0000 C CNN
F 1 "Runcam" H 5800 6300 50  0000 C CNN
F 2 "Connector_Molex:Molex_PicoBlade_53047-0410_1x02_P1.25mm_Vertical" H 5800 6500 50  0001 C CNN
F 3 "~" H 5800 6500 50  0001 C CNN
	1    5800 6500
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 X5
U 1 1 66206544
P 5800 7250
F 0 "X5" H 5800 7450 50  0000 C CNN
F 1 "Servo" H 5800 7050 50  0000 C CNN
F 2 "Connector_Molex:Molex_PicoBlade_53047-0610_1x03_P1.25mm_Vertical" H 5800 7250 50  0001 C CNN
F 3 "~" H 5800 7250 50  0001 C CNN
	1    5800 7250
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0106
U 1 1 662069AF
P 5550 7000
F 0 "#PWR0106" H 5550 6850 50  0001 C CNN
F 1 "+5V" H 5550 7150 50  0000 C CNN
F 2 "" H 5550 7000 50  0001 C CNN
F 3 "" H 5550 7000 50  0001 C CNN
	1    5550 7000
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR0107
U 1 1 66208A22
P 5550 7450
F 0 "#PWR0107" H 5550 7200 50  0001 C CNN
F 1 "GNDD" H 5554 7295 50  0000 C CNN
F 2 "" H 5550 7450 50  0001 C CNN
F 3 "" H 5550 7450 50  0001 C CNN
	1    5550 7450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 7350 5550 7350
Wire Wire Line
	5550 7350 5550 7450
Entry Wire Line
	5050 7350 5150 7250
Text Label 5200 7250 0    50   ~ 0
OESERVO
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 6621A76B
P 8550 1600
F 0 "#FLG0102" H 8550 1675 50  0001 C CNN
F 1 "PWR_FLAG" H 8550 1750 50  0000 C CNN
F 2 "" H 8550 1600 50  0001 C CNN
F 3 "~" H 8550 1600 50  0001 C CNN
	1    8550 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 1600 8550 1900
Wire Wire Line
	5150 7250 5600 7250
Wire Wire Line
	5600 7150 5550 7150
Wire Wire Line
	5550 7150 5550 7000
Wire Bus Line
	5050 1900 5050 7400
$EndSCHEMATC