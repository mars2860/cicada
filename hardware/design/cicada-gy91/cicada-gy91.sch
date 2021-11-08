EESchema Schematic File Version 4
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Cicada breakout for GY91 module"
Date "2021-10-18"
Rev "1"
Comp ""
Comment1 ""
Comment2 "Anton Sysoev (anton.sysoev.ru68@gmail.com)"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector_Generic:Conn_01x08 J2
U 1 1 616D4E0F
P 5100 2300
F 0 "J2" H 5100 2700 50  0000 C CNN
F 1 "Expansion slot2" H 4900 1750 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x08_P2.54mm_Vertical" H 5100 2300 50  0001 C CNN
F 3 "~" H 5100 2300 50  0001 C CNN
	1    5100 2300
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x08 J1
U 1 1 616D4EA6
P 3700 2300
F 0 "J1" H 3700 2700 50  0000 C CNN
F 1 "Expansion slot1" H 3700 1750 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x08_P2.54mm_Vertical" H 3700 2300 50  0001 C CNN
F 3 "~" H 3700 2300 50  0001 C CNN
	1    3700 2300
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x08 J3
U 1 1 616D4F04
P 5100 3650
F 0 "J3" H 5100 4050 50  0000 C CNN
F 1 "GY-91 Module" H 4850 3100 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical" H 5100 3650 50  0001 C CNN
F 3 "~" H 5100 3650 50  0001 C CNN
	1    5100 3650
	1    0    0    -1  
$EndComp
Entry Wire Line
	4400 2100 4500 2000
Entry Wire Line
	4400 2200 4500 2100
Entry Wire Line
	4400 2300 4500 2200
Entry Wire Line
	4400 2400 4500 2300
Entry Wire Line
	4400 2500 4500 2400
Entry Wire Line
	4400 2600 4500 2500
Entry Wire Line
	4400 2700 4500 2600
Entry Wire Line
	4400 2800 4500 2700
Entry Wire Line
	4400 3450 4500 3350
Entry Wire Line
	4400 3550 4500 3450
Entry Wire Line
	4400 3650 4500 3550
Entry Wire Line
	4400 3750 4500 3650
Entry Wire Line
	4400 3850 4500 3750
Entry Wire Line
	4400 3950 4500 3850
Entry Wire Line
	4400 4050 4500 3950
Entry Wire Line
	4400 4150 4500 4050
Entry Wire Line
	4300 2000 4400 2100
Entry Wire Line
	4300 2100 4400 2200
Entry Wire Line
	4300 2200 4400 2300
Entry Wire Line
	4300 2300 4400 2400
Entry Wire Line
	4300 2400 4400 2500
Entry Wire Line
	4300 2500 4400 2600
Entry Wire Line
	4300 2600 4400 2700
Entry Wire Line
	4300 2700 4400 2800
Wire Wire Line
	3900 2000 4300 2000
Wire Wire Line
	3900 2100 4300 2100
Wire Wire Line
	3900 2200 4300 2200
Wire Wire Line
	3900 2300 4300 2300
Wire Wire Line
	3900 2400 4300 2400
Wire Wire Line
	3900 2500 4300 2500
Wire Wire Line
	3900 2600 4300 2600
Wire Wire Line
	3900 2700 4300 2700
Wire Wire Line
	4500 2000 4900 2000
Wire Wire Line
	4500 2100 4900 2100
Wire Wire Line
	4500 2200 4900 2200
Wire Wire Line
	4500 2300 4900 2300
Wire Wire Line
	4500 2400 4900 2400
Wire Wire Line
	4500 2500 4900 2500
Wire Wire Line
	4500 2600 4900 2600
Wire Wire Line
	4500 2700 4900 2700
Wire Wire Line
	4500 3350 4900 3350
Wire Wire Line
	4500 3450 4900 3450
Wire Wire Line
	4500 3550 4900 3550
Wire Wire Line
	4500 3650 4900 3650
Wire Wire Line
	4500 3750 4900 3750
Wire Wire Line
	4500 3850 4900 3850
Wire Wire Line
	4500 3950 4900 3950
Wire Wire Line
	4500 4050 4900 4050
Wire Bus Line
	4400 1900 4400 4200
Text Label 4200 2000 2    50   ~ 0
+BATT
Text Label 4200 2100 2    50   ~ 0
ADC
Text Label 4200 2200 2    50   ~ 0
+5V
Text Label 4200 2300 2    50   ~ 0
M4
Text Label 4200 2400 2    50   ~ 0
SCK
Text Label 4200 2500 2    50   ~ 0
MISO
Text Label 4200 2600 2    50   ~ 0
MOSI
Text Label 4200 2700 2    50   ~ 0
3V3
Text Label 4600 2000 0    50   ~ 0
TXD
Text Label 4600 2100 0    50   ~ 0
RXD
Text Label 4600 2200 0    50   ~ 0
SCL
Text Label 4600 2300 0    50   ~ 0
SDA
Text Label 4600 2400 0    50   ~ 0
M3
Text Label 4600 2500 0    50   ~ 0
M2
Text Label 4600 2600 0    50   ~ 0
M1
Text Label 4600 2700 0    50   ~ 0
GND
Text Label 4600 3350 0    50   ~ 0
3V3
Text Label 4600 3550 0    50   ~ 0
GND
Text Label 4600 3650 0    50   ~ 0
SCL
Text Label 4600 3750 0    50   ~ 0
SDA
Text Label 4600 3850 0    50   ~ 0
SA0
Text Label 4600 3950 0    50   ~ 0
NCS
Text Label 4600 4050 0    50   ~ 0
CSB
$EndSCHEMATC
