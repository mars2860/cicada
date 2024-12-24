EESchema Schematic File Version 4
LIBS:cicada32-0802-esp32s3-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Cicada32 0802 ESP32S3"
Date "2024-12-12"
Rev "1"
Comp "www.liftelectronica.ru"
Comment1 ""
Comment2 "anton.sysoev.ru68@gmail.com"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector_Generic:Conn_01x07 J4
U 1 1 675A8E29
P 6700 3050
F 0 "J4" H 6700 3450 50  0000 C CNN
F 1 "EXT-RIGHT" H 6780 3001 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x07_P2.54mm_Vertical" H 6700 3050 50  0001 C CNN
F 3 "~" H 6700 3050 50  0001 C CNN
	1    6700 3050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x07 J1
U 1 1 675A8EF5
P 3400 3050
F 0 "J1" H 3400 3450 50  0000 C CNN
F 1 "EXT-LEFT" H 3480 3001 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x07_P2.54mm_Vertical" H 3400 3050 50  0001 C CNN
F 3 "~" H 3400 3050 50  0001 C CNN
	1    3400 3050
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5900 3350 6500 3350
Wire Wire Line
	5900 3250 6500 3250
Wire Wire Line
	5900 3150 6500 3150
Wire Wire Line
	5900 3050 6500 3050
Wire Wire Line
	5900 2950 6500 2950
Wire Wire Line
	5900 2850 6500 2850
Wire Wire Line
	5900 2750 6500 2750
Text Label 6200 3350 0    50   ~ 0
D0
Text Label 6200 3250 0    50   ~ 0
D1
Text Label 6200 3150 0    50   ~ 0
D2
Text Label 6200 3050 0    50   ~ 0
D3
Text Label 6200 2950 0    50   ~ 0
D4
Text Label 6200 2850 0    50   ~ 0
D5
Text Label 6200 2750 0    50   ~ 0
D6
Text Label 3900 2750 0    50   ~ 0
D7
Text Label 3900 2850 0    50   ~ 0
D8
Text Label 3900 2950 0    50   ~ 0
D9
Text Label 3900 3050 0    50   ~ 0
D10
Text Label 3900 3150 0    50   ~ 0
3V3
Text Label 3900 3250 0    50   ~ 0
GND
Wire Wire Line
	4250 3250 3600 3250
Wire Wire Line
	4250 3150 3600 3150
Wire Wire Line
	4250 3050 3600 3050
Wire Wire Line
	4250 2950 3600 2950
Wire Wire Line
	4250 2850 3600 2850
Wire Wire Line
	4250 2750 3600 2750
NoConn ~ 3600 3350
Text Notes 3650 3350 0    50   ~ 0
+BATT
Text Notes 4000 3350 0    50   ~ 0
VUSB
NoConn ~ 4250 3350
$Comp
L Connector_Generic:Conn_01x07 J3
U 1 1 675A900B
P 5700 3050
F 0 "J3" H 5700 3450 50  0000 C CNN
F 1 "EXT-RIGHT" H 5780 3001 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x07_P2.54mm_Vertical" H 5700 3050 50  0001 C CNN
F 3 "~" H 5700 3050 50  0001 C CNN
	1    5700 3050
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x07 J2
U 1 1 675A8D34
P 4450 3050
F 0 "J2" H 4450 3450 50  0000 C CNN
F 1 "EXT-LEFT" H 4530 3001 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x07_P2.54mm_Vertical" H 4450 3050 50  0001 C CNN
F 3 "~" H 4450 3050 50  0001 C CNN
	1    4450 3050
	1    0    0    -1  
$EndComp
$EndSCHEMATC
