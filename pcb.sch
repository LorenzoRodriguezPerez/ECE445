EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 1 1
Title "Schematic Template"
Date ""
Rev "0.0"
Comp ""
Comment1 "Designed By:"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L pcb-rescue:Conn_01x02-Connector_Generic J1
U 1 1 62158050
P 6300 1050
F 0 "J1" H 6380 1042 50  0000 L CNN
F 1 "Conn_01x02" H 6380 951 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 6300 1050 50  0001 C CNN
F 3 "~" H 6300 1050 50  0001 C CNN
	1    6300 1050
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:+3V8-power #PWR07
U 1 1 62158E12
P 6100 850
F 0 "#PWR07" H 6100 700 50  0001 C CNN
F 1 "+3V8" H 6115 1023 50  0000 C CNN
F 2 "" H 6100 850 50  0001 C CNN
F 3 "" H 6100 850 50  0001 C CNN
	1    6100 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 850  6100 900 
$Comp
L pcb-rescue:GND-power #PWR08
U 1 1 62159EB4
P 6100 1350
F 0 "#PWR08" H 6100 1100 50  0001 C CNN
F 1 "GND" H 6105 1177 50  0000 C CNN
F 2 "" H 6100 1350 50  0001 C CNN
F 3 "" H 6100 1350 50  0001 C CNN
	1    6100 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 1350 6100 1150
$Comp
L pcb-rescue:PWR_FLAG-power #FLG01
U 1 1 6215B49B
P 6100 900
F 0 "#FLG01" H 6100 975 50  0001 C CNN
F 1 "PWR_FLAG" V 6100 1027 50  0000 L CNN
F 2 "" H 6100 900 50  0001 C CNN
F 3 "~" H 6100 900 50  0001 C CNN
	1    6100 900 
	0    -1   -1   0   
$EndComp
Connection ~ 6100 900 
Wire Wire Line
	6100 900  6100 1050
$Comp
L pcb-rescue:3v7_to_5V_boost-components_senior_design U5
U 1 1 621706DD
P 9850 1050
F 0 "U5" H 9850 1275 50  0000 C CNN
F 1 "3v7_to_5V_boost" H 9850 1184 50  0000 C CNN
F 2 "components_ece445:3v7_to_5V_boost" H 9850 0   50  0001 C CNN
F 3 "" H 9850 1050 50  0001 C CNN
	1    9850 1050
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:+3V8-power #PWR0107
U 1 1 6217153B
P 9300 950
F 0 "#PWR0107" H 9300 800 50  0001 C CNN
F 1 "+3V8" H 9315 1123 50  0000 C CNN
F 2 "" H 9300 950 50  0001 C CNN
F 3 "" H 9300 950 50  0001 C CNN
	1    9300 950 
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:+5V-power #PWR0108
U 1 1 62171BF8
P 10400 950
F 0 "#PWR0108" H 10400 800 50  0001 C CNN
F 1 "+5V" H 10415 1123 50  0000 C CNN
F 2 "" H 10400 950 50  0001 C CNN
F 3 "" H 10400 950 50  0001 C CNN
	1    10400 950 
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:GND-power #PWR0109
U 1 1 6217270E
P 10400 1200
F 0 "#PWR0109" H 10400 950 50  0001 C CNN
F 1 "GND" H 10405 1027 50  0000 C CNN
F 2 "" H 10400 1200 50  0001 C CNN
F 3 "" H 10400 1200 50  0001 C CNN
	1    10400 1200
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:GND-power #PWR0110
U 1 1 621738DC
P 9300 1200
F 0 "#PWR0110" H 9300 950 50  0001 C CNN
F 1 "GND" H 9305 1027 50  0000 C CNN
F 2 "" H 9300 1200 50  0001 C CNN
F 3 "" H 9300 1200 50  0001 C CNN
	1    9300 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 950  9300 1050
Wire Wire Line
	9300 1050 9500 1050
Wire Wire Line
	9300 1200 9300 1150
Wire Wire Line
	9300 1150 9500 1150
Wire Wire Line
	10200 1050 10400 1050
Wire Wire Line
	10400 1050 10400 950 
Wire Wire Line
	10200 1150 10400 1150
Wire Wire Line
	10400 1150 10400 1200
Text GLabel 7250 6150 2    50   Input ~ 0
PB6
Text GLabel 1200 1800 0    50   Input ~ 0
PB14
$Comp
L pcb-rescue:R_US-device R1
U 1 1 62161E5A
P 1600 1800
F 0 "R1" V 1395 1800 50  0000 C CNN
F 1 "4.7K" V 1486 1800 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1640 1790 50  0001 C CNN
F 3 "~" H 1600 1800 50  0001 C CNN
	1    1600 1800
	0    1    1    0   
$EndComp
$Comp
L pcb-rescue:R_US-device R2
U 1 1 62162A3E
P 2300 1350
F 0 "R2" H 2368 1396 50  0000 L CNN
F 1 "4.7K" H 2368 1305 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2340 1340 50  0001 C CNN
F 3 "~" H 2300 1350 50  0001 C CNN
	1    2300 1350
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:R_US-device R3
U 1 1 62163CEA
P 3500 1350
F 0 "R3" H 3568 1396 50  0000 L CNN
F 1 "1K" H 3568 1305 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3540 1340 50  0001 C CNN
F 3 "~" H 3500 1350 50  0001 C CNN
	1    3500 1350
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:+5V-power #PWR09
U 1 1 621653CF
P 2900 950
F 0 "#PWR09" H 2900 800 50  0001 C CNN
F 1 "+5V" H 2915 1123 50  0000 C CNN
F 2 "" H 2900 950 50  0001 C CNN
F 3 "" H 2900 950 50  0001 C CNN
	1    2900 950 
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:GND-power #PWR010
U 1 1 62165B2E
P 2950 2200
F 0 "#PWR010" H 2950 1950 50  0001 C CNN
F 1 "GND" H 2955 2027 50  0000 C CNN
F 2 "" H 2950 2200 50  0001 C CNN
F 3 "" H 2950 2200 50  0001 C CNN
	1    2950 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 1200 2900 1200
Wire Wire Line
	2900 1200 2900 950 
Wire Wire Line
	3500 1200 2900 1200
Connection ~ 2900 1200
Wire Wire Line
	3500 1500 3500 1600
Wire Wire Line
	2300 1500 2300 1550
Wire Wire Line
	2300 2000 2300 2100
Wire Wire Line
	2300 2100 2950 2100
Wire Wire Line
	2950 2100 2950 2200
Wire Wire Line
	3500 2000 3500 2100
Wire Wire Line
	3500 2100 2950 2100
Connection ~ 2950 2100
Wire Wire Line
	2300 1550 3000 1550
Wire Wire Line
	3000 1550 3000 1800
Wire Wire Line
	3000 1800 3200 1800
Connection ~ 2300 1550
Wire Wire Line
	2300 1550 2300 1600
Wire Wire Line
	1750 1800 2000 1800
Wire Wire Line
	1200 1800 1450 1800
$Comp
L pcb-rescue:Conn_01x03-Connector_Generic J2
U 1 1 6217517E
P 4450 1600
F 0 "J2" H 4530 1642 50  0000 L CNN
F 1 "Conn_01x03" H 4530 1551 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 4450 1600 50  0001 C CNN
F 3 "~" H 4450 1600 50  0001 C CNN
	1    4450 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 1200 4050 1200
Wire Wire Line
	4050 1200 4050 1500
Wire Wire Line
	4050 1500 4250 1500
Connection ~ 3500 1200
Wire Wire Line
	3500 1600 4250 1600
Wire Wire Line
	3500 2100 4150 2100
Wire Wire Line
	4150 2100 4150 1700
Wire Wire Line
	4150 1700 4250 1700
Connection ~ 3500 2100
$Comp
L pcb-rescue:+5V-power #PWR011
U 1 1 6218862B
P 2950 2650
F 0 "#PWR011" H 2950 2500 50  0001 C CNN
F 1 "+5V" H 2965 2823 50  0000 C CNN
F 2 "" H 2950 2650 50  0001 C CNN
F 3 "" H 2950 2650 50  0001 C CNN
	1    2950 2650
	1    0    0    -1  
$EndComp
Connection ~ 3550 3800
Wire Wire Line
	4200 3400 4300 3400
Wire Wire Line
	4200 3800 4200 3400
Wire Wire Line
	3550 3800 4200 3800
Wire Wire Line
	3550 3300 4300 3300
Connection ~ 3550 2900
Wire Wire Line
	4100 3200 4300 3200
Wire Wire Line
	4100 2900 4100 3200
Wire Wire Line
	3550 2900 4100 2900
$Comp
L pcb-rescue:Conn_01x03-Connector_Generic J3
U 1 1 6218864A
P 4500 3300
F 0 "J3" H 4580 3342 50  0000 L CNN
F 1 "Conn_01x03" H 4580 3251 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 4500 3300 50  0001 C CNN
F 3 "~" H 4500 3300 50  0001 C CNN
	1    4500 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 3500 1500 3500
Wire Wire Line
	1800 3500 2050 3500
Wire Wire Line
	2350 3250 2350 3300
Connection ~ 2350 3250
Wire Wire Line
	3050 3500 3250 3500
Wire Wire Line
	3050 3250 3050 3500
Wire Wire Line
	2350 3250 3050 3250
Connection ~ 3000 3800
Wire Wire Line
	3550 3800 3000 3800
Wire Wire Line
	3550 3700 3550 3800
Wire Wire Line
	3000 3800 3000 3900
Wire Wire Line
	2350 3800 3000 3800
Wire Wire Line
	2350 3700 2350 3800
Wire Wire Line
	2350 3200 2350 3250
Wire Wire Line
	3550 3200 3550 3300
Connection ~ 2950 2900
Wire Wire Line
	3550 2900 2950 2900
Wire Wire Line
	2950 2900 2950 2650
Wire Wire Line
	2350 2900 2950 2900
$Comp
L pcb-rescue:R_US-device R6
U 1 1 62188625
P 3550 3050
F 0 "R6" H 3618 3096 50  0000 L CNN
F 1 "1K" H 3618 3005 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3590 3040 50  0001 C CNN
F 3 "~" H 3550 3050 50  0001 C CNN
	1    3550 3050
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:R_US-device R5
U 1 1 6218861F
P 2350 3050
F 0 "R5" H 2418 3096 50  0000 L CNN
F 1 "4.7K" H 2418 3005 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2390 3040 50  0001 C CNN
F 3 "~" H 2350 3050 50  0001 C CNN
	1    2350 3050
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:R_US-device R4
U 1 1 62188619
P 1650 3500
F 0 "R4" V 1445 3500 50  0000 C CNN
F 1 "4.7K" V 1536 3500 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1690 3490 50  0001 C CNN
F 3 "~" H 1650 3500 50  0001 C CNN
	1    1650 3500
	0    1    1    0   
$EndComp
Text GLabel 1250 3500 0    50   Input ~ 0
PB15
$Comp
L pcb-rescue:GND-power #PWR012
U 1 1 62188631
P 3000 3900
F 0 "#PWR012" H 3000 3650 50  0001 C CNN
F 1 "GND" H 3005 3727 50  0000 C CNN
F 2 "" H 3000 3900 50  0001 C CNN
F 3 "" H 3000 3900 50  0001 C CNN
	1    3000 3900
	1    0    0    -1  
$EndComp
Wire Notes Line
	5350 500  5350 2600
Wire Notes Line
	5350 2600 10500 2600
Wire Notes Line
	5500 2600 5500 4150
Wire Notes Line
	5500 4150 550  4150
Text GLabel 3050 7000 2    50   Input ~ 0
PB15
Text GLabel 3050 6900 2    50   Input ~ 0
PB14
Text GLabel 3050 6800 2    50   Input ~ 0
PB13
Text GLabel 3050 6100 2    50   Input ~ 0
PB6
Text GLabel 3050 6200 2    50   Input ~ 0
PB7
Text GLabel 1050 6500 0    50   Input ~ 0
PA10
Text GLabel 1050 6400 0    50   Input ~ 0
PA9
Wire Wire Line
	3250 7600 3050 7600
Connection ~ 3250 7700
Wire Wire Line
	3250 7700 3250 7600
Wire Wire Line
	3250 7700 3050 7700
Wire Wire Line
	3250 7750 3250 7700
$Comp
L pcb-rescue:GND-power #PWR02
U 1 1 6210B942
P 3250 7750
F 0 "#PWR02" H 3250 7500 50  0001 C CNN
F 1 "GND" H 3255 7577 50  0000 C CNN
F 2 "" H 3250 7750 50  0001 C CNN
F 3 "" H 3250 7750 50  0001 C CNN
	1    3250 7750
	1    0    0    -1  
$EndComp
Text GLabel 7250 6050 2    50   Input ~ 0
PB7
Text GLabel 7200 4750 2    50   Input ~ 0
PB13
Text GLabel 7150 3950 2    50   Input ~ 0
PA9
Text GLabel 7150 3850 2    50   Input ~ 0
PA10
Wire Wire Line
	7700 5650 7700 5750
Wire Wire Line
	7250 5650 7700 5650
$Comp
L pcb-rescue:GND-power #PWR0106
U 1 1 6212D678
P 7700 5750
F 0 "#PWR0106" H 7700 5500 50  0001 C CNN
F 1 "GND" H 7705 5577 50  0000 C CNN
F 2 "" H 7700 5750 50  0001 C CNN
F 3 "" H 7700 5750 50  0001 C CNN
	1    7700 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 5450 7700 5400
Wire Wire Line
	7250 5450 7700 5450
$Comp
L pcb-rescue:+3V3-power #PWR0105
U 1 1 6212C9CF
P 7700 5400
F 0 "#PWR0105" H 7700 5250 50  0001 C CNN
F 1 "+3V3" H 7715 5573 50  0000 C CNN
F 2 "" H 7700 5400 50  0001 C CNN
F 3 "" H 7700 5400 50  0001 C CNN
	1    7700 5400
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:compass_accelerometer-components_senior_design U3
U 1 1 6212A98F
P 6950 5400
F 0 "U3" H 6983 5615 50  0000 C CNN
F 1 "compass_accelerometer" H 6983 5524 50  0000 C CNN
F 2 "components_ece445:compass_accelerometer" H 7000 4450 50  0001 C CNN
F 3 "" H 6950 5600 50  0001 C CNN
	1    6950 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 4850 7400 4900
Wire Wire Line
	7200 4850 7400 4850
$Comp
L pcb-rescue:GND-power #PWR0104
U 1 1 62127216
P 7400 4900
F 0 "#PWR0104" H 7400 4650 50  0001 C CNN
F 1 "GND" H 7405 4727 50  0000 C CNN
F 2 "" H 7400 4900 50  0001 C CNN
F 3 "" H 7400 4900 50  0001 C CNN
	1    7400 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 4650 7200 4650
Wire Wire Line
	7400 4550 7400 4650
$Comp
L pcb-rescue:+5V-power #PWR0103
U 1 1 6212655A
P 7400 4550
F 0 "#PWR0103" H 7400 4400 50  0001 C CNN
F 1 "+5V" H 7415 4723 50  0000 C CNN
F 2 "" H 7400 4550 50  0001 C CNN
F 3 "" H 7400 4550 50  0001 C CNN
	1    7400 4550
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:Shaft_encoder-components_senior_design U2
U 1 1 621258F0
P 6900 4650
F 0 "U2" H 6958 4875 50  0000 C CNN
F 1 "Shaft_encoder" H 6958 4784 50  0000 C CNN
F 2 "components_ece445:shaft_encoder" H 6950 4300 50  0001 C CNN
F 3 "" H 6900 4850 50  0001 C CNN
	1    6900 4650
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:GPS_NEO_6M_module-components_senior_design G1
U 1 1 621243D2
P 6900 3800
F 0 "G1" H 6958 4075 50  0000 C CNN
F 1 "GPS_NEO_6M_module" H 6958 3984 50  0000 C CNN
F 2 "components_ece445:GPS_NEO_6M_module" H 6900 3400 50  0001 C CNN
F 3 "" H 6950 3850 50  0001 C CNN
	1    6900 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7500 3750 7500 3650
Wire Wire Line
	7150 3750 7500 3750
Wire Wire Line
	7500 4050 7500 4100
Wire Wire Line
	7150 4050 7500 4050
$Comp
L pcb-rescue:GND-power #PWR0102
U 1 1 6211F78C
P 7500 4100
F 0 "#PWR0102" H 7500 3850 50  0001 C CNN
F 1 "GND" H 7505 3927 50  0000 C CNN
F 2 "" H 7500 4100 50  0001 C CNN
F 3 "" H 7500 4100 50  0001 C CNN
	1    7500 4100
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:+3V3-power #PWR0101
U 1 1 6211EE7A
P 7500 3650
F 0 "#PWR0101" H 7500 3500 50  0001 C CNN
F 1 "+3V3" H 7515 3823 50  0000 C CNN
F 2 "" H 7500 3650 50  0001 C CNN
F 3 "" H 7500 3650 50  0001 C CNN
	1    7500 3650
	1    0    0    -1  
$EndComp
Wire Notes Line
	5250 4150 5250 7850
Text Notes 7400 650  0    118  ~ 0
POWER SUPPLY
Text Notes 6700 3100 0    118  ~ 0
SENSORS
Text Notes 2700 4400 0    138  ~ 0
MCU
Text Notes 1200 950  0    138  ~ 0
SERVOS
$Comp
L pcb-rescue:2N3904-Transistor_BJT Q1
U 1 1 62221B30
P 2200 1800
F 0 "Q1" H 2390 1846 50  0000 L CNN
F 1 "2N3904" H 2390 1755 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 2400 1725 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N3904.pdf" H 2200 1800 50  0001 L CNN
	1    2200 1800
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:2N3904-Transistor_BJT Q3
U 1 1 62229153
P 3400 1800
F 0 "Q3" H 3590 1846 50  0000 L CNN
F 1 "2N3904" H 3590 1755 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 3600 1725 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N3904.pdf" H 3400 1800 50  0001 L CNN
	1    3400 1800
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:2N3904-Transistor_BJT Q2
U 1 1 6222B8F4
P 2250 3500
F 0 "Q2" H 2440 3546 50  0000 L CNN
F 1 "2N3904" H 2440 3455 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 2450 3425 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N3904.pdf" H 2250 3500 50  0001 L CNN
	1    2250 3500
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:2N3904-Transistor_BJT Q4
U 1 1 6222E0D3
P 3450 3500
F 0 "Q4" H 3640 3546 50  0000 L CNN
F 1 "2N3904" H 3640 3455 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 3650 3425 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N3904.pdf" H 3450 3500 50  0001 L CNN
	1    3450 3500
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:+5V-power #PWR0111
U 1 1 622332A4
P 7050 850
F 0 "#PWR0111" H 7050 700 50  0001 C CNN
F 1 "+5V" H 7065 1023 50  0000 C CNN
F 2 "" H 7050 850 50  0001 C CNN
F 3 "" H 7050 850 50  0001 C CNN
	1    7050 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 1150 7050 1150
Wire Wire Line
	7050 1150 7050 950 
$Comp
L pcb-rescue:+3V3-power #PWR0112
U 1 1 62237AA7
P 8800 800
F 0 "#PWR0112" H 8800 650 50  0001 C CNN
F 1 "+3V3" H 8815 973 50  0000 C CNN
F 2 "" H 8800 800 50  0001 C CNN
F 3 "" H 8800 800 50  0001 C CNN
	1    8800 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 1150 8750 1150
Wire Wire Line
	8800 1150 8800 800 
Wire Notes Line
	8350 2600 8350 6750
Text Notes 8750 3050 0    138  ~ 0
RECEIVER AND\nTRANSMITTER
NoConn ~ 7250 5550
NoConn ~ 7250 5750
NoConn ~ 7250 5850
NoConn ~ 7250 5950
$Comp
L pcb-rescue:C-device C2
U 1 1 62173946
P 8750 1450
F 0 "C2" H 8865 1496 50  0000 L CNN
F 1 "10uF" H 8865 1405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8788 1300 50  0001 C CNN
F 3 "~" H 8750 1450 50  0001 C CNN
	1    8750 1450
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:C-device C1
U 1 1 62176C5E
P 7100 1450
F 0 "C1" H 7215 1496 50  0000 L CNN
F 1 "10uF" H 7215 1405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7138 1300 50  0001 C CNN
F 3 "~" H 7100 1450 50  0001 C CNN
	1    7100 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 1300 8750 1150
Connection ~ 8750 1150
Wire Wire Line
	8750 1150 8800 1150
Wire Wire Line
	7100 1300 7100 1150
$Comp
L pcb-rescue:GND-power #PWR06
U 1 1 6217C90E
P 8750 1600
F 0 "#PWR06" H 8750 1350 50  0001 C CNN
F 1 "GND" H 8755 1427 50  0000 C CNN
F 2 "" H 8750 1600 50  0001 C CNN
F 3 "" H 8750 1600 50  0001 C CNN
	1    8750 1600
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:GND-power #PWR05
U 1 1 6217D0AC
P 7100 1600
F 0 "#PWR05" H 7100 1350 50  0001 C CNN
F 1 "GND" H 7105 1427 50  0000 C CNN
F 2 "" H 7100 1600 50  0001 C CNN
F 3 "" H 7100 1600 50  0001 C CNN
	1    7100 1600
	1    0    0    -1  
$EndComp
NoConn ~ 1050 5100
$Comp
L pcb-rescue:C-device C5
U 1 1 6219BA6D
P 7700 2150
F 0 "C5" H 7815 2196 50  0000 L CNN
F 1 "100nF" H 7815 2105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 7738 2000 50  0001 C CNN
F 3 "~" H 7700 2150 50  0001 C CNN
	1    7700 2150
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:C-device C6
U 1 1 6219C31E
P 8200 2150
F 0 "C6" H 8315 2196 50  0000 L CNN
F 1 "100nF" H 8315 2105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 8238 2000 50  0001 C CNN
F 3 "~" H 8200 2150 50  0001 C CNN
	1    8200 2150
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:C-device C7
U 1 1 6219E476
P 8650 2150
F 0 "C7" H 8765 2196 50  0000 L CNN
F 1 "100nF" H 8765 2105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 8688 2000 50  0001 C CNN
F 3 "~" H 8650 2150 50  0001 C CNN
	1    8650 2150
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:C-device C8
U 1 1 621A07D3
P 9100 2150
F 0 "C8" H 9215 2196 50  0000 L CNN
F 1 "100nF" H 9215 2105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 9138 2000 50  0001 C CNN
F 3 "~" H 9100 2150 50  0001 C CNN
	1    9100 2150
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:C-device C4
U 1 1 621A2A3C
P 7250 2150
F 0 "C4" H 7365 2196 50  0000 L CNN
F 1 "100nF" H 7365 2105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 7288 2000 50  0001 C CNN
F 3 "~" H 7250 2150 50  0001 C CNN
	1    7250 2150
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:C-device C3
U 1 1 621A54A4
P 6800 2150
F 0 "C3" H 6915 2196 50  0000 L CNN
F 1 "4.7uF" H 6915 2105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 6838 2000 50  0001 C CNN
F 3 "~" H 6800 2150 50  0001 C CNN
	1    6800 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 1950 6800 1950
Wire Wire Line
	9100 1950 9100 2000
Wire Wire Line
	8650 2000 8650 1950
Connection ~ 8650 1950
Wire Wire Line
	8650 1950 9100 1950
Wire Wire Line
	8200 2000 8200 1950
Connection ~ 8200 1950
Wire Wire Line
	8200 1950 8650 1950
Wire Wire Line
	7700 2000 7700 1950
Connection ~ 7700 1950
Wire Wire Line
	7700 1950 8200 1950
Wire Wire Line
	7250 2000 7250 1950
Connection ~ 7250 1950
Wire Wire Line
	7250 1950 7700 1950
Wire Wire Line
	6800 2000 6800 1950
Connection ~ 6800 1950
Wire Wire Line
	6800 1950 7250 1950
Wire Wire Line
	9100 2300 9100 2350
Wire Wire Line
	9100 2350 8650 2350
Wire Wire Line
	6800 2350 6800 2300
Wire Wire Line
	7250 2350 7250 2300
Connection ~ 7250 2350
Wire Wire Line
	7250 2350 6800 2350
Wire Wire Line
	7700 2350 7700 2300
Connection ~ 7700 2350
Wire Wire Line
	7700 2350 7250 2350
Wire Wire Line
	8200 2350 8200 2300
Connection ~ 8200 2350
Wire Wire Line
	8200 2350 7950 2350
Wire Wire Line
	8650 2350 8650 2300
Connection ~ 8650 2350
Wire Wire Line
	8650 2350 8200 2350
$Comp
L pcb-rescue:+3V3-power #PWR013
U 1 1 621CA0C4
P 6700 1950
F 0 "#PWR013" H 6700 1800 50  0001 C CNN
F 1 "+3V3" H 6715 2123 50  0000 C CNN
F 2 "" H 6700 1950 50  0001 C CNN
F 3 "" H 6700 1950 50  0001 C CNN
	1    6700 1950
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:GND-power #PWR014
U 1 1 621CAB3C
P 7950 2350
F 0 "#PWR014" H 7950 2100 50  0001 C CNN
F 1 "GND" H 7955 2177 50  0000 C CNN
F 2 "" H 7950 2350 50  0001 C CNN
F 3 "" H 7950 2350 50  0001 C CNN
	1    7950 2350
	1    0    0    -1  
$EndComp
Connection ~ 7950 2350
Wire Wire Line
	7950 2350 7700 2350
$Comp
L pcb-rescue:GND-power #PWR015
U 1 1 621D8303
P 750 5300
F 0 "#PWR015" H 750 5050 50  0001 C CNN
F 1 "GND" H 755 5127 50  0000 C CNN
F 2 "" H 750 5300 50  0001 C CNN
F 3 "" H 750 5300 50  0001 C CNN
	1    750  5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  5300 1050 5300
Text GLabel 1050 6800 0    50   Input ~ 0
PA13
Text GLabel 1050 6900 0    50   Input ~ 0
PA14
Text GLabel 3700 5200 0    50   Input ~ 0
PA13
Text GLabel 3700 5300 0    50   Input ~ 0
PA14
Text GLabel 1050 5200 0    50   Input ~ 0
NRST
NoConn ~ 1050 5900
NoConn ~ 1050 6000
NoConn ~ 1050 6200
NoConn ~ 1050 6300
NoConn ~ 1050 6600
NoConn ~ 1050 6700
NoConn ~ 1050 7000
NoConn ~ 1050 7200
NoConn ~ 1050 7300
NoConn ~ 1050 7400
NoConn ~ 3050 5500
NoConn ~ 3050 5600
NoConn ~ 3050 5700
NoConn ~ 3050 5900
NoConn ~ 3050 6000
NoConn ~ 3050 6300
NoConn ~ 3050 6400
NoConn ~ 3050 6700
NoConn ~ 3050 7200
NoConn ~ 3050 7300
NoConn ~ 9800 1600
NoConn ~ 9900 1600
Wire Wire Line
	3300 4700 3300 4900
$Comp
L pcb-rescue:+3V3-power #PWR03
U 1 1 6227E3C2
P 3300 4700
F 0 "#PWR03" H 3300 4550 50  0001 C CNN
F 1 "+3V3" H 3315 4873 50  0000 C CNN
F 2 "" H 3300 4700 50  0001 C CNN
F 3 "" H 3300 4700 50  0001 C CNN
	1    3300 4700
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:LM1117-3.3-Regulator_Linear U4
U 1 1 62158C27
P 7850 950
F 0 "U4" H 7850 1192 50  0000 C CNN
F 1 "LM1117-3.3" H 7850 1101 50  0000 C CNN
F 2 "LM1117DT-3_1:TO228P970X238-3N" H 7850 950 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm1117.pdf" H 7850 950 50  0001 C CNN
	1    7850 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 950  8700 950 
Wire Wire Line
	8700 950  8700 1150
$Comp
L pcb-rescue:GND-power #PWR018
U 1 1 6216B3AE
P 7850 1250
F 0 "#PWR018" H 7850 1000 50  0001 C CNN
F 1 "GND" H 7855 1077 50  0000 C CNN
F 2 "" H 7850 1250 50  0001 C CNN
F 3 "" H 7850 1250 50  0001 C CNN
	1    7850 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 950  7550 950 
Connection ~ 7050 950 
Wire Wire Line
	7050 950  7050 850 
Wire Wire Line
	3050 4900 3300 4900
$Comp
L pcb-rescue:STM32F103C8T6-STM32F103C8T6 U1
U 1 1 62107107
P 2050 6300
F 0 "U1" H 2050 8067 50  0000 C CNN
F 1 "STM32F103C8T6" H 2050 7976 50  0000 C CNN
F 2 "STM32F103C8T6:QFP50P900X900X160-48N" H 2050 6300 50  0001 L BNN
F 3 "" H 2050 6300 50  0001 L BNN
F 4 "10" H 2050 6300 50  0001 L BNN "PARTREV"
F 5 "IPC7351B" H 2050 6300 50  0001 L BNN "STANDARD"
F 6 "ST Microelectronics" H 2050 6300 50  0001 L BNN "MANUFACTURER"
	1    2050 6300
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:+3V3-power #PWR016
U 1 1 622124C1
P 3100 4650
F 0 "#PWR016" H 3100 4500 50  0001 C CNN
F 1 "+3V3" H 3115 4823 50  0000 C CNN
F 2 "" H 3100 4650 50  0001 C CNN
F 3 "" H 3100 4650 50  0001 C CNN
	1    3100 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 4650 3100 4800
Wire Wire Line
	3100 4800 3050 4800
$Comp
L pcb-rescue:Conn_01x03-Connector_Generic J9
U 1 1 62207F3A
P 8950 4450
F 0 "J9" H 9030 4492 50  0000 L CNN
F 1 "Conn_01x03-Connector_Generic" H 9030 4401 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 8950 4450 50  0001 C CNN
F 3 "" H 8950 4450 50  0001 C CNN
	1    8950 4450
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:Conn_01x03-Connector_Generic J10
U 1 1 6220D4FE
P 8950 4800
F 0 "J10" H 9030 4842 50  0000 L CNN
F 1 "Conn_01x03-Connector_Generic" H 9030 4751 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 8950 4800 50  0001 C CNN
F 3 "" H 8950 4800 50  0001 C CNN
	1    8950 4800
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:Conn_01x03-Connector_Generic J11
U 1 1 62211821
P 8950 5150
F 0 "J11" H 9030 5192 50  0000 L CNN
F 1 "Conn_01x03-Connector_Generic" H 9030 5101 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 8950 5150 50  0001 C CNN
F 3 "" H 8950 5150 50  0001 C CNN
	1    8950 5150
	1    0    0    -1  
$EndComp
$Comp
L pcb-rescue:Conn_01x03-Connector_Generic J7
U 1 1 622234B4
P 8950 3750
F 0 "J7" H 9030 3792 50  0000 L CNN
F 1 "Conn_01x03-Connector_Generic" H 9030 3701 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 8950 3750 50  0001 C CNN
F 3 "" H 8950 3750 50  0001 C CNN
	1    8950 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 5250 8750 5250
Wire Wire Line
	8700 5250 8700 4900
Wire Wire Line
	8700 4900 8750 4900
Wire Wire Line
	8700 4900 8700 4550
Wire Wire Line
	8700 3850 8750 3850
Connection ~ 8700 4900
Wire Wire Line
	8700 4200 8750 4200
Connection ~ 8700 4200
Wire Wire Line
	8700 4200 8700 3850
Wire Wire Line
	8700 4550 8750 4550
Connection ~ 8700 4550
Wire Wire Line
	8700 4550 8700 4200
$Comp
L pcb-rescue:Conn_01x03-Connector_Generic J6
U 1 1 62265A09
P 3900 5300
F 0 "J6" H 3980 5342 50  0000 L CNN
F 1 "Conn_01x03-Connector_Generic" H 3980 5251 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 3900 5300 50  0001 C CNN
F 3 "" H 3900 5300 50  0001 C CNN
	1    3900 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 5450 3650 5400
Wire Wire Line
	3650 5400 3700 5400
$Comp
L device:R_US R7
U 1 1 622421B2
P 4000 6600
F 0 "R7" H 4068 6646 50  0000 L CNN
F 1 "10K" H 4068 6555 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4040 6590 50  0001 C CNN
F 3 "~" H 4000 6600 50  0001 C CNN
	1    4000 6600
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 6224316A
P 4500 7100
F 0 "SW1" V 4454 7248 50  0000 L CNN
F 1 "SW_Push" V 4545 7248 50  0000 L CNN
F 2 "Button_Switch_SMD:SW_SPST_PTS645" H 4500 7300 50  0001 C CNN
F 3 "" H 4500 7300 50  0001 C CNN
	1    4500 7100
	0    1    1    0   
$EndComp
$Comp
L device:C C9
U 1 1 622440FF
P 4000 7050
F 0 "C9" H 4115 7096 50  0000 L CNN
F 1 "100nF" H 4115 7005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4038 6900 50  0001 C CNN
F 3 "~" H 4000 7050 50  0001 C CNN
	1    4000 7050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 62245113
P 4000 7400
F 0 "#PWR04" H 4000 7150 50  0001 C CNN
F 1 "GND" H 4005 7227 50  0000 C CNN
F 2 "" H 4000 7400 50  0001 C CNN
F 3 "" H 4000 7400 50  0001 C CNN
	1    4000 7400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 62245C01
P 4500 7400
F 0 "#PWR020" H 4500 7150 50  0001 C CNN
F 1 "GND" H 4505 7227 50  0000 C CNN
F 2 "" H 4500 7400 50  0001 C CNN
F 3 "" H 4500 7400 50  0001 C CNN
	1    4500 7400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 6350 4000 6450
Wire Wire Line
	4000 6750 4000 6800
Wire Wire Line
	4000 6900 4500 6900
Connection ~ 4000 6900
Wire Wire Line
	4500 7300 4500 7400
Wire Wire Line
	4000 7200 4000 7400
Text GLabel 3900 6800 0    50   Input ~ 0
NRST
Wire Wire Line
	3900 6800 4000 6800
Connection ~ 4000 6800
Wire Wire Line
	4000 6800 4000 6900
$Comp
L pcb-rescue:+3V3-power #PWR0113
U 1 1 622705CC
P 4000 6350
F 0 "#PWR0113" H 4000 6200 50  0001 C CNN
F 1 "+3V3" H 4015 6523 50  0000 C CNN
F 2 "" H 4000 6350 50  0001 C CNN
F 3 "" H 4000 6350 50  0001 C CNN
	1    4000 6350
	1    0    0    -1  
$EndComp
Text GLabel 1050 5500 0    50   Input ~ 0
PA0
Text GLabel 1050 5600 0    50   Input ~ 0
PA1
Text GLabel 1050 5700 0    50   Input ~ 0
PA2
Text GLabel 1050 5800 0    50   Input ~ 0
PA3
Text GLabel 1050 6100 0    50   Input ~ 0
PA6
Text GLabel 8750 3650 0    50   Input ~ 0
PA0
Text GLabel 8750 4000 0    50   Input ~ 0
PA1
Text GLabel 8750 4350 0    50   Input ~ 0
PA2
Text GLabel 8750 4700 0    50   Input ~ 0
PA3
Text GLabel 8750 5050 0    50   Input ~ 0
PA6
Wire Wire Line
	8700 5900 8750 5900
$Comp
L power:GND #PWR019
U 1 1 62269E09
P 8700 5950
F 0 "#PWR019" H 8700 5700 50  0001 C CNN
F 1 "GND" H 8705 5777 50  0000 C CNN
F 2 "" H 8700 5950 50  0001 C CNN
F 3 "" H 8700 5950 50  0001 C CNN
	1    8700 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 5900 8700 5950
$Comp
L power:+5V #PWR01
U 1 1 622702F8
P 8600 5350
F 0 "#PWR01" H 8600 5200 50  0001 C CNN
F 1 "+5V" H 8615 5523 50  0000 C CNN
F 2 "" H 8600 5350 50  0001 C CNN
F 3 "" H 8600 5350 50  0001 C CNN
	1    8600 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 5350 8600 5400
Text GLabel 8750 5600 0    50   Input ~ 0
TX
Text GLabel 8750 5500 0    50   Input ~ 0
RX
Text GLabel 3050 6500 2    50   Input ~ 0
RX
Text GLabel 3050 6600 2    50   Input ~ 0
TX
$Comp
L power:GND #PWR0114
U 1 1 6232D4ED
P 3650 5450
F 0 "#PWR0114" H 3650 5200 50  0001 C CNN
F 1 "GND" H 3655 5277 50  0000 C CNN
F 2 "" H 3650 5450 50  0001 C CNN
F 3 "" H 3650 5450 50  0001 C CNN
	1    3650 5450
	1    0    0    -1  
$EndComp
NoConn ~ 8750 5150
NoConn ~ 8750 4800
NoConn ~ 8750 4450
NoConn ~ 8750 4100
NoConn ~ 8750 3750
NoConn ~ 3050 5800
$Comp
L Connector_Generic:Conn_01x06 J4
U 1 1 6236D2D4
P 8950 5600
F 0 "J4" H 9030 5592 50  0000 L CNN
F 1 "Conn_01x06" H 9030 5501 50  0000 L CNN
F 2 "Connector_Hirose:Hirose_DF13-06P-1.25DSA_1x06_P1.25mm_Vertical" H 8950 5600 50  0001 C CNN
F 3 "~" H 8950 5600 50  0001 C CNN
	1    8950 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 5400 8750 5400
NoConn ~ 8750 5700
NoConn ~ 8750 5800
Wire Wire Line
	8700 5900 8700 5250
Connection ~ 8700 5900
Connection ~ 8700 5250
$Comp
L pcb-rescue:Conn_01x03-Connector_Generic J8
U 1 1 6221F080
P 8950 4100
F 0 "J8" H 9030 4142 50  0000 L CNN
F 1 "Conn_01x03-Connector_Generic" H 9030 4051 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 8950 4100 50  0001 C CNN
F 3 "" H 8950 4100 50  0001 C CNN
	1    8950 4100
	1    0    0    -1  
$EndComp
$EndSCHEMATC
