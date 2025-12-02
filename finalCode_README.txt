finalCode_README
(hopefully) 

also testing the ESP wifi mode


ESP is used only as Access Point

No smart plug logic

No HTTP GET

Just energy monitoring + display + RTOS tasks + AP mode

ESP's SSID will be 
EnergyMonitor_AP

Password:
pasword123


What this gives for the demo

1) Plug your two USB loads through the INA219s as you wired them

2) Power the Nucleo + ESP + OLED

	3) ESP starts AP EnergyMonitor_AP / password123

	4) OLED shows:

CH1: XX.XXW YY.YYWh

CH2: XX.XXW YY.YYWh

Tot: XX.XXW YY.YYWh