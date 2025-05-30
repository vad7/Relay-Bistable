# Relay-Bistable
Переключение бистабильного реле с одной обмоткой

Используется драйвер мотора DRV8871, управляет переключением микроконтроллер ATTiny13A, опторазвязка при необходимости.

![image](https://github.com/user-attachments/assets/f4fa961b-c856-4491-8d01-08dd7417a387)

Board for efficient operation of the HF3F bistable relay. For minimum consumption, a bistable relay with one coil is used. On/off control via the Atmel Attiny13A microcontroller. It is possible to use opto-isolation or without it.

Fuses: BODLEVEL = 2.7V (BODLEVEL[1:0]=01), RSTDISBL=0, CKSEL[1:0]=10, CKDIV8=0

EEPROM:

0x00 Action: 0 - direct, 1 - switch by pulse<br>
0x01 ActiveLevel: 0 - Low, >0 - High<br>
0x02 ResetRelayAtStartup: 1 - Reset relay at power on <br>
0x03 PauseAfterSwitch: *0.1 sec, input immunity after relay switching<br>
0x04 PulseTime: *0.1 sec, powering coil time<br>
0x05 OnTime: *sec, when non zero reset after specified time<br>

