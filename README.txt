Position:

Formato da mensagem:
___________________________________________
|	|	|	|	  |	  |
|   {   |   P   | ...   |   }     |  \n   |
|_______|_______|_______|_________|_______|
Em que:
	- Caracter inicio: '{' 
	- P = "position"
	- ...: Payload
	- Caracteres fim: '}\n'


Sensor: 

Formato da mensagem:
__________________________________________________
|	|	|	|	  |	  |       |
|   {   |   S   |   X   |   ...   |   }   |  \n   |
|_______|_______|_______|_________|_______|_______|
Em que:
	- Caracter inicio: '{' 
	- S = "sensor"
	- X: Número do sensor
	- ...: payload:
		+ accel_x,
		+ accel_y
		+ accel_z
		+ roll
		+ pitch
		+ yaw
	- Caracteres fim: '}\n'

===============================================================
roll,pitch,yaw: graus
accel: Gs
