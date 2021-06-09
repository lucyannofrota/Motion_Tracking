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
	- S = "sensor"			(1 Byte)
	- X: Número do sensor 		(1 Byte)
	- ...: payload:
		+ accel_x		(2 Bytes)
		+ accel_y		(2 Bytes)	
		+ accel_z		(2 Bytes)
		+ roll			(2 Bytes)
		+ pitch			(2 Bytes)
		+ yaw			(2 Byte)
	- Caracteres fim: '}'		(1 Byte)
			  '\n'		(1 Byte)
					========
					(16 Bytes)						


===============================================================
roll,pitch,yaw: graus
accel: Gs/1000.f
