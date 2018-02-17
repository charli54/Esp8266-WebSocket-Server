# Esp8266-WebSocket-Server
Esp8266 WebSocket Server control via AT_COMMAND by ATmega32 microcontroler

The ATmega32 is program like a state Machine and send command to the ESP8266 via UART.

Microcontroler host a web page, store in progmem, and send it to the client a request is made.
The request is 192.168.4.1/page
# Compile
- make writeflash

or

- make
- make hex
- avrdude -p m32 -c your_isp_programmer -U flash:w:esp8266-webSocket.hex
