# Esp8266-WebSocket-Server
Esp8266 WebSocket Server control via AT_COMMAND by ATmega32 microcontroler.
Suppport HTTP request.
Support websocket : https://tools.ietf.org/html/rfc6455

The ATmega32 is program like a state Machine and send command to the ESP8266 via UART.

Microcontroler host a web page, store in progmem, and send it to the client a request is made.
The request is 192.168.4.1/page
To open a websocket connection, the client (web browser) send a http request to the server. The header of the request contain a key wich is concatanate with the magic key, crypted with SHA1 algorithm and encoded in base 64. The encoded key is then return to the client.
- To send data from the server to the client, the data must be framed.
- The first byte must be 0x81 (0x8 to tell the message will end ans 0x01 to tell the data is text)
- The next byte represent the lenght of the data payload
- The rest of the message is the data payload.

# Compile
- make writeflash

or

- make
- make hex
- avrdude -p m32 -c your_isp_programmer -U flash:w:esp8266-webSocket.hex
