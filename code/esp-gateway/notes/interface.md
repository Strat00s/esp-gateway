run ->

while hasdata {
    //loop through all interfaces
    //get next packet from first interface with data
    //check if packet is valid
    //check if it is an answer
    msg_type = getNextPacket(packet_t *packet);

    response -> response handler
    request -> request handler
    forward -> send
    broadcast -> response or request and send
}

get data
check if they are valid packet
if so, handle them
request:
    ping -> answer ok


response:
OK or ERR and data


getNextData - get data from interface
checkPacket - check if data are valid packet and what type of packet it is (request, response | forward, broadcast)
handlePacket - 
