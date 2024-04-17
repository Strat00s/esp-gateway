#include "TinyMeshManager.hpp"
#include <string.h>


//private

bool TinyMeshManager::isResponse(TMPacket *request, TMPacket *response) {
    // response source == request destination
    //  AND
    // response destination == response source
    //  AND
    // response is OK OR ERR
    return (response->getSource() == request->getDestination() &&
            response->getDestination() == request->getSource() &&
            (response->getMessageType() == TM_MSG_OK || response->getMessageType() == TM_MSG_ERR));
}


TMPacketID TinyMeshManager::createPacketID(TMPacket *packet) {
    TMPacketID pid;
    pid.setSource(packet->getSource());
    pid.setDestination(packet->getDestination());
    pid.setSequence(packet->getSequence());
    pid.setMessageType(packet->getMessageType());
    pid.setRepeatCount(packet->getRepeatCount());
    return pid;
}

void TinyMeshManager::setPacketID(TMPacketID *pid, TMPacket *packet) {
    pid->setSource(packet->getSource());
    pid->setDestination(packet->getDestination());
    pid->setSequence(packet->getSequence());
    pid->setMessageType(packet->getMessageType());
    pid->setRepeatCount(packet->getRepeatCount());
}

bool TinyMeshManager::savePacketID(TMPacket *packet) {
    for (uint8_t i = 0; i < TMM_PID_SIZE; i++) {
        if (pid_list[i].getSource() == packet->getSource() &&
            pid_list[i].getSequence() == packet->getSequence() &&
            pid_list[i].getDestination() == packet->getDestination()) {
            
            if (pid_list[i].getRepeatCount() < packet->getRepeatCount()) {
                setPacketID(&pid_list[i], packet);
                return true;
            }
            // duplicate
            return false;
        }
    }

    setPacketID(&pid_list[pid_index], packet);
    pid_index++;
    if (pid_index >= TMM_PID_SIZE)
        pid_index = 0;

    return true;
}


uint8_t TinyMeshManager::classifyPacket() {
    if (isResponse(&send_queue[curr_index], &packet))
        return TMM_PACKET_RESPONSE;

    // response
    if (packet.getMessageType() == TM_MSG_OK && packet.getMessageType() == TM_MSG_ERR) {
        if (packet.getDestination() == address)
            return TMM_PACKET_RND_RESPONSE;

        return TMM_PACKET_FORWARD;
    }

    // request
    if (packet.getDestination() == address)
        return TMM_PACKET_REQUEST;

    if (packet.getDestination() == TM_BROADCAST_ADDRESS)
        return TMM_PACKET_REQUEST | TMM_PACKET_FORWARD;

    return TMM_PACKET_FORWARD;
}


uint8_t TinyMeshManager::handleRequest(TMPacket *request, bool fwd) {
    if (request->getMessageType() == TM_MSG_PING) {
        if (fwd)
            return TMM_OK;

        uint8_t data = request->getData()[0];
        return sendResponse(request->getSource(), TM_MSG_OK, &data, 1);
    }

    return requestHandler(request, fwd);
}


inline uint8_t TinyMeshManager::sendData(TMPacket *packet, InterfaceWrapper *interface) {
    return interface ? interface->sendData(packet->raw, packet->size(), true) : if_manager->sendData(packet->raw, packet->size());
}


uint8_t TinyMeshManager::receivePacket(InterfaceWrapper *interface) {
    if (!(interface ? interface->hasData() : if_manager->hasData()))
        return TMM_NO_DATA;

    uint8_t len = TM_PACKET_SIZE;
    last_ret = interface ? interface->getData(packet.raw, &len) : if_manager->getNextData(packet.raw, &len);
    if (last_ret)
        return TMM_ERR_GET_DATA;

    last_ret = packet.checkHeader();
    if (last_ret)
        return TMM_ERR_CHECK_HEADER;

    return TMM_OK;
}



//public

TinyMeshManager::TinyMeshManager(InterfaceManager *interface_manager) {
    this->if_manager = interface_manager;
}
TinyMeshManager::TinyMeshManager(uint8_t address, InterfaceManager *interface_manager) : TinyMeshManager(interface_manager) {
    this->address = address;
}
TinyMeshManager::TinyMeshManager(uint8_t address, uint8_t node_type, InterfaceManager *interface_manager) : TinyMeshManager(address, interface_manager) {
    this->node_type = node_type;
}


void TinyMeshManager::setNodeType(uint8_t node_type) {
    if (node_type > TM_NODE_TYPE_LP)
        node_type = TM_NODE_TYPE_NORMAL;
    this->node_type = node_type;
}


uint8_t TinyMeshManager::sendResponse(uint8_t destination, uint8_t message_type, uint8_t *data, uint8_t length, InterfaceWrapper *interface) {
    if (!if_manager && !interface)
        return TMM_ERR_NULL;
    
    last_ret = packet.buildPacket(address, destination, sequence_num++, node_type, message_type, packet.getRepeatCount(), data, length);
    if (last_ret)
        return TMM_ERR_BUILD_PACKET;

    last_ret = sendData(&packet, interface);
    if (last_ret)
        return TMM_ERR_SEND_DATA;

    if (!savePacketID(&packet))
        return TMM_PACKET_DUPLICATE;

    return TMM_OK;
}

uint8_t TinyMeshManager::queueRequest(uint8_t destination, uint8_t message_type, uint8_t *data, uint8_t length) {
    // TODO ignore duplicats?
    if (send_index == curr_index && queue_len)
        return TMM_ERR_QUEUE_FULL;

    last_ret = send_queue[send_index].buildPacket(address, destination, sequence_num++, node_type, message_type, 0, data, length);
    if (last_ret) {
        send_queue[send_index].clear();
        return TMM_ERR_BUILD_PACKET;
    }

    queue_len++;
    send_index++;
    if (send_index >= TMM_QUEUE_SIZE)
        send_index = 0;

    return TMM_OK;
}

uint8_t TinyMeshManager::loop(InterfaceWrapper *interface) {
    if (!if_manager && !interface)
        return TMM_ERR_NULL;

    uint8_t ret = TMM_OK;
    last_ret = receivePacket(interface);
    if (last_ret == TMM_ERR_GET_DATA || last_ret == TMM_ERR_CHECK_HEADER)
        return ret | TMM_ERR_RECEIVE;

    // got some data
    if (!last_ret) {
        last_ret = classifyPacket();

        // not a duplicate packet
        if (savePacketID(&packet)) {
            if (packet.getMessageType() == TM_MSG_PING)
                packet.getData()[0]++;

            // handle request
            if (last_ret & TMM_PACKET_REQUEST) {
                last_ret = handleRequest(&packet, last_ret & TMM_PACKET_FORWARD);
                ret |= TMM_REQUEST;
            }
            // handle response
            if (last_ret & TMM_PACKET_RESPONSE) {
                last_ret = responseHandler(&send_queue[curr_index], &packet);
                queue_len--;
                send_queue[curr_index].clear();
                ret |= TMM_RESPONSE;
            }

            // forward packet
            if (last_ret & TMM_PACKET_FORWARD) {
                //TODO Problem with simultaneous sending -> queue it
                //TODO -> change how packet queue is used
                //TODO -> another big rework ahead
                if (sendData(&packet, interface))
                    ret |= TMM_ERR_FORWARD;
                ret |= TMM_FORWARD;
            }
        }
    }

    if (!queue_len)
        return ret | TMM_QUEUE_EMPTY;

    // empty curr_index
    if (send_queue[curr_index].empty()) {
        curr_index++;
        if (curr_index >= TMM_QUEUE_SIZE)
            curr_index = 0;
        repeat_time = 0;
    }

    // repeat
    if (millis() - repeat_time >= 1000) {
        // total timeout
        if (send_queue[curr_index].getRepeatCount() == 3) {
            queue_len--;
            send_queue[curr_index].clear();
            return ret | TMM_ERR_TIMEOUT;
        }

        last_ret = sendData(&send_queue[curr_index], interface);
        //last_ret = if_manager->sendData(send_queue[curr_index].raw, send_queue[curr_index].size());
        if (last_ret)
            return ret | TMM_ERR_SEND_DATA;

        send_queue[curr_index].setRepeatCount(send_queue[curr_index].getRepeatCount() + 1);
        repeat_time = millis();
        return ret | TMM_SENT;
    }

    return ret | TMM_AWAIT;
}
