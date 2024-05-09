#include "TinyMeshManager.hpp"
#include <string.h>


//private

bool TinyMeshManager::isResponse(TMPacket *response) {
    // response source == request destination
    //  AND
    // response destination == response source
    //  AND
    // response is OK OR ERR

    for (size_t i = 0; i < TMM_QUEUE_SIZE; i++) {
        if (send_queue[i].packet.isResponse() || send_queue[i].packet.empty())
            continue;

        if (response->getSource() == send_queue[i].packet.getDestination() && response->getDestination() == send_queue[i].packet.getSource()) {
            request_index = i;
            return true;
        }
    }

    return false;
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
    //if (isResponse(&send_queue[curr_index], &packet))
    //    return TMM_PACKET_RESPONSE;

    // response
    if (packet.isResponse()) {
        if (isResponse(&packet))
            return TMM_PACKET_RESPONSE;

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
        return queuePacket(request->getSource(), TM_MSG_OK, &data, 1);
    }

    return requestHandler(request, fwd);
}


uint8_t TinyMeshManager::receivePacket() {
    if (!interface->hasData())
        return TMM_NO_DATA;

    uint8_t len = TM_PACKET_SIZE;
    last_ret = interface->getData(packet.raw, &len);
    if (last_ret)
        return TMM_ERR_GET_DATA;

    last_ret = packet.checkHeader();
    if (last_ret)
        return TMM_ERR_CHECK_HEADER;

    return TMM_OK;
}



//public

TinyMeshManager::TinyMeshManager(InterfaceWrapperBase *interface) {
    this->interface = interface;
}
TinyMeshManager::TinyMeshManager(uint8_t address, InterfaceWrapperBase *interface) : TinyMeshManager(interface) {
    this->address = address;
}
TinyMeshManager::TinyMeshManager(uint8_t address, uint8_t node_type, InterfaceWrapperBase *interface) : TinyMeshManager(address, interface) {
    this->node_type = node_type;
}


void TinyMeshManager::setNodeType(uint8_t node_type) {
    if (node_type > TM_NODE_TYPE_LP)
        node_type = TM_NODE_TYPE_NORMAL;
    this->node_type = node_type;
}


uint8_t TinyMeshManager::queuePacket(uint8_t destination, uint8_t message_type, uint8_t *data, uint8_t length) {
    if (send_queue.reserve())
        return TMM_ERR_QUEUE_FULL;
    
    printf("New queue size: %d\n", send_queue.size());

    auto request = send_queue.last();
    last_ret = request->packet.buildPacket(address, destination, sequence_num++, node_type, message_type, 0, data, length);
    if (last_ret) {
        request->packet.clear();
        send_queue.popBack();
        return TMM_ERR_BUILD_PACKET;
    }
    request->tts = 0;
    return TMM_OK;
}

uint8_t TinyMeshManager::loop() {
    uint8_t ret = TMM_OK;
    last_ret = receivePacket();
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
                last_ret = responseHandler(&send_queue[request_index].packet, &packet);
                send_queue[request_index].packet.clear();
                ret |= TMM_RESPONSE;
            }

            // forward packet
            if (last_ret & TMM_PACKET_FORWARD) {
                if (send_queue.reserve())
                    ret |= TMM_ERR_FORWARD;
                else {
                    send_queue.last()->packet = packet;
                    send_queue.last()->tts = 0;
                    ret |= TMM_FORWARD;
                }
            }
        }
    }

    if (interface->isMediumBusy())
        return ret | TMM_MEDIUM_BUSY;

    last_loop_time = millis() - last_loop_time;
    bool is_empty = true;
    bool skip = false;
    for (size_t i = 0; i < send_queue.size(); i++) {
        auto item = send_queue.at(i);
        if (item->packet.empty())
            continue;

        is_empty = false;

        if (item->tts > last_loop_time) {
            item->tts -= last_loop_time;
            continue;
        }

        if (skip)
            continue;

        //timeout for this packet
        if (item->packet.getRepeatCount() == 3) {
            printf("Clearing stale packet\n");
            item->packet.clear();
            ret |= TMM_ERR_TIMEOUT;
            continue;
        }

        //send it
        last_ret = interface->sendData(item->packet.raw, item->packet.size());
        if (last_ret) {
            skip = true;
            continue;
        }

        //if it was a response, remove it
        if (item->packet.isResponse()) {
            printf("Clearing response\n");
            item->packet.clear();
            continue;
        }

        printf("rpt: %d\n", item->packet.getRepeatCount());
        item->packet.setRepeatCount(item->packet.getRepeatCount() + 1);
        item->tts = 1000;
    }
    last_loop_time = millis();

    int i = 0;
    while (send_queue.first()->packet.empty() && !send_queue.empty()) {
        send_queue.popFront();
        i++;
    }
    if (i)
        printf("Removed %d packet(s)\n", i);
    
    if (skip)
        return (ret & 0xF0) | TMM_ERR_SEND_DATA;

    if (is_empty)
        return ret | TMM_QUEUE_EMPTY;
    
    if ((ret & 0x0F) != TMM_ERR_TIMEOUT)
        return ret | TMM_SENT;

    return ret | TMM_AWAIT;
}
