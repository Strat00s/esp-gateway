/**
 * @file simpleQueue.hpp
 * @author Lukáš Baštýř (l.bastyr@seznam.cz)
 * @brief Very simple thread safe queue (since the one from freertos is pretty bad)
 * @version 0.1
 * @date 02-01-2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once
#include <string>
#include <queue>
#include <freertos/semphr.h>
#include <freertos/portmacro.h>

template<typename T>
class SimpleQueue {
private:
    SemaphoreHandle_t mux;      //mux for accessing the underlying queue
    SemaphoreHandle_t read_sem; //semaphore showing how many items can be taken out
    std::queue<T> buffer;

public:
    SimpleQueue() {
        this->mux = xSemaphoreCreateMutex();   //access to underlying data in queue
        this->read_sem = xSemaphoreCreateCounting(SIZE_MAX, 0); //waiting "line" for the queue (how many items can be currently accessed one after another)
        //xSemaphoreGive(this->mux);
    }
    ~SimpleQueue() {

    }

    /** @brief Write data to the queue
     * 
     * @param data Data to write of type T
     * @param timeout_ms Timeout in ms when trying to access the underlying buffer data sctructure
     */
    void write(T data, uint32_t timeout_ms = 10) {
        uint8_t ret = xSemaphoreTake(this->mux, pdMS_TO_TICKS(timeout_ms));
        if (ret) {
            this->buffer.push(data);
            xSemaphoreGive(this->mux);
            xSemaphoreGive(this->read_sem); //there are data to be read
        }
    }

    /** @brief Block for wait_ms and read first data inserted into the queue.
     * 
     * @tparam T 
     * @param wait_ms How long to wait for a possible item to be inserted into the queue (waiting for item in queue)
     * @param timeout_ms Timeout in ms when trying to access the underlying buffer data sctructure (waiting for access to item in queue)
     * @return 0 on success, 1 on timeout
     */
    uint8_t read(T *data, uint32_t wait_ms = portMAX_DELAY, uint32_t timeout_ms = 10) {
        //block until data is available
        uint8_t ret = xSemaphoreTake(this->read_sem, pdMS_TO_TICKS(wait_ms));
        if (ret == pdFALSE) {
            data = nullptr;
            return 1;
        }
    
        //gain access to the buffer and read from it
        ret = xSemaphoreTake(this->mux, pdMS_TO_TICKS(timeout_ms));
        if (ret) {
            *data = this->buffer.front();
            this->buffer.pop();
            xSemaphoreGive(this->mux);
        }

        return 0;
    }

    /** @brief Get number of stored items in the queue 
     * 
     * @return Number of stored items
     */
    size_t size() {
        return this->buffer.size();
    }
};