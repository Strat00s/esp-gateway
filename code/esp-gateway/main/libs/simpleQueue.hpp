#include <string>
#include <queue>
#include <freertos/semphr.h>
#include <freertos/portmacro.h>


template<typename T>
class SimpleQueue {
private:
    SemaphoreHandle_t mux;
    SemaphoreHandle_t read_sem;
    std::queue<T> buffer;

public:
    SimpleQueue() {
        this->mux = xSemaphoreCreateBinary();
        this->read_sem = xSemaphoreCreateCounting(SIZE_MAX, 0);
        xSemaphoreGive(this->mux);
    }
    ~SimpleQueue() {

    }

    void write(T data, uint32_t timeout_ms = 10) {
        uint8_t ret = xSemaphoreTake(this->mux, pdMS_TO_TICKS(timeout_ms));
        if (ret) {
            this->buffer.push(data);
            xSemaphoreGive(this->mux);
            xSemaphoreGive(this->read_sem);
        }
    }

    /** @brief Block for wait_ms and read first data inserted into the queue.
     * 
     * @tparam T 
     * @param wait_ms How long to wait for a possible item to be inserted into the queue
     * @param timeout_ms Timeout in ms when trying to access the underlying buffer data sctructure
     * @return 
     */
    T read(uint32_t wait_ms = portMAX_DELAY, uint32_t timeout_ms = 10) {
        T data;

        //block until data is available
        uint8_t ret = xSemaphoreTake(this->read_sem, pdMS_TO_TICKS(wait_ms));
        if (!ret)
            return data;
    
        //gain access to the buffer and read from it
        ret = xSemaphoreTake(this->mux, pdMS_TO_TICKS(timeout_ms));
        if (ret) {
            data = this->buffer.front();
            this->buffer.pop();
            xSemaphoreGive(this->mux);
        }

        return data;
    }

    /** @brief Get number of stored items in the queue 
     * 
     * @return Number of stored items
     */
    size_t size() {
        return this->buffer.size();
    }
};