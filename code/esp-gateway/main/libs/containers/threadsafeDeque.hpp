#include <deque>
#include <freertos/semphr.h>
#include <freertos/portmacro.h>

template <typename T>
class ThreadSafeDeque {
private:
    std::deque<T> deque;
    SemaphoreHandle_t mutex;

public:
    ThreadSafeDeque() {
        // Create the binary semaphore
        mutex = xSemaphoreCreateBinary();
        // Initially, the semaphore must be given to be available
        xSemaphoreGive(mutex);
    }

    ~ThreadSafeDeque() {
        vSemaphoreDelete(mutex);
    }

    // Disallow copy and move semantics
    ThreadSafeDeque(const ThreadSafeDeque&) = delete;
    ThreadSafeDeque& operator=(const ThreadSafeDeque&) = delete;

    void push_back(const T& value) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        deque.push_back(value);
        xSemaphoreGive(mutex);
    }

    void push_front(const T& value) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        deque.push_front(value);
        xSemaphoreGive(mutex);
    }

    void pop_back() {
        xSemaphoreTake(mutex, portMAX_DELAY);
        if (!deque.empty()) {
            deque.pop_back();
        }
        xSemaphoreGive(mutex);
    }

    void pop_front() {
        xSemaphoreTake(mutex, portMAX_DELAY);
        if (!deque.empty()) {
            deque.pop_front();
        }
        xSemaphoreGive(mutex);
    }

    T at(size_t index) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        T value = deque.at(index); // throws std::out_of_range for invalid index
        xSemaphoreGive(mutex);
        return value;
    }

    size_t size() const {
        xSemaphoreTake(mutex, portMAX_DELAY);
        size_t size = deque.size();
        xSemaphoreGive(mutex);
        return size;
    }

    bool empty() const {
        xSemaphoreTake(mutex, portMAX_DELAY);
        bool isEmpty = deque.empty();
        xSemaphoreGive(mutex);
        return isEmpty;
    }

    void clear() {
        xSemaphoreTake(mutex, portMAX_DELAY);
        deque.clear();
        xSemaphoreGive(mutex);
    }

    // Thread-safe operator[]
    T& operator[](size_t index) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        T& value = deque[index]; // Danger: returning a reference to internal data
        xSemaphoreGive(mutex);
        return value;
    }

    // Thread-safe erase by index method
    void erase(size_t index) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        if (index < deque.size()) {
            auto it = deque.begin() + index;
            deque.erase(it);
        }
        xSemaphoreGive(mutex);
    }



    // Additional methods as needed...
};