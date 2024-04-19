#include <stddef.h>
#include <stdint.h>

template<typename T, size_t N>
class StaticDeque {
private:
    T data[N];
    size_t front = 0;
    size_t count = 0;

public:
    //StaticDeque(/* args */);
    //~StaticDeque();

    /** @brief Push item to the back of the deque.
     * 
     * @param item Item to be pushed.
     * @return 0 on success.
     * 1 if full.
     */
    uint8_t pushBack(const T& item) {
        if (count >= N)
            return 1;
        
        data[(front + count) % N] = item;
        count++;
        return 0;
    }

    /** @brief Remove first item from the deque.
     * 
     * @return 0 on success.
     * 1 if empty.
     */
    uint8_t popFront() {
        if (!count)
            return 1;
        
        front = (front + 1) % N;
        count--;
        return 0;
    }

    /** @brief Remove last item from the deque.
     * 
     * @return 0 on success.
     * 1 if empty.
     */
    uint8_t popBack() {
        if (!count)
            return 1;

        count--;
        return 0;
    }

    /** @brief Reserver extra slot in the queue for direct acccess ("push" nothing).
     * Effectively just increases size by one (up to maximum size).
     * 
     * @return 0 on success.
     * 1 if not enough space.
     */
    uint8_t reserve() {
        if (count >= N)
            return 1;
        count++;
        return 0;
    }

    /** @brief Get reference to first item.*/
    inline T *first() {
        return &data[front];
    }

    /** @brief Get reference to last item.*/
    inline T *last() {
        return &data[(front + count - 1) % N];
    }

    /** @brief Get reference to any item.
     * 
     * @param index Index of the item.
     */
    inline T &operator[](size_t index) {
        return data[(front + index) % N];
    }

    inline T* at(size_t index) {
        return &data[(front + index) % N];
    }

    
    inline bool full() const {
        return count == N;
    }

    inline bool empty() const {
        return !count;
    }

    inline size_t size() const {
        return count;
    }
};
