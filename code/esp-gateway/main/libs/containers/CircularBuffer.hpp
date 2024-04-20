#include <stddef.h>

template<typename T, size_t N>
class CircularBuffer {
private:
    T buf[N];
    size_t index;

public:
    CircularBuffer(/* args */);
    ~CircularBuffer();

    void push(T item) {
        buf[index] = item;
        index++;
        if (index > N)
            index = 0;
    }

    size_t size() {
        return N;
    }

    size_t getIndex() {
        return index;
    }

    T &operator[](size_t index) {
        if (index > N)
            return buf[0];
        return buf[index];
    }
};
