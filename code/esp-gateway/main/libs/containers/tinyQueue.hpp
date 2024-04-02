#include <stdint.h>

template<typename T>
struct Item {
    T data;
    Item *next;
};


template<typename T>
class TinyQueue {
private:
    Item<T> *iter   = nullptr;
    Item<T> *head   = nullptr;
    Item<T> *tail   = nullptr;
    size_t _size = 0;

public:

    TinyQueue(/* args */);
    ~TinyQueue() {
        clear();
    }

    /** @brief Iterate over the items. Starts at head and loops areound tail back to head.
     * 
     * @param reset Force start from head
     * @return Pointer to an inner item. Nullptr when iterating after tail
     */
    T *iterate(bool reset = false) {
        if (reset)
            iter = head;

        if (iter == nullptr)
            return nullptr;

        T *ret = &(iter->data);
        iter = iter->next;
        return ret;
    }

    void push(T data) {
        auto pushed = new Item<T>;
        pushed->data = data;
        pushed->next = nullptr;
        if (tail == nullptr)
            tail = pushed;
        else {
            tail->next = pushed;
            tail = pushed;
        }
        
        if (head == nullptr)
            head = pushed;
        _size++;
    }

    T pop() {
        if (_size == 0)
            return {0};

        T poped = head->data;
        auto tmp = head;
        this->head = head->next;
        
        //on last item
        if (head == nullptr)
            tail = nullptr;
        
        delete tmp;
        _size--;
        return poped;
    }

    void clear() {
        while (head != nullptr) {
            tail = head;
            head = head->next;
            delete tail;
        }

        _size = 0;
        tail = nullptr;
    }

    size_t size() {
        return _size;
    }
};
