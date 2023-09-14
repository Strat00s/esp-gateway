#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <utility>

template<typename T>
class SimpleContainer {
private:
    std::vector<std::pair<uint16_t, T>> container;
    //std::vector<uint16_t> ids;

public:
    SimpleContainer() {}
    ~SimpleContainer() {}

    /** @brief Add item to container. If item with same id already exists, it will be replaced.
     * 
     * @param node Item to add
     */
    void addItem(uint16_t id, T item) {
        for (size_t i = 0; i < this->container.size(); i++) {
            if (this->container[i].first == id) {
                this->container[i] = std::pair(id, item);
                return;
            }
        }
        this->container.push_back(std::pair(id, item));
    }

    /** @brief Remove item with ID
     * 
     * @param id Id of item to be removed
     */
    void removeItem(uint16_t id) {
        for (size_t i = 0; i < this->container.size(); i++) {
            if (this->container[i].first == id) {
                this->container.erase(this->container.begin() + i);
                return;
            }
        }
    }

    /** @brief Retrieve item from container
     * 
     * @param id Id of item to get
     * @return Found item (zerod if none found)
     */
    T getItem(uint16_t id) {
        for (size_t i = 0; i < this->container.size(); i++) {
            if (this->container[i].first == id)
                return this->container[i].second;
        }
        return {0};
    }
};

