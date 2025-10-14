#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <Arduino.h>

template <typename T>
class RingBuffer {
private:
    T* buffer;
    size_t capacity;
    size_t head;
    size_t tail;
    size_t count;

public:

    RingBuffer(size_t size)
        : capacity(size), head(0), tail(0), count(0)
    {
        buffer = new T[capacity];
    }

    
    ~RingBuffer() {
        delete[] buffer;
    }

    
    void push(const T& item) {
        buffer[head] = item;
        head = (head + 1) % capacity;

        if (count < capacity)
            count++;
        else
            tail = (tail + 1) % capacity; 
    }


    T pop() {
        if (isEmpty()) return T();
        T value = buffer[tail];
        tail = (tail + 1) % capacity;
        count--;
        return value;
    }


    T get(size_t index) const {
        if (index >= count) return T();
        size_t pos = (tail + index) % capacity;
        return buffer[pos];
    }

    T getSum(size_t startIndex, size_t amount) const {
        if (count == 0 || startIndex >= count) return 0;

        T sum = 0;
        size_t actualCount = (amount< count) ? amount : count;

        
        for (size_t i = 0; i < actualCount; i++) {
            
            size_t idx = (tail + ((startIndex + count) - i) % count) % capacity;
            sum += static_cast<float>(buffer[idx]);
        }

        return sum;
    }
    bool isFull() const { return count == capacity; }
    bool isEmpty() const { return count == 0; }
    size_t size() const { return count; }

    
    float average() const {
        if (count == 0) return 0;
        float sum = 0;
        for (size_t i = 0; i < count; i++)
            sum += static_cast<float>(buffer[(tail + i) % capacity]);
        return sum / count;
    }

    void clear() {
        head = tail = count = 0;
    }
};

#endif
