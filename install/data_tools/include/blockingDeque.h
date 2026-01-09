#pragma once
#ifndef _BLOCKING_DEQUE_H_
#define _BLOCKING_DEQUE_H_

#include <mutex>
#include <math.h>
#include <condition_variable>

template<typename T>
class BlockingDeque{
private:
    std::deque<T> deque;
    std::mutex mutex;
    std::condition_variable condition;

public:
    void push_back(const T& item){
        std::lock_guard<std::mutex> lock(mutex);
        while(deque.size() >= 2000)
            deque.pop_front();
        deque.push_back(item);
        condition.notify_one();
    }

    T getRecentItem(double time){
        T item;
        std::lock_guard<std::mutex> lock(mutex);
        double minTimeDiff = INFINITY;
        for(int i=0; i < deque.size(); i++){
            double timeDiff = fabs(deque.at(i).header.stamp.toSec() - time);
            if(timeDiff < minTimeDiff){
                item = deque.front();
                deque.pop_front();
            }
            else
                break;
        }
        return item;
    }

    T pop_front(){
        std::unique_lock<std::mutex> lock(mutex);
        while(deque.empty())
            condition.wait(lock);
        T item = deque.front();
        deque.pop_front();
        return item;
    }

    T front(){
        std::unique_lock<std::mutex> lock(mutex);
        while(deque.empty())
            condition.wait(lock);
        T item = deque.front();
        return item;
    }

    T pop_back(){
        std::unique_lock<std::mutex> lock(mutex);
        while(deque.empty())
            condition.wait(lock);
        T item = deque.back();
        deque.pop_back();
        return item;
    }

    T back(){
        std::unique_lock<std::mutex> lock(mutex);
        while(deque.empty())
            condition.wait(lock);
        T item = deque.back();
        return item;
    }

    int size(){
        std::lock_guard<std::mutex> lock(mutex);
        return deque.size();
    }
};

#endif
