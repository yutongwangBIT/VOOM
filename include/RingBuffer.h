/**
* This file is part of OA-SLAM.
*
* Copyright (C) 2022 Matthieu Zins <matthieu.zins@inria.fr>
* (Inria, LORIA, Université de Lorraine)
* OA-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OA-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OA-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/


#include <vector>
#include <ostream>

namespace ORB_SLAM2
{
    
template<class T, class Allocator = std::allocator<T>> 
class RingBuffer : private std::vector<T, Allocator>
{
    /**
     *  =======  RingBuffer: circular container (implemented with std::vector)  =======  
     * interface:
     *  - RingBuffer(int max_size): construct a pre-allocated buffer of max_size+1.
     *  - push_front(): insert a new element at the front (overwrite last element if needed) (O(1))
     *  - is_full(): return true if the buffer is full (O(1))
     *  - size(): return the size of the buffer (O(1))
     *  - back(): return the last element. (O(1))
     *  - front(): return the element at the front. (O(1))
     * */

public:
    RingBuffer(size_t size) : std::vector<T, Allocator>(size+1), max_size_(size), impl_size_(size+1) {
    }

    void push_front(const T& x)  {
        this->operator[](head_) = x;
        head_ = (head_ + 1) % impl_size_;
        size_ = std::min(size_+1, max_size_);
    }

    bool is_full() const {
        return size_ == max_size_;
    }


    const T& back() const {
        if (is_full())
            return this->operator[]((head_ + 1) % impl_size_);
        else
            return this->operator[](0);
    }

    const T& front() const {
        int i = head_ - 1;
        if (i < 0)
            i += impl_size_;
        return this->operator[](i);
        
    }

    size_t size() const noexcept {
        return size_;
    }

    /**
     * Clear the buffer. Just reset head and size, do not erase elements
     * */
    void clear() noexcept {
        head_ = 0;
        size_ = 0;
    }


    void reset(size_t size) noexcept {
        clear();
        max_size_ = size;
        impl_size_ = size + 1;
        this->resize(size);
    }

    template <class U, class V>
    friend std::ostream& operator<<(std::ostream& os, const RingBuffer<U, V>& rb);

    std::vector<T, Allocator> to_vector() const {
        std::vector<T, Allocator> v(size_);
        int a = 0;
        for (auto it = this->begin(); it != this->end(); ++it, ++a) {
            v[a] = *it;
        }
        return v;
    }

private:
    size_t max_size_;  // maximum number of elements contained in the buffer
    size_t impl_size_; // internal size of the buffer (should be max_size_ +1 element used as head)
    int head_ = 0;     // head of the buffer. Indicates the first place to use when pushing front and correspond the end (i.e the next place after the last elements).
    size_t size_ = 0;  // current size

public:


    class iterator {
    private:
        friend class RingBuffer;
        RingBuffer* buffer_;
        int index_;
        iterator(RingBuffer* buffer, int idx) : buffer_(buffer), index_(idx) {}
    public:
        T& operator*() { return buffer_->operator[](index_); }
        T* operator->() { return &(buffer_->operator[](index_)); }
        iterator& operator++() {
            --index_;
            if (index_ < 0)
                index_ += buffer_->impl_size_;

            return *this;
        }

        bool operator!= (const iterator& it) const {
            return it.index_ != index_ || it.buffer_ != buffer_;
        }
    };

    iterator begin() {
        int i = head_ - 1;
        if (i < 0)
            i += impl_size_;
        return iterator(this, i);
    }

    iterator end() {
        if (size_ < max_size_) 
            return iterator(this, impl_size_-1);
        else
            return iterator(this, head_);
    }



    class const_iterator {
    private:
        friend class RingBuffer;
        const RingBuffer* buffer_;
        int index_;
        const_iterator(const RingBuffer* buffer, int idx) : buffer_(buffer), index_(idx) {}
    public:
        const T& operator*() { return buffer_->operator[](index_); }
        const T* operator->() { return &(buffer_->operator[](index_)); }
        const_iterator& operator++() {
            --index_;
            if (index_ < 0)
                index_ += buffer_->impl_size_;

            return *this;
        }

        bool operator!= (const const_iterator& it) const {
            return it.index_ != index_ || it.buffer_ != buffer_;
        }
    };

    const_iterator cbegin() const {
        int i = head_ - 1;
        if (i < 0)
            i += impl_size_;
        return const_iterator(this, i);
    }

    const_iterator cend() const {
        if (size_ < max_size_) 
            return const_iterator(this, impl_size_-1);
        else
            return const_iterator(this, head_);
    }


    // Declare standard const iterators to be able to used range-based loops
    const_iterator begin() const {
        int i = head_ - 1;
        if (i < 0)
            i += impl_size_;
        return const_iterator(this, i);
    }

    const_iterator end() const {
        if (size_ < max_size_) 
            return const_iterator(this, impl_size_-1);
        else
            return const_iterator(this, head_);
    }



    void debug() const {
        for (size_t i = 0; i < impl_size_; ++i) 
            std::cout << this->operator[](i) << " ";    
        std::cout << "\n";
        std::cout << "head: " << head_ << "\n";
        std::cout << "size: " << size_ << "\n";
        std::cout << "max_size: " << max_size_ << "\n";
        std::cout << "impl_size: " << impl_size_ << "\n";
    }
};



template<class T, class Allocator> 
std::ostream& operator<<(std::ostream& os, RingBuffer<T, Allocator>& rb) {
    for (const T& v : rb) {
        os << v << " ";
    }

    return os;
}


}
