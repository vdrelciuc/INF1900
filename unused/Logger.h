#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>

using size_t = uint32_t;

/// Specialized container deleting old data as new data is added
template <typename T, size_t N>
class Logger
{
public:
    /// Iterator for the Logger container
    class iterator
    {
    public:
        /// Constructor
        /// \param element              The address of the element pointed by the iterator
        /// \param index                The index of the element in its container. This parameter is
        ///                             necessary, since the iterator must point to the first element
        ///                             of the container the increment result in an out-of-bound iterator.
        /// \param containerCapacity    The capacity of the container containing the element. This parameter is
        ///                             necessary, since the iterator must point to the first element
        ///                             of the container the increment result in an out-of-bound iterator
        /// \param isOutOfBounds        A bool representing if the iterator points to the end of the container.
        ///                             This parameter is necessary, because it is the only way to know if the
        ///                             iterator points to the end of the container since the iterator can't point
        ///                             outside of the container.
        iterator(T* element, size_t index, size_t containerCapacity, bool isOutOfBounds)
            : m_element(element)
            , m_index(index)
            , m_containerCapacity(containerCapacity)
            , m_isOutOfBounds(isOutOfBounds)
            , m_isModified(false)
        {
        }

        /// Addition operator
        /// \return A new iterator incremented by a value
        iterator operator+(size_t number)
        {
            iterator it = iterator(*this);
            return it += number;
        }

        /// Increment operator
        /// \return A reference to the iterator
        iterator& operator++()
        {
            operator+=(1);
            return *this;
        }

        /// Increment operator
        /// \return A copy of the previous value of the iterator
        iterator operator++(int)
        {
            auto temp = this;
            operator+=(1);
            return *temp;
        }

        /// Increment operator
        /// \return A reference to the iterator
        iterator& operator+=(size_t number)
        {
            m_isModified = true;
            m_isOutOfBounds = false;
            size_t newIndex = (m_index + number) % m_containerCapacity;
            m_element += (newIndex - m_index);
            m_index = newIndex;
            return *this;
        }

        /// Subtraction operator
        /// \return A to the iterator
        iterator operator-(size_t number)
        {
            iterator it = iterator(*this);
            return it -= number;
        }

        /// Decrement operator
        /// \return A reference to the iterator
        iterator& operator--()
        {
            operator-=(1);
            return *this;
        }

        /// Decrement operator
        /// \return A copy of the previous value of the iterator
        iterator operator--(int)
        {
            auto temp = this;
            operator-=(1);
            return *temp;
        }

        /// Decrement operator
        /// \return A reference to the iterator
        iterator& operator-=(int32_t number)
        {
            m_isModified = true;
            m_isOutOfBounds = false;
            size_t newIndex;
            if (static_cast<int32_t>(m_index) - number >= 0)
            {
                newIndex = m_index - number;
            }
            else
            {
                newIndex = m_containerCapacity + ((static_cast<int32_t>(m_index) - number) % static_cast<int32_t>(m_containerCapacity));                
            }
            
            m_element += (static_cast<int32_t>(newIndex) - m_index);

            m_index = newIndex;
            return *this;
        }

        /// Dereference operator
        /// \return A reference to the iterator
        T& operator*() const { return *m_element; }

        /// Arrow operator
        /// \return A pointer to the iterator
        T* operator->() const { return m_element; }

        /// == operator
        /// \return A bool representing if the two objects are equal
        bool operator==(const iterator& other)
        {
            return m_element == other.m_element && ((m_isModified == true || other.m_isModified == true) || (m_isOutOfBounds == other.m_isOutOfBounds));
        }

        /// != operator
        /// \return A bool representing if the two objects are not equal
        bool operator!=(const iterator& other) { return !operator==(other); }

    private:
        T* m_element;
        size_t m_index;
        size_t m_containerCapacity;
        bool m_isOutOfBounds;
        // Represents if m_element has been changed since the constructor
        bool m_isModified;
    };

    /// Default constructor
    Logger()
        : m_data{}
        , m_size(0)
        , m_capacity(N)
        , m_lastElementIndex(N - 1)
        , m_firstElementIndex(0)
    {
    }

    /// Returns a non-const reference to the element at specified location pos
    /// \param pos The index of the element
    /// \return A non-const reference to the specified element
    T& operator[](size_t pos) { return m_data[(m_firstElementIndex + pos) % m_capacity]; }

    /// Returns a const reference to the element at specified location pos
    /// \param pos The index of the element
    /// \return A const reference to the specified element
    const T& operator[](size_t pos) const { return m_data[(m_firstElementIndex + pos) % m_capacity]; }

    /// Returns pointer to the underlying array serving as element storage
    /// \return Pointer to the underlying element storage
    const T* data() const { return m_data; }

    /// Returns a non-const reference to the first element in the container. Calling front on an empty container is undefined
    /// \return A non-const reference to the first element of the container
    T& front() { return m_data[m_firstElementIndex]; }

    /// Returns a const reference to the first element in the container. Calling front on an empty container is undefined
    /// \return A const reference to the first element of the container
    const T& front() const { return m_data[m_firstElementIndex]; }

    /// Returns a non-const reference to the last element in the container. Calling back on an empty container is undefined
    /// \return A non-const reference to the last element of the container
    T& back() { return m_data[m_lastElementIndex]; }

    /// Returns a const reference to the last element in the container. Calling back on an empty container is undefined
    /// \return A const reference to the last element of the container
    const T& back() const { return m_data[m_lastElementIndex]; }

    /// Add an element to the container. Warning: if there is no more space for the
    /// new element to be added, the oldest element will be overwritten by the new one
    /// \param element  The element to be added
    void add(T element)
    {
        if (m_capacity == 0)
        {
            return;
        }
        // Increment m_lastElementIndex
        m_lastElementIndex = (m_lastElementIndex + 1) % m_capacity;

        // If array is not full yet
        if (m_size < m_capacity)
        {
            m_size++;
        }
        else
        {
            // Increment m_firstElementIndex
            m_firstElementIndex = (m_firstElementIndex + 1) % m_capacity;
        }
        m_data[m_lastElementIndex] = element;
    }

    /// Erases all elements from the container. After this call, size() returns zero.
    void clear()
    {
        if (m_capacity == 0)
        {
            return;
        }
        m_size = 0;
        m_firstElementIndex = 0;
        m_lastElementIndex = m_capacity - 1;
    }

    /// Assigns the given value value to all elements in the container.
    /// \return The value to assign to the elements 
    void fill(const T& value)
    {
        if (m_capacity == 0)
        {
            return;
        }

        for (auto& log : m_data)
        {
            log = value;
        }
        m_size = m_capacity;
        m_firstElementIndex = 0;
        m_lastElementIndex = m_capacity - 1;
    }

    /// Retrieve the number of element of the container
    /// \return The number of element of the container
    size_t size() const { return m_size; }

    /// Return true if the container is empty
    /// \return true if the container is empty
    bool empty() const { return m_size == 0; }

    /// Retrieve the allocated capacity of the container
    /// \return The allocated capacity of the container
    size_t capacity() const { return m_capacity; }

    /// Retrieve the iterator pointing to the first element
    /// \return An iterator pointing to the element at index 0
    iterator begin()
    {
        return iterator(&m_data[m_firstElementIndex], m_firstElementIndex, m_capacity, false);
    }

    /// Retrieve the iterator pointing just after the last added element
    /// \return An iterator pointing just after the last added element
    iterator end()
    {
        return iterator(&m_data[(m_lastElementIndex + 1) % m_capacity], m_firstElementIndex, m_capacity, m_size == m_capacity);
    }

private:
    T m_data[N];
    size_t m_size;
    const size_t m_capacity;
    size_t m_lastElementIndex;
    size_t m_firstElementIndex;
};

#endif // LOGGER_H