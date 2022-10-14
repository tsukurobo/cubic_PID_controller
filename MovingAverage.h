#pragma once
#include <Arduino.h>

template <class TYPE>
class Moving_average
{
private:
    //! Number of values used for smoothing. The default value shall be 10.
    unsigned int smoothingReadings;

    //! Array of values used for smoothing
    TYPE *values;

    //! Index of the array
    unsigned int index;

    //! Sum of the values used for smoothing
    TYPE sum;

public:
    /**
     * @brief Construct a new Moving Average object
     *
     * @param initialValue Initial value of the moving average. The default value shall be 0.
     * @param smoothingReadings Number of values used for smoothing. The default value shall be 10.
     */
    Moving_average(const TYPE initialValue = 0, const unsigned int smoothingReadings = 10);

    /**
     * @brief Destroy the Moving Average object
     *
     */
    ~Moving_average();

    /**
     * @brief Add a new value to the array and return the average of the values
     *
     * @param value
     */
    void add(const TYPE value);

    /**
     * @brief Get the average of the values
     *
     * @return TYPE
     */
    TYPE get() const;

    /**
     * @brief Initialize
     *
     * @param initialValue
     * @param smoothingReadings
     */
    void initialize(TYPE initialValue, unsigned int smoothingReadings);

    void initialize(TYPE initialValue);
};

template <class TYPE>
Moving_average<TYPE>::Moving_average(const TYPE initialValue, const unsigned int smoothingReadings)
{
    this->smoothingReadings = max(1, smoothingReadings);
    this->values = new TYPE[this->smoothingReadings];
    this->index = 0;
    this->sum = 0;
    if (initialValue != 0)
    {
        for (unsigned int i = 0; i < smoothingReadings; i++)
        {
            this->values[i] = initialValue;
        }
        this->sum = initialValue * smoothingReadings;
    }
}

template <class TYPE>
Moving_average<TYPE>::~Moving_average() {}

template <class TYPE>
void Moving_average<TYPE>::add(const TYPE value)
{
    sum -= values[index];
    values[index] = value;
    sum += values[index];
    index = (index + 1) % smoothingReadings;
}

template <class TYPE>
TYPE Moving_average<TYPE>::get() const
{
    return sum / smoothingReadings;
}

template <class TYPE>
void Moving_average<TYPE>::initialize(const TYPE initialValue,const unsigned int smoothingReadings)
{
    this->smoothingReadings = max(1, smoothingReadings);
    this->values = new TYPE[this->smoothingReadings];
    this->initialize(initialValue);
}

template <class TYPE>
void Moving_average<TYPE>::initialize(const TYPE initialValue)
{
    this->index = 0;
    this->sum = 0;
    if (initialValue != 0)
    {
        for (unsigned int i = 0; i < this->smoothingReadings; i++)
        {
            this->values[i] = initialValue;
        }
        this->sum = initialValue * this->smoothingReadings;
    }
}