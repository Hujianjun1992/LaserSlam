/*
 * MedianFilter1D.h
 *
 *  Created on: 02/nov/2009
 *      Author: mallo
 */

#ifndef MEDIANFILTER_H_
#define MEDIANFILTER_H_

#include <vector>
#include <iostream>
#include <algorithm>

template <class dataType>
static bool sort_using_greater_than(dataType u, dataType v) {
    return u > v;
}

//! Median of an array
/*!
 * \param array input array
 * \param windowSize window size
 * \return the Median
 */
template <class dataType>
inline dataType median(
        const dataType array[],
        unsigned int windowSize) {

    std::vector<dataType> v;
    for (unsigned int i = 0; i < windowSize; i++) {
        v.push_back(array[i]);
    }

    // sort
    std::sort(v.begin(), v.end(), sort_using_greater_than<dataType>);

    return v[windowSize / 2];
}


//! Compute the median filter for an array
/*!
 * \param array is the input
 * \param arrayRet is the array returned
 * \param arraySize  is the size of the array
 * \param windowSize is the size of the median filter window
 */
template <class dataType>
void doMedianFilter(
        const dataType array[],
        dataType arrayRet[],
        unsigned int arraySize,
        unsigned int windowSize) {

    unsigned int half = windowSize / 2;

    for (unsigned int i = 0; i < arraySize; ++i){
        if (i < half) { // For the first ones till windowSize/2
            arrayRet[i] = array[i];
        } else if (i > (arraySize - half) ) { // For the last ones till arraySize - windowSize/2
            arrayRet[i] = array[i];
        } else {  // for the central part
            arrayRet[i] = median(array + i - half, windowSize);
        }
    }
}

#endif /* MEDIANFILTER_H_ */
