/*
 * MobileAverage.h
 *
 *  Created on: 30/ott/2009
 *      Author: mallo
 */

#ifndef MOBILEAVERAGE_H_
#define MOBILEAVERAGE_H_

#define MOBILE_AVERAGE_BODY(fun)                                                                \
    unsigned int half = windowSize / 2;                                                         \
                                                                                                \
    for (unsigned int i=0; i<arraySize; ++i) {                                                  \
        if (i < half) { /* For the first ones till windowSize/2 */                              \
            arrayRet[i] = array[i];                                                             \
        } else if (i > (arraySize - half)) { /* For the last ones till sizeA - windowSize/2 */  \
            arrayRet[i] = array[i];                                                             \
        } else {  /* for the central part */                                                    \
            arrayRet[i] = fun;                                                                  \
        }                                                                                       \
    }

//! Mean/Average of an array
/*!
 * \param array
 * \param windowSize window size
 * \return the mean/average
 */
template <class dataType>
inline dataType mean(const dataType array[], unsigned int windowSize) {
      dataType sum = 0;

      for (unsigned int i=0; i < windowSize; i++) {
         sum += array[i];
      }

      return sum / windowSize;
}

//! Mean/Average of an array
/*!
 * \param array
 * \param arrayWeight
 * \param idxMax last index
 * \return the mean/average
 */
template <class dataType>
inline dataType mean(const dataType array[], const dataType arrayWeight[], unsigned int windowSize) {
      dataType sum = 0;

      for (unsigned int i=0; i < windowSize; i++) {
         sum += array[i] * arrayWeight[i];
      }

      return sum /* /num */;
}

//! Compute the mobile average for an array
/*!
 * \param array is the input
 * \param arrayRet is the array returned
 * \param arraySize is the size of the array
 * \param windowSize is the size of arrayWeight
 * \param arrayWeight is n array s.t. sum(arrayWeight[i])=1 that specify the weights
 */
template <class dataType>
void doMobileAverage(
        const dataType array[],
        dataType arrayRet[],
        unsigned int arraySize,
        unsigned int windowSize,
        const dataType arrayWeight[]) {

    MOBILE_AVERAGE_BODY(mean(array + i - half, arrayWeight, windowSize))
}


//! Compute the mobile average for an array
/*!
 * Note that arrayWeight is uniform -> arrayWeight[i] = 1/NumOfWeight
 * \param array is the input
 * \param arrayRet is the array returned
 * \param arraySize is the size of the array
 * \param windowSize is the size of arrayWeight
 */
template <class dataType>
void doMobileAverage(
        const dataType array[],
        dataType arrayRet[],
        unsigned int arraySize,
        unsigned int windowSize) {

    MOBILE_AVERAGE_BODY(mean(array + i - half, windowSize))
}

#undef MOBILE_AVERAGE_BODY

#endif /* MOBILEAVERAGE_H_ */
