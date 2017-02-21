/*
 * topvalues.h
 *
 *  Created on: 12/apr/2012
 *      Author: Mladen Mazuran
 */

#ifndef TOPVALUES_H_
#define TOPVALUES_H_

/*
    Support class that finds the best N integers (indices) according to the passed cost
    in order of increasing cost value (smaller is better)
*/
template <int N>
class TopValues
{
public:
    TopValues();
    /* Add an integer "index" with cost "value" */
    void add(double value, int index);

    /* Number of best values, less or equal to N, depending on how many indices have been added */
    int count() const;

    /* Best cost at index i < N */
    double value(int i = 0) const;

    /* Best index at index i < N */
    int index(int i = 0) const;

private:
    template<typename S>
    void shift(S *array, int start);

    double values[N];
    int indices[N], maxidx;
};

template <int N>
inline TopValues<N>::TopValues() :
    maxidx(-1)
{
}

template <int N>
inline int TopValues<N>::count() const
{
    return maxidx + 1;
}

template <int N>
inline double TopValues<N>::value(int i) const
{
    return values[i];
}

template <int N>
inline int TopValues<N>::index(int i) const
{
    return indices[i];
}

template <int N> template<typename S>
inline void TopValues<N>::shift(S *array, int start)
{
    for(int i = N - 1; i > start; i--) {
        array[i] = array[i - 1];
    }
}

template <int N>
inline void TopValues<N>::add(double value, int index)
{
    int i;
    for(i = maxidx; i >= 0; i--) {
        if(value > values[i])
            break;
    }
    if(++i < N) {
        shift(values, i);
        shift(indices, i);
        values[i] = value;
        indices[i] = index;
        if(maxidx < N - 1) maxidx++;
    }
}

/* ------------------------------------ N = 1 Specialization ------------------------------------ */
/* Template specialization for slightly more efficient minimum-only search */

template <>
class TopValues<1>
{
public:
    inline TopValues() : first(true), val(INFINITY), idx(-1) {}
    inline void add(double value, int index) {
    	if(value < val) {
    		idx = index;
    		val = value;
    		first = false;
    	}
    }
    inline int count() const { return first ? 0 : 1; }

    inline double value(int i = 0) const { (void) i; return val; }  // (void) avoids unused
    inline int index(int i = 0) const { (void) i; return idx; }     // parameter warnings

private:
    bool first;
    double val;
    int idx;
};

#endif /* TOPVALUES_H_ */
