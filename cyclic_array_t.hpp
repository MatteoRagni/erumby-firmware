#ifndef CYCLIC_ARRAY_T_HPP
#define CYCLIC_ARRAY_T_HPP

/**
 * \file cyclic_array_t.hpp
 * \author Matteo Ragni
 *
 * Implementations of a cyclic array. It is possible to push back
 * in the array losing the first elements. It does not use allocation
 * but only a statically sized array.
 */

#include <Arduino.h>

/** \brief Cyclic array implementation
 *
 * The Cyclic array implementation allows to have a queue of elements
 * for which the user can push back elements, but without using
 * allocation.
 *
 * \tparam T type contained in the internal array
 * \tparam N size of the internal array
 */
template < typename T, size_t N >
class cyclic_array_t {
  T data[N];     /**< Actual data */
  size_t offset; /**< Current offset for data */

  /** \brief Returns the data index from the rolling index
   *
   * The functions evaluates the actual position in \p data
   * by using a remainder operation. If on \p AVR (e.g. Arduino)
   * the function returns an index even if the parameter is out
   * of bound.
   * For compilers capable of handling exceptions, it will raise
   * an \p std::out_of_bound exception for index greter than size.
   *
   * \param idx required index on the cyclic array
   * \return the internal array index
   */
  inline size_t index(size_t idx) const {
    return (offset + idx) % size;
  }

 public:
  const size_t size; /**< Constant representing the size of the cyclic array */

  /** \brief Empty constructor */
  cyclic_array_t() : offset(0), size(N) {  }
  /** \brief Filling constructor
   *
   * The constructor fills the underlay array with data included
   * as first parameter.
   *
   * \param value element for filling the array
   */
  cyclic_array_t(T value) : offset(0), size(N) { fill(value); }
  /** \brief Copy constructor
   *
   * Creates a new cycling array making a copy of an existing one
   * \param other the origin array
   */
  cyclic_array_t(const cyclic_array_t< T, N >& other) : size(N) { copy(other); }

  /** \brief Append element to the end (overwriting the first one)
   *
   * The push back function overwrites the first element of the
   * array then moves the offset in order to keep track of the head of the
   * cycling array.
   *
   * \param value the value to append
   * \return the current instance
   */
  cyclic_array_t< T, N >& push_back(T value) {
    this->front() = value;
    offset = (offset + 1) % size;
    return *this;
  }

  /** \brief makes a copy of another array on the current array
   *
   * The copy method copies another array on the current instance.
   * The type ensures the compatibility (is two array are different it raises
   * an error compile time).
   *
   * \param other array to be copied
   * \return the current instance
   */
  cyclic_array_t< T, N >& copy(const cyclic_array_t< T, N >& other) {
    if (this == &other)
      return *this;
    offset = other.offset;
    for (size_t i = 0; i < size; i++)
      (*this)[i] = other[i];
    return *this;
  }

  /** \brief makes a copy of another array on the current array
   *
   * The \p = operatorn copies another array on the current instance.
   * The type ensures the compatibility (is two array are different it raises
   * an error compile time).
   *
   * \param other array to be copied
   * \return the current instance
   */
  cyclic_array_t< T, N >& operator=(const cyclic_array_t< T, N >& other) { return copy(other); }

  /** \brief First element of the cycling array
   *
   * Return the element in the position 0 of the cycling array
   *
   * \return the element in front (cannot be changed)
   */
  const T& front() const { return (*this)[0]; }
  /** \brief Reference to the first element of the cycling array
   *
   * Return the element in the position 0 of the cycling array. The
   * returned element may be modified.
   *
   * \return the element in front
   */
  T& front() { return (*this)[0]; }
  /** \brief Last element of the cycling array
   *
   * Return the element in the tail (last element) of the cycling array
   *
   * \return the element in the tail (cannot be changed)
   */
  const T& back() const { return (*this)[size - 1]; }
  /** \brief Reference to the last element of the cycling array
   *
   * Return the element in the tail (last element) of the cycling array.
   * The returned element may be modified.
   *
   * \return the element in the tail
   */
  T& back() { return (*this)[size - 1]; }
  /** \brief Access an element of the cycling array
   *
   * Return an element in the cycling array through an index.
   * The returned element cannot be modified.
   *
   * \param idx position of the element
   * \return the element in the position required (constant)
   */
  const T& operator[](size_t idx) const { return data[index(idx)]; }

  /** \brief Access the reference of an element of the cycling array
   *
   * Return an element in the cycling array through an index.
   * The returned element cannot be modified.
   *
   * \param idx position of the element
   * \return the element in the position required (constant)
   */
  T& operator[](size_t idx) { return data[index(idx)]; }

  /** \brief Array filler
   *
   * The method fills the underlay array with data included
   * as first parameter.
   *
   * \param value element for filling the array
   * \return the reference to the filled array (\p this)
   */
  cyclic_array_t< T, N >& fill(T value) {
    for (size_t i = 0; i < size; i++)
      (*this)[i] = value;
    return (*this);
  }
};


#endif /* CYCLIC_ARRAY_T_HPP */
