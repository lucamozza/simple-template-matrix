/*
 * Copyright (c) 2019, Terraneo Federico & Luca Mozzarelli
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <tuple>
#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <ostream>

/**
 * A simple fully template matrix class.
 *
 * You do not want to use MatrixBase directly, but rather use a using
 * declaration to specify its template parameters:
 * \code
 * using Matrix3f = MatrixBase<float,3,3>;
 *
 * Matrix3f myMatrix(0);
 * \endcode
 */
template<typename T, unsigned R, unsigned C>
class MatrixBase
{
    T m[R][C];

public:
    MatrixBase(const MatrixBase&) = default;
    MatrixBase& operator= (const MatrixBase&) = default;

    using value_type = T; ///< value_type trait for STL compatibility

    /**
     * Default constructor.
     * Leaves the matrix uninitialized for performace.
     */
    MatrixBase() {}

    /**
     * Construct a matrix filling every element with the same value
     * \param v value to fill the matrix
     * \code
     * Matrix3f myMatrix(0);
     * \endcode
     */
    explicit MatrixBase(T v)
    {
        for(unsigned r = 0; r < R; r++)
            for(unsigned c = 0; c < C; c++)
                m[r][c] = v;
    }

    /**
     * Construct a matrix from an initializer list
     * \param args initializer list. Its size must be equal to R*C
     * \code
     * Matrix3f myMatrix(
     * {
     *     1.f, 2.f, 3.f,
     *     4.f, 5.f, 6.f,
     *     7.f, 8.f, 9.f
     * });
     * \endcode
     */
    explicit MatrixBase(std::initializer_list<T> args)
    {
        if(args.size() != R*C)
            throw std::range_error("MatrixBase(initializer_list<T> v)");

        auto it = args.begin();
        for(unsigned r = 0; r < R; r++)
            for(unsigned c = 0; c < C; c++)
                m[r][c] = *it++;
    }

    /**
     * Mixed type construction
     * \code
     * MatrixBase<float,2,2> a(0);
     * MatrixBase<int,2,2> b(a);
     * \endcode
     */
    template<typename U>
    MatrixBase(const MatrixBase<U,R,C>& rhs)
    {
        for(unsigned r = 0; r < R; r++)
            for(unsigned c = 0; c < C; c++)
                m[r][c] = rhs(r,c);
    }

    /**
     * Mixed type assignment
     * \code
     * MatrixBase<float,2,2> a(0);
     * MatrixBase<int,2,2> b;
     * b = a;
     * \endcode
     */
    template<typename U>
    MatrixBase& operator= (const MatrixBase<U,R,C>& rhs)
    {
        for(unsigned r = 0; r < R; r++)
            for(unsigned c = 0; c < C; c++)
                m[r][c] = rhs(r,c);
        return *this;
    }

    /**
     * \return the identity matrix
     * \code
     * auto I = Matrix3f::eye();
     * \endcode
     */
    static MatrixBase eye()
    {
        MatrixBase result;
        for(unsigned r = 0; r < R; r++)
            for(unsigned c = 0; c < C; c++)
                result(r,c) = (r == c ? 1 : 0);
        return result;
    }

    /**
     * \return the number of rows of the matrix
     */
    unsigned rows() const { return R; }

    /**
     * \return the number of columns of the matrix
     */
    unsigned cols() const { return C; }

    /**
     * \return the size of the matrix
     * \code
     * auto sz = myMatrix.size();
     * unsigned rows = get<0>(sz);
     * unsigned cols = get<1>(sz);
     * \endcode
     */
    std::tuple<unsigned,unsigned> size() const { return std::make_tuple(R,C); }

    /**
     * Access an element (read-only) (without bound checks)
     * \param r row    (from 0 to R-1)
     * \param c column (from 0 to C-1)
     * \return the corresponding element
     * \code
     * float f = myMatrix(0,0);
     * \endcode
     */
    T operator() (unsigned r, unsigned c) const { return m[r][c]; }

    /**
     * Access an element (read-write) (without bound checks)
     * \param r row    (from 0 to R-1)
     * \param c column (from 0 to C-1)
     * \return the corresponding element
     * \code
     * myMatrix(0,0) = 1.0f;
     * \endcode
     */
    T& operator() (unsigned r, unsigned c) { return m[r][c]; }

    /**
     * Access an element (read-only) (with bound checks)
     * \param r row    (from 0 to R-1)
     * \param c column (from 0 to C-1)
     * \return the corresponding element
     * \throws range_error if r and/or c are not in range
     * \code
     * float f = myMatrix.at(0,0);
     * \endcode
     */
    T at(unsigned r, unsigned c) const
    {
        if(r >= R || c >= C) throw std::range_error("MatrixBase::at()");
        return m[r][c];
    }

    /**
     * Access an element (read-write) (with bound checks)
     * \param r row    (from 0 to R-1)
     * \param c column (from 0 to C-1)
     * \return the corresponding element
     * \throws range_error if r and/or c are not in range
     * \code
     * myMatrix.at(0,0) = 1.0f;
     * \endcode
     */
    T& at(unsigned r, unsigned c)
    {
        if(r >= R || c >= C) throw std::range_error("MatrixBase::at()");
        return m[r][c];
    }
};

/**
 * \code
 * Matrix3f a(0);
 * cout << a << endl;
 * \endcode
 */
template<typename T, unsigned R, unsigned C>
std::ostream& operator<< (std::ostream& os, const MatrixBase<T,R,C>& a)
{
    for(unsigned r = 0; r < R; r++)
    {
        for(unsigned c = 0; c < C; c++)
            os << a(r,c) <<' ';
        os << '\n';
    }
    return os;
}

/**
 * \code
 * Matrix3f a(0), b(0);
 * b = transpose(a);
 * \endcode
 */
template<typename T, unsigned R, unsigned C>
MatrixBase<T,C,R> transpose(const MatrixBase<T,R,C>& a)
{
    MatrixBase<T,C,R> result;
    for(unsigned r = 0; r < R; r++)
        for(unsigned c = 0; c < C; c++)
            result(c,r) = a(r,c);
    return result;
}

/**
 * \code
 * Matrix3f a(0), b(0), c(0);
 * c = a + b;
 * \endcode
 */
template<typename T, typename U, unsigned Ra, unsigned Ca, unsigned Rb, unsigned Cb>
auto operator+ (const MatrixBase<T,Ra,Ca>& a, const MatrixBase<U,Rb,Cb>& b)
    -> MatrixBase<decltype(a(0,0)+b(0,0)),Ra,Ca>
{
    static_assert(Ra == Rb, "matrix row mismatch");
    static_assert(Ca == Cb, "matrix column mismatch");

    MatrixBase<decltype(a(0,0)+b(0,0)),Ra,Ca> result;
    for(unsigned r = 0; r < Ra; r++)
        for(unsigned c = 0; c < Ca; c++)
            result(r,c) = a(r,c) + b(r,c);
    return result;
}

/**
 * \code
 * Matrix3f a(0), c(0);
 * float b = 0;
 * c = a + b;
 * \endcode
 */
template<typename T, typename U, unsigned R, unsigned C>
auto operator+ (const MatrixBase<T,R,C>& a, U b)
    -> MatrixBase<decltype(a(0,0)+b),R,C>
{
    MatrixBase<decltype(a(0,0)+b),R,C> result;
    for(unsigned r = 0; r < R; r++)
        for(unsigned c = 0; c < C; c++)
            result(r,c) = a(r,c) + b;
    return result;
}

/**
 * \code
 * float a = 0;
 * Matrix3f b(0), c(0);
 * c = a + b;
 * \endcode
 */
template<typename T, typename U, unsigned R, unsigned C>
auto operator+ (T a, const MatrixBase<U,R,C>& b)
    -> MatrixBase<decltype(a+b(0,0)),R,C>
{
    MatrixBase<decltype(a+b(0,0)),R,C> result;
    for(unsigned r = 0; r < R; r++)
        for(unsigned c = 0; c < C; c++)
            result(r,c) = a + b(r,c);
    return result;
}

/**
 * \code
 * Matrix3f a(0), b(0);
 * a += b;
 * \endcode
 */
template<typename T, typename U, unsigned Ra, unsigned Ca, unsigned Rb, unsigned Cb>
void operator+= (MatrixBase<T,Ra,Ca>& a, const MatrixBase<U,Rb,Cb>& b)
{
    static_assert(Ra == Rb, "matrix row mismatch");
    static_assert(Ca == Cb, "matrix column mismatch");

    for(unsigned r = 0; r < Ra; r++)
        for(unsigned c = 0; c < Ca; c++)
            a(r,c) += b(r,c);
}

/**
 * \code
 * Matrix3f a(0);
 * float b = 0;
 * a += b;
 * \endcode
 */
template<typename T, typename U, unsigned R, unsigned C>
void operator+= (MatrixBase<T,R,C>& a, U b)
{
    for(unsigned r = 0; r < R; r++)
        for(unsigned c = 0; c < C; c++)
            a(r,c) += b;
}

/**
 * \code
 * Matrix3f a(0), b(0), c(0);
 * c = a - b;
 * \endcode
 */
template<typename T, typename U, unsigned Ra, unsigned Ca, unsigned Rb, unsigned Cb>
auto operator- (const MatrixBase<T,Ra,Ca>& a, const MatrixBase<U,Rb,Cb>& b)
    -> MatrixBase<decltype(a(0,0)-b(0,0)),Ra,Ca>
{
    static_assert(Ra == Rb, "matrix row mismatch");
    static_assert(Ca == Cb, "matrix column mismatch");

    MatrixBase<decltype(a(0,0)-b(0,0)),Ra,Ca> result;
    for(unsigned r = 0; r < Ra; r++)
        for(unsigned c = 0; c < Ca; c++)
            result(r,c) = a(r,c) - b(r,c);
    return result;
}

/**
 * \code
 * Matrix3f a(0), c(0);
 * float b = 0;
 * c = a - b;
 * \endcode
 */
template<typename T, typename U, unsigned R, unsigned C>
auto operator- (const MatrixBase<T,R,C>& a, U b)
    -> MatrixBase<decltype(a(0,0)-b),R,C>
{
    MatrixBase<decltype(a(0,0)-b),R,C> result;
    for(unsigned r = 0; r < R; r++)
        for(unsigned c = 0; c < C; c++)
            result(r,c) = a(r,c) - b;
    return result;
}

/**
 * \code
 * float a = 0;
 * Matrix3f b(0), c(0);
 * c = a - b;
 * \endcode
 */
template<typename T, typename U, unsigned R, unsigned C>
auto operator- (T a, const MatrixBase<U,R,C>& b)
    -> MatrixBase<decltype(a-b(0,0)),R,C>
{
    MatrixBase<decltype(a-b(0,0)),R,C> result;
    for(unsigned r = 0; r < R; r++)
        for(unsigned c = 0; c < C; c++)
            result(r,c) = a - b(r,c);
    return result;
}

/**
 * \code
 * Matrix3f a(0), b(0);
 * a -= b;
 * \endcode
 */
template<typename T, typename U, unsigned Ra, unsigned Ca, unsigned Rb, unsigned Cb>
void operator-= (MatrixBase<T,Ra,Ca>& a, const MatrixBase<U,Rb,Cb>& b)
{
    static_assert(Ra == Rb, "matrix row mismatch");
    static_assert(Ca == Cb, "matrix column mismatch");

    for(unsigned r = 0; r < Ra; r++)
        for(unsigned c = 0; c < Ca; c++)
            a(r,c) -= b(r,c);
}

/**
 * \code
 * Matrix3f a(0);
 * float b = 0;
 * a -= b;
 * \endcode
 */
template<typename T, typename U, unsigned R, unsigned C>
void operator-= (MatrixBase<T,R,C>& a, U b)
{
    for(unsigned r = 0; r < R; r++)
        for(unsigned c = 0; c < C; c++)
            a(r,c) -= b;
}

/**
 * \code
 * Matrix3f a(0), b(0), c(0);
 * c = a * b;
 * \endcode
 */
template<typename T, typename U, unsigned Ra, unsigned Ca, unsigned Rb, unsigned Cb>
auto operator* (const MatrixBase<T,Ra,Ca>& a, const MatrixBase<U,Rb,Cb>& b)
    -> MatrixBase<decltype(a(0,0)*b(0,0)),Ra,Cb>
{
    static_assert(Ca == Rb, "matrix multiply size mismatch");

    MatrixBase<decltype(a(0,0)*b(0,0)),Ra,Cb> result;
    for(unsigned r = 0; r < Ra; r++)
    {
        for(unsigned c = 0; c < Cb; c++)
        {
            result(r,c) = 0;
            for(unsigned x = 0; x < Ca; x++) result(r,c) += a(r,x) * b(x,c);
        }
    }
    return result;
}

/**
 * \code
 * float b = 0;
 * Matrix3f a(0), c(0);
 * c = a * b;
 * \endcode
 */
template<typename T, typename U, unsigned R, unsigned C>
auto operator* (const MatrixBase<T,R,C>& a, U b)
    -> MatrixBase<decltype(a(0,0)*b),R,C>
{
    MatrixBase<decltype(a(0,0)*b),R,C> result;
    for(unsigned r = 0; r < R; r++)
        for(unsigned c = 0; c < C; c++)
            result(r,c) = a(r,c) * b;
    return result;
}

/**
 * \code
 * float a;
 * Matrix3f b(0), c(0);
 * c = a * b;
 * \endcode
 */
template<typename T, typename U, unsigned R, unsigned C>
auto operator* (T a, const MatrixBase<U,R,C>& b)
    -> MatrixBase<decltype(a*b(0,0)),R,C>
{
    MatrixBase<decltype(a*b(0,0)),R,C> result;
    for(unsigned r = 0; r < R; r++)
        for(unsigned c = 0; c < C; c++)
            result(r,c) = a * b(r,c);
    return result;
}

/**
 * \code
 * Matrix3f a(0), b(0);
 * a *= b;
 * \endcode
 */
template<typename T, typename U, unsigned Ra, unsigned Ca, unsigned Rb, unsigned Cb>
void operator*= (MatrixBase<T,Ra,Ca>& a, const MatrixBase<U,Rb,Cb>& b)
{
    static_assert(Ca == Rb, "matrix multiply size mismatch");
    static_assert(Rb == Cb, "matrix multiply size mismatch");

    MatrixBase<decltype(a(0,0)*b(0,0)),Ra,Cb> temp;
    for(unsigned r = 0; r < Ra; r++)
    {
        for(unsigned c = 0; c < Cb; c++)
        {
            temp(r,c) = 0;
            for(unsigned x = 0; x < Ca; x++) temp(r,c) += a(r,x) * b(x,c);
        }
    }
    a = temp;
}

/**
 * \code
 * Matrix3f a(0);
 * float b = 0;
 * a *= b;
 * \endcode
 */
template<typename T, typename U, unsigned R, unsigned C>
void operator*= (MatrixBase<T,R,C>& a, U b)
{
    for(unsigned r = 0; r < R; r++)
        for(unsigned c = 0; c < C; c++)
            a(r,c) *= b;
}

/**
 * Determinant of 1x1 matrix
 * \code
 * Scalarf a(2);
 * float d = det(a);
 * \endcode
 */
template<typename T>
inline T det(const MatrixBase<T,1,1>& a)
{
    return a(0,0);
}

/**
 * Determinant of 2x2 matrix
 * \code
 * Matrix2f a({1,2,3,4});
 * float d = det(a);
 * \endcode
 */
template<typename T>
T det(const MatrixBase<T,2,2>& a)
{
    return a(0,0)*a(1,1)
         - a(0,1)*a(1,0);
}

/**
 * Determinant of 3x3 matrix
 * \code
 * Matrix2f a({1,2,3,4,1,6,7,8,9});
 * float d = det(a);
 * \endcode
 */
template<typename T>
T det(const MatrixBase<T,3,3>& a)
{
    return a(0,0)*a(1,1)*a(2,2)
         + a(0,1)*a(1,2)*a(2,0)
         + a(0,2)*a(1,0)*a(2,1)
         - a(0,2)*a(1,1)*a(2,0)
         - a(0,1)*a(1,0)*a(2,2)
         - a(0,0)*a(1,2)*a(2,1);
}

/**
 * Determinant of 4x4 matrix
 * \code
 * Matrix2f a({1,2,3,4,1,6,7,8,9});
 * float d = det(a);
 * \endcode
 */
template<typename T>
T det(const MatrixBase<T,4,4>& a)
{
    // Decomposing the matrix ito two triangular matrices
    MatrixBase<T,4,4> L(0);
    MatrixBase<T,4,4> U(0);
    luDecomposition(a, L, U);

    // The determinant of a triangular matrix is the product of the elements on
    // its diagonal
    T detL = 1;
    T detU = 1;
    for (unsigned i = 0; i < a.rows(); i++)
    {
        detL = detL * L(i,i);
        detU = detU * U(i,i);
    }

    // The determinant of the product of two matrices is the product of the
    // determinants
    return detL * detU;
}

/**
 * Compute the LU decomposition of a square matrix.
 * This method is not optimized for large matrices.
 * Important: Initialize the lower and upper matrices to zero!
 * Source:
 * https://www.geeksforgeeks.org/doolittle-algorithm-lu-decomposition/
 */
template <typename T, unsigned N>
void luDecomposition(MatrixBase<T,N,N> a, MatrixBase<T,N,N>& lower, MatrixBase<T,N,N>& upper)
{

    // Decomposing matrix into Upper and Lower
    // triangular matrix
    for (unsigned i = 0; i < N; i++)
    {
        // Upper Triangular
        for (unsigned k = i; k < N; k++)
        {
            // Summation of L(i, j) * U(j, k)
            T sum = 0;
            for (unsigned j = 0; j < i; j++)
                sum += (lower(i,j) * upper(j,k));

            // Evaluating U(i, k)
            upper(i,k) = a(i,k) - sum;
        }

        // Lower Triangular
        for (unsigned k = i; k < N; k++)
        {
            if (i == k)
                lower(i,i) = 1;  // Diagonal as 1
            else
            {

                // Summation of L(k, j) * U(j, i)
                T sum = 0;
                for (unsigned j = 0; j < i; j++)
                    sum += (lower(k,j) * upper(j,i));

                // Evaluating L(k, i)
                lower(k,i) = (a(k,i) - sum) / upper(i,i);
            }
        }
    }
}

/**
 * Computes the minor of element a(x,y) for square matrices.
 * Since the determinant is implemented for matrices up to 4x4 this methods works up to 5x5.
 */
template <typename T, unsigned R, unsigned C>
T matrixMinor(const MatrixBase<T,R,C> &a, unsigned x, unsigned y)
{
    static_assert(R == C, "Minor implemented for square matrices only");
    MatrixBase<T,R-1,C-1> minor_matrix;
    for (unsigned i = 0; i < minor_matrix.rows(); i++)
    {
        for (unsigned j = 0; j < minor_matrix.cols(); j++)
        {
            unsigned h, k;
            if (i < x)
                h = i;
            else
                h = i + 1;
            if (j < y)
                k = j;
            else
                k = j + 1;
            minor_matrix(i,j) = a(h,k);
        }
    }
    return det(minor_matrix);
}

/**
 * Computes the cofactor matrix of a square matrix
 * Since the determinant is implemented for matrices up to 4x4 this methods works up to 5x5.
 */
template <typename T, unsigned R, unsigned C>
MatrixBase<T,R,C> cofactorMatrix(const MatrixBase<T,R,C> &a)
{
    static_assert(R == C, "Cofactor matrix implemented only for square matrices");
    MatrixBase<T,R,C> result;
    for (unsigned i = 0; i < a.rows(); i++)
    {
        for (unsigned j = 0; j < a.cols(); j++)
        {
            T min = matrixMinor(a, i, j);
            if ((i+j)%2 == 0) {
                result(i,j) = min;
            }
            else
            {
                result(i,j) = -min;
            }

        }
    }
    return result;
}

/**
 * Inverse of 1x1 matrix. Produces undefined behavior if determinant == 0
 * \param a matrix to invert
 * \param determinant determinant of a, computed using det(a)
 * \retrun 1/a
 * \code
 * Scalarf a(2);
 * auto determinant = det(a);
 * if(fabs(determinant) < 1e-3f) cerr << "Ill-conditioned matrix" << endl;
 * else {
 *     auto b = inv(a, determinant);
 * }
 * \endcode
 */
template<typename T>
MatrixBase<T,1,1> inv(const MatrixBase<T,1,1>& a, T determinant)
{
    (void)determinant;
    return MatrixBase<T,1,1>{1/a(0,0)};
}

/**
 * Inverse of 2x2 matrix. Produces undefined behavior if determinant == 0
 * \param a matrix to invert
 * \param determinant determinant of a, computed using det(a)
 * \retrun 1/a
 * \code
 * Matrix2f a({1,2,3,4});
 * auto determinant = det(a);
 * if(fabs(determinant) < 1e-3f) cerr << "Ill-conditioned matrix" << endl;
 * else {
 *     auto b = inv(a, determinant);
 * }
 * \endcode
 */
template<typename T>
MatrixBase<T,2,2> inv(const MatrixBase<T,2,2>& a, T determinant)
{
    return (1/determinant)*MatrixBase<T,2,2>({
         a(1,1), -a(0,1),
        -a(1,0),  a(0,0)
    });
}

/**
 * Inverse of 3x3 matrix. Produces undefined behavior if determinant == 0
 * \param a matrix to invert
 * \param determinant determinant of a, computed using det(a)
 * \retrun 1/a
 * \code
 * Matrix3f a({1,2,3,4,1,6,7,8,9});
 * auto determinant = det(a);
 * if(fabs(determinant) < 1e-3f) cerr << "Ill-conditioned matrix" << endl;
 * else {
 *     auto b = inv(a, determinant);
 * }
 * \endcode
 */
template<typename T>
MatrixBase<T,3,3> inv(const MatrixBase<T,3,3>& a, T determinant)
{
    T A = a(0,0), B = a(0,1), C = a(0,2),
      D = a(1,0), E = a(1,1), F = a(1,2),
      G = a(2,0), H = a(2,1), I = a(2,2);

    return (1/determinant)*MatrixBase<T,3,3>({
          (E*I - F*H), -(B*I - C*H),  (B*F - C*E),
         -(D*I - F*G),  (A*I - C*G), -(A*F - C*D),
          (D*H - E*G), -(A*H - B*G),  (A*E - B*D)
    });
}

/**
* Inverse of 4x4 matrix. Produces undefined behavior if determinant == 0
* \param a matrix to invert
* \param determinant determinant of a, computed using det(a)
* \retrun 1/a
* \code
* Matrix4f a({1,2,3,4,1,6,7,8,9,10,12,14,16,14,15,16});
* auto determinant = det(a);
* if(fabs(determinant) < 1e-3f) cerr << "Ill-conditioned matrix" << endl;
* else {
*     auto b = inv(a, determinant);
* }
* \endcode
*/
template<typename T>
MatrixBase<T,4,4> inv(const MatrixBase<T,4,4>& a, T determinant)
{
    return transpose(cofactorMatrix(a))/determinant;
}

/**
 * Inverse of a matrix. Throws std::runtime_error if determinant == 0
 * \code
 * Matrix2f a({1,2,3,4,1,6,7,8,9});
 * auto b = inv(a);
 * \endcode
 */
template<typename T, unsigned R, unsigned C>
MatrixBase<T,R,C> inv(const MatrixBase<T,R,C>& a)
{
    static_assert(R == C, "matrix must be square");
    T d = det(a);
    if(d == 0) throw std::runtime_error("matrix singular");
    return inv(a, d);
}

/**
 * \code
 * float b = 0;
 * Matrix3f a(0), c(0);
 * c = a / b;
 * \endcode
 */
template<typename T, typename U, unsigned R, unsigned C>
auto operator/ (const MatrixBase<T,R,C>& a, U b)
    -> MatrixBase<decltype(a(0,0)/b),R,C>
{
    MatrixBase<decltype(a(0,0)/b),R,C> result;
    for(unsigned r = 0; r < R; r++)
        for(unsigned c = 0; c < C; c++)
            result(r,c) = a(r,c) / b;
    return result;
}

/**
 * \code
 * Matrix3f a(0);
 * float b = 0;
 * a /= b;
 * \endcode
 */
template<typename T, typename U, unsigned R, unsigned C>
void operator/= (MatrixBase<T,R,C>& a, U b)
{
    for(unsigned r = 0; r < R; r++)
        for(unsigned c = 0; c < C; c++)
            a(r,c) /= b;
}

/**
 * Eigenvalue of a scalar
 * \code
 * Scalarf a({1});
 * auto d = eig(a);
 * \endcode
 */
template<typename T>
MatrixBase<T,1,1> eig(const MatrixBase<T,1,1>& a)
{
  return a;
}

/**
 * Eigenvalues of 2x2 matrix
 * \code
 * Matrix2f a({1,2,3,4});
 * auto d = eig(a);
 * \endcode
 */
template<typename T>
MatrixBase<T,2,1> eig(const MatrixBase<T,2,2>& a)
{
    MatrixBase<T,2,1> eigs({0,0});
    T root = sqrt(a(0,0)*a(0,0) - 2*a(0,0)*a(1,1) + a(1,1)*a(1,1) + 4*a(0,1)*a(1,0));
    eigs(0,0) = a(0,0)/2 + a(1,1)/2 - root/2;
    eigs(1,0) = a(0,0)/2 + a(1,1)/2 + root/2;
    return eigs;
}
