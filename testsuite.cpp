/*
 * Copyright (c) 2019, Terraneo Federico
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

#include "matrix.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <type_traits>

using namespace std;

using Scalarf   = MatrixBase<float,1,1>;
using Matrix2f  = MatrixBase<float,2,2>;
using Matrix3f  = MatrixBase<float,3,3>;
using Matrix4f  = MatrixBase<float,4,4>;
using Matrix32f = MatrixBase<float,3,2>;
using Matrix23f = MatrixBase<float,2,3>;

using Matrix2i  = MatrixBase<int,2,2>;

template<unsigned R, unsigned C>
bool compare(const MatrixBase<float,R,C>& a, const MatrixBase<float,R,C>& b)
{
  #warning changed epsilon value
    float epsilon = 1e-4;

    for(unsigned r = 0; r < R; r++)
        for(unsigned c = 0; c < C; c++)
            if(fabs(a(r,c)-b(r,c))>epsilon) return false;
    return true;
}

template<typename T>
void checkValueTypeFloat(T t)
{
    static_assert(is_same<typename T::value_type, float>::value,"");
}

bool floatCompare(float a, float b)
{
  #warning epsilon value
  float epsilon = 1e-3;
  return fabs(a-b) < epsilon;
}

int main()
{
    // ********************
    // *   CONSTRUCTORS   *
    // ********************
    // Test scalar constructor and initializer_list constructor
    assert(compare(Matrix2f(1), Matrix2f({1,1,1,1})));

    // Test eye matrix constructors
    auto e1 = Scalarf::eye();
    assert(e1(0,0) == 1);
    auto e2 = Matrix2f::eye();
    assert(e2(0,0) == 1);
    assert(e2(1,1) == 1);
    assert(e2(0,1) == 0);
    assert(e2(1,0) == 0);

    Matrix32f a32(
    {
        1, 2,
        3, 4,
        5, 6
    });

    // Test rows(), cols(), size()
    assert(a32.rows() == 3);
    assert(a32.cols() == 2);
    auto sz = a32.size();
    assert(get<0>(sz) == 3);
    assert(get<1>(sz) == 2);



    // *****************
    // *   OPERATORS   *
    // *****************
    // Test operator() and at()
    assert(a32(0,0) == 1);
    assert(a32(0,1) == 2);
    assert(a32(1,0) == 3);
    assert(a32(1,1) == 4);
    assert(a32(0,0) == a32.at(0,0));
    assert(a32(0,1) == a32.at(0,1));
    assert(a32(1,0) == a32.at(1,0));
    assert(a32(1,1) == a32.at(1,1));
    a32(0,0) = 10;
    assert(a32(0,0) == 10);
    a32.at(0,0) = 1;
    assert(a32(0,0) == 1);
    a32(0,1) = 10;
    assert(a32(0,1) == 10);
    a32.at(0,1) = 2;
    assert(a32(0,1) == 2);
    a32(1,0) = 10;
    assert(a32(1,0) == 10);
    a32.at(1,0) = 3;
    assert(a32(1,0) == 3);
    a32(1,1) = 10;
    assert(a32(1,1) == 10);
    a32.at(1,1) = 4;
    assert(a32(1,1) == 4);
    try {
        a32.at(0,2) = 0;
        assert(false);
    } catch(range_error& e) {}
    try {
        a32.at(3,0) = 0;
        assert(false);
    } catch(range_error& e) {}

    Matrix2f c22(
    {
        1, 2,
        3, 4
    });
    Matrix32f d32;

    // Test operator+
    assert(compare(a32 + a32, Matrix32f({2,4,6,8,10,12})));
    assert(compare(a32 + 1, Matrix32f({2,3,4,5,6,7})));
    assert(compare(1 + a32, Matrix32f({2,3,4,5,6,7})));

    // Test operator+=
    d32 = a32;
    d32 += a32;
    assert(compare(d32, Matrix32f({2,4,6,8,10,12})));
    d32 = a32;
    d32 += 1;
    assert(compare(d32, Matrix32f({2,3,4,5,6,7})));

    // Test operator-
    assert(compare(a32 - a32, Matrix32f(0)));
    assert(compare(a32 - 1, Matrix32f({0,1,2,3,4,5})));
    assert(compare(1 - a32, Matrix32f({0,-1,-2,-3,-4,-5})));

    // Test operator-=
    d32 = a32;
    d32 -= a32;
    assert(compare(d32, Matrix32f(0)));
    d32 = a32;
    d32 -= 1;
    assert(compare(d32, Matrix32f({0,1,2,3,4,5})));

    // Test operator*
    assert(compare(a32 * c22, Matrix32f({7,10,15,22,23,34})));
    assert(compare(a32 * 2, Matrix32f({2,4,6,8,10,12})));
    assert(compare(2 * a32, Matrix32f({2,4,6,8,10,12})));

    // Test operator*=
    d32 = a32;
    d32 *= c22;
    assert(compare(d32, Matrix32f({7,10,15,22,23,34})));
    d32 = a32;
    d32 *= 2;
    assert(compare(d32, Matrix32f({2,4,6,8,10,12})));

    // Test operator/
    assert(compare(a32 / 2, Matrix32f({0.5,1,1.5,2,2.5,3})));

    // Test operator/=
    d32 = a32;
    d32 /= 2;
    assert(compare(d32, Matrix32f({0.5,1,1.5,2,2.5,3})));



    // *****************
    // *   TRANSPOSE   *
    // *****************
    // Test transpose
    Matrix23f b23 = transpose(a32);
    assert(compare(b23, Matrix23f({1,3,5,2,4,6})));



    // *******************
    // *   DETERMINANT   *
    // *******************
    // Test det()
    Matrix3f e33(
    {
        1,2,3,
        4,1,6,
        7,8,9
    });
    Scalarf f11(-4);
    assert(det(c22) == -2);
    assert(det(e33) == 48);
    assert(det(f11) == -4);

    Matrix4f i44(
      {
        1,    3,    -5,   7,
        8,    3,    -5,   1,
        14,   10,   15,   3,
        2,    1,  9,      10
      });
      assert(floatCompare(det(i44),-8555));

      // Test LU decomposition
      Matrix3f L,U;
      luDecomposition(e33, L, U);
      assert(compare(L, Matrix3f(
        {
          1,   0,         0,
          4,   1,         0,
          7,   0.8571,    1
        }
      )));
      assert(compare(U, Matrix3f(
        {
          1,    2,     3,
          0,   -7,    -6,
          0,    0,    -6.8571
        }
      )));



    // ***************
    // *   INVERSE   *
    // ***************
    // Test inv()
    assert(compare(inv(f11), Scalarf(-0.25)));
    assert(compare(inv(f11), inv(f11,det(f11))));
    assert(compare(inv(c22), Matrix2f({-2,1,1.5,-0.5})));
    assert(compare(inv(c22), inv(c22,det(c22))));
    assert(compare(inv(e33), Matrix3f(
    {
        -0.8125,       0.125,  0.1875,
        0.125,        -0.25,   0.125,
        0.5208333731,  0.125, -0.1458333433
    })));
    assert(compare(inv(e33), inv(e33,det(e33))));
    assert(compare(inv(i44), Matrix4f(
      {
        -0.1086,   0.1612,  -0.0224,   0.0666,
         0.2046,  -0.1712,   0.1058,  -0.1578,
        -0.0430,  -0.0406,   0.0223,   0.0275,
         0.0400,   0.0214,  -0.0262,   0.0777
      })));
    assert(compare(inv(i44), inv(i44,det(i44))));

    // Test cofactor matrix and minors
    assert(compare(cofactorMatrix(e33), Matrix3f(
      {
        -39,   6,    25,
         6,   -12,   6,
         9,    6,   -7
      }
    )));
    assert(minor(e33,0,0) == -39);
    assert(minor(e33,0,1) == -6 );
    assert(minor(e33,0,2) ==  25);
    assert(minor(e33,1,0) == -6 );
    assert(minor(e33,1,1) == -12);
    assert(minor(e33,1,2) == -6 );
    assert(minor(e33,2,0) ==  9 );
    assert(minor(e33,2,1) == -6 );
    assert(minor(e33,2,2) == -7 );
    assert(compare(cofactorMatrix(c22), Matrix2f(
      {
        4, -3,
        -2, 1
      }
    )));



    // ******************
    // *   MIXED TYPE   *
    // ******************
    // Test mixed type operations's correct type deduction through decltype
    Matrix2i ci22 = Matrix2i::eye();
    auto g22 = c22 + ci22;
    checkValueTypeFloat(g22);

    // Test mixed type in-place operation
    c22 += ci22;

    // Test mixed type assignment
    c22 = ci22;
    assert(c22(0,0) == 1);
    assert(c22(1,1) == 1);
    assert(c22(0,1) == 0);
    assert(c22(1,0) == 0);
    c22 = Matrix2f(
    {
        1, 2,
        3, 4
    });

    // Test mixed type construction
    Matrix2i h22(c22);
    assert(c22(0,0) == 1);
    assert(c22(0,1) == 2);
    assert(c22(1,0) == 3);
    assert(c22(1,1) == 4);

    cout<<"Tests passed."<<endl;
    return 0;
}
