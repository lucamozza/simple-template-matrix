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

#include "matrix.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <type_traits>
#include <complex>

using namespace std;

using cx = complex<float>;

using Scalarf    = MatrixBase<float,1,1>;
using Matrix2f   = MatrixBase<float,2,2>;
using Matrix3f   = MatrixBase<float,3,3>;
using Matrix4f   = MatrixBase<float,4,4>;
using Matrix32f  = MatrixBase<float,3,2>;
using Matrix23f  = MatrixBase<float,2,3>;
using CVector2f  = MatrixBase<float, 2, 1>;

using Matrix2i   = MatrixBase<int,2,2>;

using Matrix2cx  = MatrixBase<cx, 2,2>;
using Matrix23cx = MatrixBase<cx, 2,3>;




bool floatCompare(float a, float b)
{
  float epsilon = 1e-6;
  float minVal = std::min(a, b);
  if ( fabs(a-b)/minVal > epsilon )
    return false;
  return true;
}

bool floatCompare(float a, float b, float epsilon)
{
  float minVal = std::min(a, b);
  if ( fabs(a-b)/minVal > epsilon )
    return false;
  return true;
}

template<unsigned R, unsigned C>
bool compare(const MatrixBase<float,R,C>& a, const MatrixBase<float,R,C>& b)
{
    for(unsigned r = 0; r < R; r++)
        for(unsigned c = 0; c < C; c++)
          if ( !floatCompare(a(r,c),b(r,c)) )
            return false;

    return true;
}

template<typename T>
void checkValueTypeFloat(T t)
{
    static_assert(is_same<typename T::value_type, float>::value,"");
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
      Matrix3f L(0);
      Matrix3f U(0);

      luDecomposition(e33, L, U);
      assert(compare(L, Matrix3f(
        {
          1,   0,                   0,
          4,   1,                   0,
          7,   0.857142857142857,   1
        }
      )));
      assert(compare(U, Matrix3f(
        {
          1,    2,     3,
          0,   -7,    -6,
          0,    0,    -6.857142857142858
        }
      )));
      assert(compare(L*U, e33));


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
        -0.108591466978375,   0.161192285213326,  -0.022443015780245,   0.066627703097604,
         0.204558737580362,  -0.171244886031561,   0.105786090005845,  -0.157802454704851,
        -0.043015780245470,  -0.040561075394506,   0.022326125073057,   0.027469316189363,
         0.039976621858562,   0.021390999415546,  -0.026183518410286,   0.077732320280538
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
    assert(matrixMinor(e33,0,0) == -39);
    assert(matrixMinor(e33,0,1) == -6 );
    assert(matrixMinor(e33,0,2) ==  25);
    assert(matrixMinor(e33,1,0) == -6 );
    assert(matrixMinor(e33,1,1) == -12);
    assert(matrixMinor(e33,1,2) == -6 );
    assert(matrixMinor(e33,2,0) ==  9 );
    assert(matrixMinor(e33,2,1) == -6 );
    assert(matrixMinor(e33,2,2) == -7 );
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


    // *******************
    // *   EIGENVALUES   *
    // *******************
    // Test eig()
    assert(floatCompare(std::get<0>(eig(e1)), 1));

    assert(floatCompare(std::get<0>(eig(e2)), 1));
    assert(floatCompare(std::get<1>(eig(e2)), 1));

    assert(floatCompare(std::get<0>(eig(c22)), 5.3723));
    assert(floatCompare(std::get<1>(eig(c22)), -0.3723));

    Matrix2cx j22({1,1,-1,1});
    assert(floatCompare(std::get<0>(eig(j22)).real(), 1));
    assert(floatCompare(std::get<1>(eig(j22)).imag(), -1));
    assert(floatCompare(std::get<0>(eig(j22)).real(), 1));


    // ***********
    // *   SVD   *
    // ***********
    // Test svd()
    assert(floatCompare(std::get<0>(svd(b23)), 0.514300580658645, 1e-5));
    assert(floatCompare(std::get<1>(svd(b23)), 9.525518091565110));

    assert(floatCompare(std::get<0>(svd(a32)), 0.514300580658645, 1e-5));
    assert(floatCompare(std::get<1>(svd(a32)), 9.525518091565107));


    Matrix23cx k23({cx(1,-1), cx(3,0), cx(5,1), cx(20,0), cx(4,0), cx(6,1)});

    assert(floatCompare(std::get<0>(svd(k23)).real(), 5.180686114607004));
    assert(floatCompare(std::get<1>(svd(k23)).real(), 21.521163801753808));


    cout<<"Tests passed."<<endl;
    return 0;
}
