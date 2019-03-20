A simple all-template matrix class for C++
==========================================

This project is useful when you just need to do some simple matrix computations in C++, and you don't want to add a complex framework such as Armadillo or Eigen as a dependency.

Pros:

* Header-only (actually, everything is in a single header).
* No dynamic memory allocation for the matrix data.
* Compile-time dimensions checking for all matrix operations.
* Supports operations on mixed types, such as the sum of a matrix of ```int``` and a mmatrix of ```float```.
* Modern C++ with no external dependencies.
* Suited for embedded/microcontroller environments (C++11 support needed, though).

Cons:

* Not optimized for large matrices. Consider using Armadillo or Eigen in this case.

Getting started
===============

## Declare your matrix types

The ```matrix.h``` header provides the template ```MatrixBase<T,R,C>``` class.
It is recommended to explicitly define the matrix types you need based on this class.

```
#include "matrix.h"

using Matrix2f  = MatrixBase<float,3,3>; // 2x2 square matrix
using Matrix3f  = MatrixBase<float,3,3>; // 3x3 square matrix
using Matrix32f = MatrixBase<float,3,2>; // 3x2 matrix (3 rows, 2 columns)
using Matrix23f = MatrixBase<float,2,3>; // 2x3 matrix (2 rows, 3 columns)
using RVector3f = MatrixBase<float,3,1>; // 3x1 row vector
using CVector3f = MatrixBase<float,1,3>; // 1x3 column vector
```

## Constructors

The default constructor leaves the matrix unintialized. To fill each matrix element with a single value, just pass it to the constructor.

```
/*
 * myMatrix =
 * 0 0 0
 * 0 0 0
 * 0 0 0
 */
Matrix3f myMatrix(0);
```

Initializer-lists are used to initialize a matrix to an arbitrary value.

```
Matrix3f myMatrix(
{
    1, 2, 3,
    4, 5, 6,
    7, 8, 9
});
```

## Accessors

```rows()``` and ```cols()``` return the matrix dimensions. ```size()``` returning a tuple can be used instead.

```
unsigned int rows = myMatrix.rows();
unsigned int cols = myMatrix.cols();
auto sz = myMatrix.size();
assert(rows == get<0>(sz));
assert(cols == get<1>(sz));
```

Individual elements of the matrix can be accessed using either ```operator()``` or ```at()```. ```operator()``` does not perform bound checking, while ```at()``` throws ```std::range_error``` if the access is out of bounds.

```
float f = myMatrix(0,0);
myMatrix(0,0) = 15;

myMatrix(100,100);    // Undefined behavior
myMatrix.at(100,100); // throws std::range_error
```

A matrix can be printed with ```operator<<``` on an ostream.

```
cout << myMatrix << endl;
```

### Operations

Operator overloading provides matrix operations.

```
Matrix32f a(0), b(0), c(0);

Matrix23f d = transpose(a);

c = a + b;
c = a + 1;

c += a;
c += 1;

c = a - b;
c = a - 1;

c -= a;
c -= 1;

Matrix3f e = a * d;
c = a * 2;

e *= Matrix2f(2);
c = a * 2;

c = a / 2;
c /= 2;

```

Matrix inverse and determinant are supported only for 2x2 and 3x3 matrices. This is done _on purpose_. 4x4 matrices support could be added (patches welcome), but that's about the limit where complex algorithms are needed or performance drops significantly. If you need to invert large matrices, it probably means you have outgrown this simple matrix library and need to switch to a full-fledged linear algebra library. Consider using Armadillo or Eigen.

```
Matrix3f f(
{
    1, 2, 3,
    4, 5, 6,
    7, 8, 9
});

const float threshold = 1e-3f;
if(det(f) < threshold) cerr << "Ill-conditioned matrix" << endl;

auto g = inv(f); // Throws std::runtime_error if matrix singular
```