// https://stackoverflow.com/questions/8380348/assigning-an-entire-array-with-a-single-statement
// https://en.wikipedia.org/wiki/Matrix_representation

#ifndef __MatrixMath
#define __MatrixMath

#define ROW2    2
#define COL2    2
#define COL1    1

/*
 * Brief: Multiplies two matrices. A*B = C
 *
 * Input:
 *  - rowsA: The amount of rows matrix A has.
 *  - colsA: The amount of columns matrix A has.
 *  - matA:  Matrix A.
 *  - colsB: The amount of columns matrix B has.
 *  - matB:  Matrix B.
 *  - returnSet: Matrix C.
 */
void matrix_multiplication(int rowsA, int colsA, float matA[][colsA], int colsB, float matB[][colsB], float returnSet[][colsB]);

/*
 * Brief: Multiplies a matrix with a vector. x*B = C
 *
 * Input:
 *  - rowsA: The amount of rows matrix A has.
 *  - colsA: The amount of columns matrix A has.
 *  - matA:  Matrix A.
 *  - vector: Vector x.
 *  - returnSet: Matrix C.
 */
void matrix_dot_vector(int rows, int cols, float matA[][cols], float vector, float returnSet[][cols]);

/*
 * Brief: Adds two matrices. A+B = C
 *
 * Input:
 *  - rowsA: The amount of rows matrix A has.
 *  - colsA: The amount of columns matrix A has.
 *  - matA:  Matrix A.
 *  - colsB: The amount of columns matrix B has.
 *  - matB:  Matrix B.
 *  - returnSet: Matrix C.
 */
void matrix_addition(int rows, int cols, float matA[][cols], float matB[][cols], float returnSet[][cols]);

/*
 * Brief: Substracts two matrices. A-B = C
 *
 * Input:
 *  - rowsA: The amount of rows matrix A has.
 *  - colsA: The amount of columns matrix A has.
 *  - matA:  Matrix A.
 *  - colsB: The amount of columns matrix B has.
 *  - matB:  Matrix B.
 *  - returnSet: Matrix C.
 */
void matrix_substraction(int rows, int cols, float matA[][cols], float matB[][cols], float returnSet[][cols]);

/*
 * Brief: Adds a value to all elements in a matrix. A+x = C
 *
 * Input:
 *  - rowsA: The amount of rows matrix A has.
 *  - colsA: The amount of columns matrix A has.
 *  - matA:  Matrix A.
 *  - single: Value x.
 *  - returnSet: Matrix C.
 */
void matrix_additionSingle(int rows, int cols, float matA[][cols], float single, float returnSet[][cols]);

/*
 * Brief: Transposes matrix A. A^T = C
 *
 * Input:
 *  - rows: The amount of rows matrix A has.
 *  - cols: The amount of columns matrix A has.
 *  - matA:  Matrix A
 *  - returnSet: Matrix C.
 */
void matrix_transpose(int rows, int cols, float matA[][cols], float returnSet[][rows]);

/*
 * Brief: Only keeps the diagonal values of a matrix, makes the rest 0. f(A) = C
 *
 * Input:
 *  - rows: The amount of rows matrix A has.
 *  - cols: The amount of columns matrix A has.
 *  - matA:  Matrix A.
 *  - returnSet: Matrix C.
 */
void matrix_keep_diagonal(int rows, int cols, float matA[][cols], float returnSet[][cols]);

/*
 * Brief: Duplicates a matrix. A = C
 *
 * Input:
 *  - rowsA: The amount of rows matrix A has.
 *  - colsA: The amount of columns matrix A has.
 *  - matA:  Matrix A.
 *  - returnSet: Matrix C.
 */
void matrix_duplicate(int rows, int cols, float matA[][cols], float returnSet[][cols]);

/*
 * Brief: Calulates the inverse of a 2x2 matrix. A^-1 = C
 *
 * Input:
 *  - matA:  Matrix A.
 *  - returnSet: Matrix C.
 */
void matrix_inverse(float matA[][COL2], float returnSet[][COL2]);

#endif
