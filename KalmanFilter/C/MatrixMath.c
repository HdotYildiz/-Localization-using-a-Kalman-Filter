#include "MatrixMath.h"
#include <stdio.h>

void matrix_multiplication(int rowsA, int colsA, float matA[][colsA], int colsB, float matB[][colsB], float returnSet[][colsB])
{
    int i, j, k;
    // Clear returnSet first
    for (i = 0; i < rowsA; i++)
    {
        for(j = 0; j < colsB; j++) 
        {
            returnSet[i][j] = 0;
        }
    }

    for (i = 0; i < rowsA; i++) {
        for(j = 0; j < colsB; j++) {
            for(k = 0; k < colsA; k++) {
                returnSet[i][j] += matA[i][k] * matB[k][j];
            }
            //printf("%f\n", returnSet[i][j]);
        }
    }
}

// Multiply by number
void matrix_dot_vector(int rows, int cols, float matA[][cols], float vector, float returnSet[][cols])
{
    int i, j;
    for (i = 0; i < rows; i++) 
    {
        for (j = 0 ; j < cols; j++) 
        {
            returnSet[i][j] = matA[i][j] * vector;
        }
    }
}

void matrix_addition(int rows, int cols, float matA[][cols], float matB[][cols], float returnSet[][cols])
{
    int i, j;
    for (i = 0; i < rows; i++) {

        for (j = 0 ; j < cols; j++) 
        {
            returnSet[i][j] = matA[i][j] + matB[i][j];
            //printf("Addition: %f\n", returnSet[i][j]);
        }
    }
}

void matrix_substraction(int rows, int cols, float matA[][cols], float matB[][cols], float returnSet[][cols])
{
    int i, j;
    for (i = 0; i < rows; i++) 
    {
        for (j = 0 ; j < cols; j++) 
        {
            returnSet[i][j] = matA[i][j] - matB[i][j];
            //printf("Substraction: %f\n", returnSet[i][j]);
        }
    }
}

void matrix_additionSingle(int rows, int cols, float matA[][cols], float single, float returnSet[][cols])
{
    int i, j;
    for (i = 0; i < rows; i++) 
    {
        for (j = 0 ; j < cols; j++) 
        {
            returnSet[i][j] = matA[i][j] + single;
            //printf("Single addition: %f\n", returnSet[i][j]);
        }
    }
}

void matrix_transpose(int rows, int cols, float matA[][cols], float returnSet[][rows])
{
    int i, j;
    for (i = 0; i < rows; i++) 
    {
        for (j = 0 ; j < cols; j++) 
        {
          //printf("Transpose: %f\n", returnSet[j][i]);
            returnSet[j][i] = matA[i][j];
        }
    }
}

void matrix_keep_diagonal(int rows, int cols, float matA[][cols], float returnSet[][cols])
{
    int i, j;
    for (i = 0; i < rows; i++) 
    {
        for (j = 0 ; j < cols; j++) 
        {
            if( i == j )
                returnSet[i][j] = matA[i][j];
            else
                returnSet[i][j] = 0;
            //printf("Diagonal: %f, %d - %d\n", returnSet[i][j]);
        }
    }
}

void matrix_duplicate(int rows, int cols, float matA[][cols], float returnSet[][cols])
{
    int i, j;
    for (i = 0; i < rows; i++) 
    {
        for (j = 0 ; j < cols; j++) 
        {
            returnSet[i][j] = matA[i][j];
        }
    }
}

// Get inverse of a 2x2 Matrix, "hardcoded" for now. Zero's can become negative
void matrix_inverse(float matA[][COL2], float returnSet[][COL2])
{
    //fprintf(f, "Determinant: ( %f , %f ) - ( %f , %f ) = %f\n", matA[0][0], matA[1][1], matA[0][1], matA[1][0], 1 / ((matA[0][0] * matA[1][1]) - (matA[0][1] * matA[1][0])));
    float determinant = 1 / ((matA[0][0] * matA[1][1]) - (matA[0][1] * matA[1][0]));

    returnSet[0][0] = matA[1][1] * determinant;
    returnSet[0][1] = matA[0][1] * -determinant;
    returnSet[1][0] = matA[1][0] * -determinant;
    returnSet[1][1] = matA[0][0] * determinant;
}
