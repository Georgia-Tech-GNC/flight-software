#include <stdio.h>
//#include "arm_math.h"
#include "../Inc/state_est_helpers.h"


int main(){

    float matA[2][3] = {{2, 2}, {2, 2}};
    float matB[2][2] = {{3, 3}, {3, 3}};

    int rowsA = sizeof(matA) / sizeof(matA[0]);
    int colsA = sizeof(matA[0]) / sizeof(matA[0][0]);

    int rowsB = sizeof(matB) / sizeof(matB[0]);
    int colsB = sizeof(matB[0]) / sizeof(matB[0][0]);

    float *pA = &matA[0][0];
    float *pB = &matB[0][0];

    float matAB[rowsA][colsB];

    matAB = multiply_matrices(*pA, rowsA, colsA, *pB, rowsB, colsB);

    printf("Number of Rows: %d\n ", rowsA);
    printf("Number of Columns: %d\n", colsA);

    return 0;
}

