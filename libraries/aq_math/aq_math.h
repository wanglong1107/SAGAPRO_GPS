/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011-2014  Bill Nesbitt
*/

#ifndef _aq_math_h
#define _aq_math_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdint.h>
#include <math.h>


#ifndef M_PI
#define M_PI			3.14159265f
#define M_PI_2			(M_PI / 2.0f)
#endif

#define RAD_TO_DEG		(180.0f / M_PI)
#define DEG_TO_RAD		(M_PI / 180.0f)

#define GRAVITY			9.80665f	// m/s^2

#define AQ_US_PER_SEC		1000000


#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef LROUNDF
#define LROUNDF(Xx) ((Xx) >= 0.0f ? (long)((Xx) + 0.5f) : (long)((Xx) - 0.5f))
#endif

/**
* @brief 32-bit floating-point type definition.
*/
typedef float float32_t;

typedef enum
{
	ARM_MATH_SUCCESS = 0,                /**< No error */
	ARM_MATH_ARGUMENT_ERROR = -1,        /**< One or more arguments are incorrect */
	ARM_MATH_LENGTH_ERROR = -2,          /**< Length of data buffer is incorrect */
	ARM_MATH_SIZE_MISMATCH = -3,         /**< Size of matrices is not compatible with the operation. */
	ARM_MATH_NANINF = -4,                /**< Not-a-number (NaN) or infinity is generated */
	ARM_MATH_SINGULAR = -5,              /**< Generated by matrix inversion if the input matrix is singular and cannot be inverted. */
	ARM_MATH_TEST_FAILURE = -6           /**< Test Failed  */
} arm_status;

typedef struct
{
	uint16_t numRows;     /**< number of rows of the matrix.     */
	uint16_t numCols;     /**< number of columns of the matrix.  */
	float32_t *pData;     /**< points to the data of the matrix. */
} arm_matrix_instance_f32;

/**
* @brief  Floating-point square root function.
* @param[in]  in     input value.
* @param[out] *pOut  square root of input value.
* @return The function returns ARM_MATH_SUCCESS if input value is positive value or ARM_MATH_ARGUMENT_ERROR if
* <code>in</code> is negative value and returns zero output for negative values.
*/

static inline arm_status arm_sqrt_f32(float32_t in,float32_t * pOut)
{
	if(in > 0)
	{
		*pOut = sqrtf(in);
		return (ARM_MATH_SUCCESS);
	}
	else
	{
		*pOut = 0.0f;
		return (ARM_MATH_ARGUMENT_ERROR);
	}
}

// first order quat filter
typedef struct {
    float tc;
    float qz1[4];
} quatFilter_t;


void arm_mat_init_f32(arm_matrix_instance_f32 * S, uint16_t nRows, uint16_t nColumns, float32_t * pData);
void arm_fill_f32(float32_t value, float32_t * pDst, uint32_t blockSize);
arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 * pSrcA,const arm_matrix_instance_f32 * pSrcB,arm_matrix_instance_f32 * pDst);
arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 * pSrc,arm_matrix_instance_f32 * pDst);
void arm_std_f32( float32_t * pSrc, uint32_t blockSize, float32_t * pResult);



extern int matrixInit(arm_matrix_instance_f32 *m, int rows, int cols);
extern void matrixFree(arm_matrix_instance_f32 *m);
extern void matrixDump(char *name, arm_matrix_instance_f32 *m);
extern int qrDecompositionT_f32(arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *Q, arm_matrix_instance_f32 *R);
extern void matrixDiv_f32(arm_matrix_instance_f32 *X, arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *B, arm_matrix_instance_f32 *Q, arm_matrix_instance_f32 *R, arm_matrix_instance_f32 *AQ);
extern void quatMultiply(float32_t *qr, float32_t *q1, float32_t *q2);
extern void eulerToQuatYPR(float32_t *q, float32_t yaw, float32_t pitch, float32_t roll);
extern void eulerToQuatRPY(float32_t *q, float32_t roll, float32_t pitch, float32_t yaw);
extern void vectorNormalize(float32_t *v, int n);
extern void nlerp(float32_t *r, float32_t *a, float32_t *b, float32_t t);
extern void quatFilterReset(quatFilter_t *f, float32_t *q);
extern void quatFilterReset3(quatFilter_t *f, float32_t *q);
extern void quatFilterInit(quatFilter_t *f, float32_t dt, float32_t tau, float32_t *q);
extern void quatFilterInit3(quatFilter_t *f, float32_t dt, float32_t tau, float32_t *q);
extern float32_t *quatFilter(quatFilter_t *f, float32_t *b);
extern float32_t *quatFilter3(quatFilter_t *f, float32_t *b);
extern int cholF(float32_t *U);
extern void svd(float32_t *A, float32_t *S2, int n);


#ifdef __cplusplus
}
#endif

#endif //_aq_math_h
