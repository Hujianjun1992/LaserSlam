#ifndef H_GPC_UTILS
#define H_GPC_UTILS

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_poly.h>

/* The GSL is a pain to work with. The library DOES NOT HAVE a determinant() function
   or an inv() function: you have to write your own routines. */

#define M(matrix, rows, col) gsl_matrix*matrix = gsl_matrix_alloc(rows,col)
#define MF(matrix) gsl_matrix_free(matrix)
#define gmg gsl_matrix_get
#define gms gsl_matrix_set

void m_trans(const gsl_matrix*A, gsl_matrix*A_t);
void m_mult(const gsl_matrix*A, const gsl_matrix*B, gsl_matrix*AB);
void m_add(const gsl_matrix*A, const gsl_matrix*B, gsl_matrix*ApB);
void m_add_to(const gsl_matrix*A, gsl_matrix*B);
void m_scale(double m, gsl_matrix*A);
double m_dot(const gsl_matrix*A,const gsl_matrix*B);
void m_inv(const gsl_matrix*A, gsl_matrix*invA);
double m_det(const gsl_matrix*A);

double poly_greatest_real_root(unsigned int n, double*);
void m_display(const char*str, gsl_matrix*m);

#endif
