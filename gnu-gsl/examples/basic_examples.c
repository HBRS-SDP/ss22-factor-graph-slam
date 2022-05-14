#include<math.h>
#include <stdio.h>
#include <gsl/gsl_sf_bessel.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_eigen.h>


int main (void)
{
// Computing the Bessel function
double x = 5.0;
double y = gsl_sf_bessel_J0 (x);
printf("Example 1 \n");
printf("\n");

printf (" Bessel function value: J0(%g) = %.18e\n", x, y);
printf("\n");
printf("\n");
printf("\n");

printf("Example 2: Computing norm of columns of a matrix\n");
printf("\n");

// Creating a matrix and finding the norms of the columns
size_t i,j;

  gsl_matrix *m = gsl_matrix_alloc (10, 10);

  for (i = 0; i < 10; i++)
    for (j = 0; j < 10; j++)
      gsl_matrix_set (m, i, j, sin (i) + cos (j));

  for (j = 0; j < 10; j++)
    {
      gsl_vector_view column = gsl_matrix_column (m, j);
      double d;

      d = gsl_blas_dnrm2 (&column.vector);
      printf ("matrix column %zu, norm = %g\n", j, d);
    }

  gsl_matrix_free (m);

printf("\n");
printf("\n");
// Computing the eigen values and eigen vectors of a matrix

printf("Example 3: Computing eigen values for the matrix \n");
printf("\n");

double data[] = { 1.0  , 1/2.0, 1/3.0, 1/4.0,
                    1/2.0, 1/3.0, 1/4.0, 1/5.0,
                    1/3.0, 1/4.0, 1/5.0, 1/6.0,
                    1/4.0, 1/5.0, 1/6.0, 1/7.0 };

  gsl_matrix_view m1
    = gsl_matrix_view_array (data, 4, 4);

  gsl_vector *eval = gsl_vector_alloc (4);
  gsl_matrix *evec = gsl_matrix_alloc (4, 4);

  gsl_eigen_symmv_workspace * w =
    gsl_eigen_symmv_alloc (4);

  gsl_eigen_symmv (&m1.matrix, eval, evec, w);

  gsl_eigen_symmv_free (w);

  gsl_eigen_symmv_sort (eval, evec,
                        GSL_EIGEN_SORT_ABS_ASC);

  {
    int i;

    for (i = 0; i < 4; i++)
      {
        double eval_i
           = gsl_vector_get (eval, i);
        gsl_vector_view evec_i
           = gsl_matrix_column (evec, i);

        printf ("eigenvalue = %g\n", eval_i);
        printf ("eigenvector = \n");
        gsl_vector_fprintf (stdout,
                            &evec_i.vector, "%g");
        printf("\n");

      }
  }

  gsl_vector_free (eval);
  gsl_matrix_free (evec);

return 0;
}
