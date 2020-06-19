#include <stdio.h>
#include <stdlib.h>
#include "lsd.h"

int main(void)
{
  double * image;
  double * out;
  int x,y,i,j,n;
  int X = 128;  /* x image size */
  int Y = 128;  /* y image size */

  /* create a simple image: left half black, right half gray */
  image = (double *) malloc( X * Y * sizeof(double) );
  if( image == NULL )
    {
      mexErrMsgTxt("error: not enough memory\n");
    }
  for(x=0;x<X;x++)
    for(y=0;y<Y;y++)
      image[x+y*X] = x<X/2 ? 0.0 : 64.0; /* image(x,y) */


  /* LSD call */
  out = lsd(&n,image,X,Y);
  /* print output */
  mexPrintf("%d line segments found:\n",n);
  for(i=0;i<n;i++)
    {
      for(j=0;j<7;j++)
        mexPrintf("%f ",out[7*i+j]);
      mexPrintf("\n");
    }

  /* free memory */
  free( (void *) image );
  free( (void *) out );

  return EXIT_SUCCESS;
}
