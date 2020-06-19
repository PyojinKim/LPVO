/*----------------------------------------------------------------------------
  
  Matlab wrapper for LSD

  Copyright (c) 2014 Paul-Darius Sarmadi <paul-darius.sarmadi@telecom-sudparis.eu>

  Based on lsd_cmd.c, a file developed by Rafael Grompone <grompone@gmail.com>

----------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include "mex.h"
#include "../../lsd.h"

/*----------------------------------------------------------------------------*/
/*                                    Matlab Main Function                    */
/*----------------------------------------------------------------------------*/
/** Main Matlab function call */

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
   double scale      = 0.8;
   double sigma_coef = 0.6;
   double quant      = 2.0;
   double ang_th     = 22.5;
   double log_eps    = 0.0;
   double density_th = 0.7;
   int n_bins        = 1024;
   double width      = 1.5;

   double* ptr;
   double* image;
   int X;
   int Y;
   double* segs;
   int n;
   int regX, regY;

   if (!(nlhs==1))
   {
      mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nlhs","One output required.");
   }
   if(nrhs<1)
   {
      mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nrhs","At least one input required.");
   }
   if(nrhs>2)
   {
      mexErrMsgIdAndTxt("MyToolbox:arrayProduct:nrhs","A maximum of two inputs is required.");
   }
   if(nrhs==1 || nrhs==2)
   {
      if (!(mxIsDouble(prhs[0])))
      {
         mexErrMsgTxt("The input must be a double matrix.");
      }
      if (nrhs==2 && (!mxIsStruct(prhs[1])))
      {
         mexErrMsgTxt("The second and optional input must be a structure. Type \"help lsd\" to discover more.");
      }

      if (nrhs==2)
      {
         int tmp=0;
         for( tmp=0; tmp<mxGetNumberOfFields(prhs[1]);tmp++)
         {
            if ( strcmp(mxGetFieldNameByNumber(prhs[1],tmp),"scale")==0)
            {
               if (!(mxIsDouble(mxGetField(prhs[1],0,mxGetFieldNameByNumber(prhs[1],0)))))
               {
                  mexErrMsgTxt("A double argument was expected.");
               }
               if (mxGetNumberOfElements((mxGetField(prhs[1],0,mxGetFieldNameByNumber(prhs[1],0))))!=1)
               {
                  mexErrMsgTxt("Only one value was expected.");
               }
               ptr=mxGetPr(mxGetField(prhs[1],0,mxGetFieldNameByNumber(prhs[1],tmp)));
               if(ptr[0]>=0)
               {
                  scale=ptr[0];
                  mexPrintf("ptr[0]=%e et scale=%e",ptr[0],scale);
               }
               else
               {
                  mexErrMsgTxt("The scale value is not acceptable. For more information, type \"help lsd\"");
               }
            }

            if ( strcmp(mxGetFieldNameByNumber(prhs[1],tmp),"sigma_coef")==0)
            {
               if (!mxIsDouble(mxGetField(prhs[1],0,mxGetFieldNameByNumber(prhs[1],0))))
               {
                  mexErrMsgTxt("A double argument was expected.");
               }
               if (!mxGetNumberOfElements((mxGetField(prhs[1],0,mxGetFieldNameByNumber(prhs[1],0)))))
               {
                  mexErrMsgTxt("Only one value was expected.");
               }
               ptr=mxGetPr(mxGetField(prhs[1],0,mxGetFieldNameByNumber(prhs[1],tmp)));
               if(ptr[0]>=0)
               {
                  sigma_coef=ptr[0];
               }
               else
               {
                  mexErrMsgTxt("The sigma_coef value is not acceptable. For more information, type \"help lsd\"");
               }
            }

            if ( strcmp(mxGetFieldNameByNumber(prhs[1],tmp),"quant")==0)
            {
               if (!mxIsDouble(mxGetField(prhs[1],0,mxGetFieldNameByNumber(prhs[1],0))))
               {
                  mexErrMsgTxt("A double argument was expected.");
               }
               if (!mxGetNumberOfElements((mxGetField(prhs[1],0,mxGetFieldNameByNumber(prhs[1],0)))))
               {
                  mexErrMsgTxt("Only one value was expected.");
               }
               ptr=mxGetPr(mxGetField(prhs[1],0,mxGetFieldNameByNumber(prhs[1],tmp)));
               if(ptr[0]>=0)
               {
                  quant=ptr[0];
               }
               else
               {
                  mexErrMsgTxt("The quant value is not acceptable. For more information, type \"help lsd\"");
               }
            }

            if ( strcmp(mxGetFieldNameByNumber(prhs[1],tmp),"ang_th")==0)
            {
               if (!mxIsDouble(mxGetField(prhs[1],0,mxGetFieldNameByNumber(prhs[1],0))))
               {
                  mexErrMsgTxt("A double argument was expected.");
               }
               if (!mxGetNumberOfElements((mxGetField(prhs[1],0,mxGetFieldNameByNumber(prhs[1],0)))))
               {
                  mexErrMsgTxt("Only one value was expected.");
               }
               ptr=mxGetPr(mxGetField(prhs[1],0,mxGetFieldNameByNumber(prhs[1],tmp)));
               if(ptr[0]>=0 && ptr[0]<=180)
               {
                  ang_th=ptr[0];
               }
               else
               {
                  mexErrMsgTxt("The ang_th value is not acceptable. For more information, type \"help lsd\"");
               }
            }

            if ( strcmp(mxGetFieldNameByNumber(prhs[1],tmp),"log_eps")==0)
            {
               if (!mxIsDouble(mxGetField(prhs[1],0,mxGetFieldNameByNumber(prhs[1],0))))
               {
                  mexErrMsgTxt("A double argument was expected.");
               }
               if (!mxGetNumberOfElements((mxGetField(prhs[1],0,mxGetFieldNameByNumber(prhs[1],0)))))
               {
                  mexErrMsgTxt("Only one value was expected.");
               }
               ptr=mxGetPr(mxGetField(prhs[1],0,mxGetFieldNameByNumber(prhs[1],tmp)));
               log_eps=ptr[0];
            }

            if ( strcmp(mxGetFieldNameByNumber(prhs[1],tmp),"n_bins")==0)
            {
               if (!mxIsDouble(mxGetField(prhs[1],0,mxGetFieldNameByNumber(prhs[1],0))))
               {
                  mexErrMsgTxt("A double argument was expected.");
               }
               if (!mxGetNumberOfElements((mxGetField(prhs[1],0,mxGetFieldNameByNumber(prhs[1],0)))))
               {
                  mexErrMsgTxt("Only one value was expected.");
               }
               ptr=mxGetPr(mxGetField(prhs[1],0,mxGetFieldNameByNumber(prhs[1],tmp)));
               if(ptr[0]>=1)
               {
                  n_bins=ptr[0];
               }
               else
               {
                  mexErrMsgTxt("The n_bins value is not acceptable. For more information, type \"help lsd\"");
               }
            }

         }
      }
      image=mxGetPr(prhs[0]);
      X=mxGetM(prhs[0]);
      Y=mxGetN(prhs[0]);

      /* execute LSD */
      segs = LineSegmentDetection( &n, image, X, Y,
            scale,
            sigma_coef,
            quant,
            ang_th,
            log_eps,
            density_th,
            n_bins,
            NULL,
            &regX, &regY );

      /* The output created here will be an array */
      {
         double* pointeur;
         int dim = 7;
         int j=0,z=0;

         plhs[0]=mxCreateDoubleMatrix(n,dim,mxREAL);
         pointeur=(double*)mxGetPr(plhs[0]);
         for(z=0;z<n;z++)
         {
            for(j=0;j<dim;j++)
            {
               pointeur[z+j*n]=segs[z*dim+j];
            }
         }
         z=0;
         j=0;
         for(z=0;z<n;z++)
         {
            for(j=0;j<dim;j++)
            {
               double c = pointeur[z+3*n];
               pointeur[z+3*n]=pointeur[z];
               pointeur[z]=c;
               c=pointeur[z+2*n];
               pointeur[z+2*n]=pointeur[z+n];
               pointeur[z+n]=c;
            }
         }
      }
   }
}
