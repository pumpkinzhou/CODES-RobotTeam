/**************************************************************************
 * File Name     : parseNatNetMex.c
 * Author        : Dingjiang Zhou
 *                 Boston University, Boston, 02215
 * Create Time   : Fri, Jul. 19th, 2013. 05:34:44 PM
 * Last Modified : Fri, Aug. 09th, 2013. 01:57:52 AM
 * Purpose       : the C/mex version of the m file parseNatNet.m, 
 *                 in order to speed up the calculation. 
 *                 also, the output data is much less than the original m 
 *                 file. refer to mainParseNatNet.m
 *
 * for how to use this file, refer to parseNatNetMexTest.m
 *
 * sample output (the columns is determined in the code):
 *        rigidBodies.ID  = [1, 2] 
 *        rigidBodies.SE3 = [ 0.2188  1.2643
 *                            0.1066  0.0476
 *                           -0.3380 -0.1724
 *                            0.0066 -0.0019
 *                           -0.2787  0.3082
 *                            0.0000  0.0021
 *                            0.9604  0.9513]
 *
 * Test shows this function is 43 times faster than the original function 
 * written in matlab (in ubuntu 32bit with Intel T5800 processor) and 34 
 * times faster in ubuntu 64bit with Intel i5 processor. the matlab
 * function runs about 1.5ms for each data packet.
 * if delete the code that is not necessary in the m file, the C/Mex 
 * version is still 15 times faster.
 *************************************************************************/
#include "stdlib.h"
#include "mex.h"
#include "math.h"
#include "string.h" /* memcpy() */

/*typedef uchar uchar;*/
/* macro variables */
#define uchar unsigned char
#define MASK_BIT_7 0b01111111
#define MASK_NOT_0 0b00000001
#define MASK_NOT_1 0b00000010
#define MASK_NOT_2 0b00000100
#define MASK_NOT_3 0b00001000
#define MASK_NOT_4 0b00010000
#define MASK_NOT_5 0b00100000
#define MASK_NOT_6 0b01000000
#define MASK_NOT_7 0b10000000

/* variables used in functions */
int     sign;
float   ret;
float   frac[3];
int     bit[8];

/* functions delaration */
float typecast4int8toSingle(uchar dataIn[]);
float bytes2Float(uchar,uchar,uchar,uchar);

void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    int         i,j;
    int         idx;            /* data index */
    int         msgID;          /* message id */
    int         msgLength;      /* message length in the dataIn packet */
    int         byteSize;       /* data size for each group */
    int         numMarkerSets;  /* total marker sets number */
    int         numMarkers;     /* total markers number */
    int         numUnMarkers;   /* un-identified markders */
    int         numRigidBodies; /* rigid body numbers */
    int         numRigidMarkers;/* markers number for each rigid body */
    const char  *fnames[2];    	/* pointers to field names */
    mxArray     *ID;            /* for output */
    mxArray     *SE3;           /* for output */
    double      *IDptr,*SE3ptr; /* a pointer for ID and SE3 */
    
    /* data in */
    uchar *data = mxGetPr(prhs[0]); /* a warning, don't know why */
    int dataSize = mxGetScalar(prhs[1]);
    
    /* output: assign field names	*/
    fnames[0] = (char*)mxMalloc(12);
    fnames[1] = (char*)mxMalloc(16);
    memcpy((char*)fnames[0],"ID",  3);  /* sizeof("ID") = 3 */
    memcpy((char*)fnames[1],"SE3", 4);
    
    /* create a 1x1 struct matrix for output    */
    plhs[0] = mxCreateStructMatrix(1, 1, 2, fnames);
    
    
    
    /* --------------------- parseNatnet main code --------------------- */
    /* message ID/type */
    idx = 0;
    byteSize = 2;
    msgID = data[idx] + 256*data[idx+1];
    if(!(msgID==7))
        printf("Message ID = %d.\n",msgID);
    idx = idx + byteSize;
    
    /* message length */
    byteSize = 2;
    msgLength = data[idx] + 256*data[idx+1];
    if(msgLength > dataSize)
        printf("message length = %d, increase the buffer size before using.\n",msgLength);
    idx = idx + byteSize;
    
    /* frame, never used, skip */
    byteSize = 4;
    idx = idx + byteSize;

    /* number of marketSets, skip the typecasting */
    byteSize = 4;
    /* a simple one, assuming it will never larger than 255 in MSL */ 
    numMarkerSets = data[idx];
    idx = idx + byteSize;
    for(i=0;i<numMarkerSets;i++)
    {
        /* marker sets name like "Rigid Body 1", will never have "0" 
         * inside */
        while(!(data[idx++]==0));
        byteSize = 4;
        numMarkers = data[idx];
        idx = idx + byteSize + numMarkers*12;/* four bytes for one coord */
    }
    
    /* unidentified markers, data not used */
    byteSize = 4;
    numUnMarkers = data[idx] + 
                   data[idx+1]*256 + 
                   data[idx+2]*65536 + 
                   data[idx+3]*16777216;
    idx = idx + byteSize + numUnMarkers*12;
    
    /* number of rigid bodies, assume < 255 */
    byteSize = 4;
    numRigidBodies = data[idx];
    idx = idx + byteSize;
    
    /* creat the space */
    /* if I move this part to the very beginning after the output struct 
     * created, matlab will crash, don't know why */
    ID     = mxCreateDoubleMatrix(1,numRigidBodies,mxREAL);
    IDptr  = mxGetPr(ID);
    SE3    = mxCreateDoubleMatrix(7,numRigidBodies,mxREAL);
    SE3ptr = mxGetPr(SE3);
    
    /* process rigid bodies, assign the values	*/
    for(i=0;i<numRigidBodies;i++)
    {
        byteSize = 4;
        /* suppose rigid bodies number won't larger than 255 */
        IDptr[i] = data[idx]; 
        idx = idx + byteSize;
        for(j=0;j<7;j++)            /* SE3 data */
        {
            /* SE3ptr[i*7+j] = typecast4int8toSingle(data+idx); */
			SE3ptr[i*7+j] = bytes2Float(data[idx],data[idx+1],data[idx+2],data[idx+3]);
            idx = idx + 4;
        } 
        
        /* number of rigid markers */
        byteSize = 4;
        numRigidMarkers = data[idx];
        idx = idx + byteSize;
        
        /* rigid marker data 
        idx = idx + numRigidMarkers*12;*/
        
        /* associated marker Ids 
        byteSize = numRigidMarkers*4; 
        idx = idx + byteSize; */
        
        /* associated marker bytesizes 
        byteSize = numRigidMarkers*4; 
        idx = idx + byteSize; */
        
        /* mean marker error 
        idx = idx + 4; */
        
        /* combine the above four into one */
        idx = idx + numRigidMarkers*20 + 4;
    }
    
    /*Assign the field values                   */
    mxSetFieldByNumber(plhs[0],0,0,ID);
    mxSetFieldByNumber(plhs[0],0,1,SE3);
    
}


/* --------------------------- SUB FUNCTIONS  -------------------------- */
/* typecasting function, only works for converting four int8 numbers into 
 * single-precise float number, NOTICE that the four int8 numbers are in 
 * reverse sequence                                                      */
/* examples: dataIn  = int8([-111,20,96,62])
 *           dataOut = 0.2188285                                         */
/* how it works:
 *    data[3]   |   data[2]   |    data[1]  |   data[0]
 * |    62      |     96      |      20     |   145(== -111)
 *   0011 1110  |  0110 0000  |  0000 1101  |  0001 0001
 * bit31: sign = 0    ---> (-1)^sign = (-1)^0
 * bit30-bit23 = 0111 1100 = (dec)124 ---> 2^(124-127)
 * bit22-bit0: No. 1,2,11,13,16,19,23 are "1"s, ---> 
 *                                        1.f = 1+2^(-1)+2^(-2)+...+2^(-23)
 * then: 
 * dataOut = (-1)^0*2^(124-127)*
 *              (1+2^(-1)+2^(-2)+2^(-11)+2^(-13)+2^(-16)+2^(-19)+2^(-23))
 *         = 0.218828454613686                                           */
/* function designed by Dingjiang */
/* too slow, the conversion should not be done in this way */
float typecast4int8toSingle(uchar dataIn[]) /* -111,20,96,62 */
{
    /* only deal with 4 int8 bytes 
    printf("%d\t%d\t%d\t%d\n",dataIn[0],dataIn[1],dataIn[2],dataIn[3]);*/
    sign = dataIn[3]>>7;
    /* the bit7 in the fourth bytes should be masked */
    ret = pow(-1,sign)*
          pow(2,((dataIn[3] & MASK_BIT_7) << 1) + (dataIn[2] >> 7) - 127);

    /* fraction part */
    int i;
    for(i=0;i<3;i++)
    {
        bit[7] = (dataIn[i] & MASK_NOT_7) >> 7;
        bit[6] = (dataIn[i] & MASK_NOT_6) >> 6;
        bit[5] = (dataIn[i] & MASK_NOT_5) >> 5;
        bit[4] = (dataIn[i] & MASK_NOT_4) >> 4;
        bit[3] = (dataIn[i] & MASK_NOT_3) >> 3;
        bit[2] = (dataIn[i] & MASK_NOT_2) >> 2;
        bit[1] = (dataIn[i] & MASK_NOT_1) >> 1;
        bit[0] = (dataIn[i] & MASK_NOT_0);
        frac[i]= pow(2,-1*bit[7])*bit[7] +
                 pow(2,-2*bit[6])*bit[6] +
                 pow(2,-3*bit[5])*bit[5] +
                 pow(2,-4*bit[4])*bit[4] +
                 pow(2,-5*bit[3])*bit[3] +
                 pow(2,-6*bit[2])*bit[2] +
                 pow(2,-7*bit[1])*bit[1] +
                 pow(2,-8*bit[0])*bit[0];
    }
    
    /* the bit7 in the third bytes is not counted */
    ret = ret*(1 + (frac[2] - pow(2,-1*bit[7])*bit[7])*2 + 
                                  pow(2,-7)*(frac[1] + pow(2,-8)*frac[0]));
    return ret;
}

/* the good function found from the stackoverflow */
/* very fast, since no calculation is needed */
float bytes2Float(uchar b0, uchar b1, uchar b2, uchar b3)
{
	float output;
	*((uchar*)(&output) + 3) = b3;
	*((uchar*)(&output) + 2) = b2;
	*((uchar*)(&output) + 1) = b1;
	*((uchar*)(&output) + 0) = b0;
	return output;
}

/* --------------------------- end of file ----------------------------- */



