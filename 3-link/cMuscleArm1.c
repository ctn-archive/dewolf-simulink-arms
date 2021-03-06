/***************************************************
 * DO NOT EDIT MANUALLY!
 * Automatically generated by Maple.
 * Created On: Mon Feb 07 20:33:06 2011.
***************************************************/

#define S_FUNCTION_NAME cMuscleArm1
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "math.h"

/***************************************************
* Variable Definition for System:

* State variable(s):
*    x[ 0] = `Main.MuscleArm1.DFPSubsys1inst.theta_R1`(t)
*    x[ 1] = diff(`Main.MuscleArm1.DFPSubsys1inst.theta_R1`(t),t)
*    x[ 2] = `Main.MuscleArm1.DFPSubsys1inst.theta_R2`(t)
*    x[ 3] = diff(`Main.MuscleArm1.DFPSubsys1inst.theta_R2`(t),t)
*    x[ 4] = `Main.MuscleArm1.DFPSubsys1inst.theta_R3`(t)
*    x[ 5] = diff(`Main.MuscleArm1.DFPSubsys1inst.theta_R3`(t),t)
*
* Output variable(s):
*    y[ 0] = `Main.MuscleArm1.DFPSubsys1inst.theta_R2`(t)
*    y[ 1] = `Main.MuscleArm1.DFPSubsys1inst.theta_R1`(t)
*    y[ 2] = `Main.MuscleArm1.DFPSubsys1inst.theta_R3`(t)
*    y[ 3] = `Main.'MuscleArm1::emR1'`(t)
*    y[ 4] = `Main.'MuscleArm1::emR2'`(t)
*    y[ 5] = `Main.'MuscleArm1::emR3'`(t)
*
* Input variable(s):
*    u[ 0] = `Main.'MuscleArm1::ElbowTorque'`(t)
*    u[ 1] = `Main.'MuscleArm1::ShoulderTorque'`(t)
*    u[ 2] = `Main.'MuscleArm1::WristTorque'`(t)
*
************************************************/

/* Fixed parameters */
#define NDIFF 6
#define NEQ 30
#define NPAR 0
#define NINP 3
#define NDISC 0
#define NIX1 24
#define NOUT 6
#define NCON 0
#define NEVT 0
#ifdef EVTHYST
#define NZC 2*NEVT
#else
#define NZC 2*NEVT
#endif

static real_T dsn_zero=0.0;
static unsigned char dsn_undefC[8] = { 0, 0, 0, 0, 0, 0, 0xF0, 0x7F };
static real_T *dsn_undef = (real_T *)&dsn_undefC;
static unsigned char dsn_posinfC[8] = { 0, 0, 0, 0, 0, 0, 0xF0, 0xFF };
static real_T *dsn_posinf = (real_T *)&dsn_posinfC;
static unsigned char dsn_neginfC[8] = { 0, 0, 0, 0, 0, 0, 0xF8, 0x7F };
static real_T *dsn_neginf = (real_T *)&dsn_neginfC;
#define trunc(v) ( (v>0.0) ? floor(v) : ceil(v) )


void DecompCInc(int_T n, real_T *A, int_T Ainc, int_T *ip)
{
	int_T i,j,k,m;
	real_T t;

	ip[n-1]=1;
	for(k=0;k<n-1;k++) {
		m=k;
		for(i=k+1;i<n;i++)
			if( fabs(A[i*Ainc+k])>fabs(A[m*Ainc+k]) ) m=i;
		ip[k]=m;
		if( m!=k ) ip[n-1]=-ip[n-1];
		t=A[m*Ainc+k]; A[m*Ainc+k]=A[(Ainc+1)*k]; A[(Ainc+1)*k]=t;
		if( t==0.0 ) { ip[n-1]=0; return; }
		t=-1.0/t;
		for(i=k+1;i<n;i++) A[i*Ainc+k]=A[i*Ainc+k]*t;
		for(j=k+1;j<n;j++) {
			t=A[m*Ainc+j]; A[m*Ainc+j]=A[k*Ainc+j]; A[k*Ainc+j]=t;
			if( t!=0.0 )
				for(i=k+1;i<n;i++) A[i*Ainc+j]+=A[i*Ainc+k]*t;
		}
	}
	if(A[(n-1)*(Ainc+1)]==0.0) ip[n-1]=0;
}
void DecompC(int_T n, real_T *A, int_T *ip) { DecompCInc(n,A,n,ip); }


void SolveCInc(int_T n, real_T *A, int_T Ainc, int_T *ip, real_T *b)
{
	int_T i,j,m;
	real_T t;

	if( n>1 ) {
		for(j=0;j<n-1;j++) {
			m=ip[j];
			t=b[m]; b[m]=b[j]; b[j]=t;
			for(i=j+1;i<n;i++) b[i]+=A[i*Ainc+j]*t;
		}
		for(j=n-1;j>0;j--) {
			b[j]=b[j]/A[(Ainc+1)*j];
			t=-b[j];
			for(i=0;i<=j-1;i++) b[i]+=A[i*Ainc+j]*t;
		}
	}
	b[0]=b[0]/A[0];
}
void SolveC(int_T n, real_T *A, int_T *ip, real_T *b) { SolveCInc(n,A,n,ip,b); }


void fp(int_T N, real_T T, real_T *Y, real_T *YP)
{
	real_T M[9], V[3], Z[34];
	int_T P[3], ti1, ti2;

	YP[0] = Y[1];
	YP[2] = Y[3];
	YP[4] = Y[5];
	M[0] = 2.;
	V[0] = -(1.+tanh(314159.265358979324*Y[0]-955544.51357303793));
	Y[8] = V[0]/M[0];
	M[0] = 0.001;
	V[0] = Y[1]+0.1*Y[0];
	Y[9] = V[0]/M[0];
	M[0] = 1.;
	V[0] = Y[8]*Y[9];
	Y[10] = V[0]/M[0];
	M[0] = 2.;
	V[0] = -(1.+tanh(-31415.9265358979324-314159.265358979324*Y[0]));
	Y[6] = V[0]/M[0];
	M[0] = 0.001;
	V[0] = Y[1]+0.1*Y[0];
	Y[7] = V[0]/M[0];
	M[0] = 1.;
	V[0] = Y[10]+Y[6]*Y[7];
	Y[21] = V[0]/M[0];
	M[0] = 2.;
	V[0] = -(1.+tanh(314159.265358979324*Y[2]-955544.51357303793));
	Y[13] = V[0]/M[0];
	M[0] = 0.001;
	V[0] = Y[3]+0.1*Y[2];
	Y[14] = V[0]/M[0];
	M[0] = 1.;
	V[0] = Y[13]*Y[14];
	Y[15] = V[0]/M[0];
	M[0] = 2.;
	V[0] = -(1.+tanh(-31415.9265358979324-314159.265358979324*Y[2]));
	Y[11] = V[0]/M[0];
	M[0] = 0.001;
	V[0] = Y[3]+0.1*Y[2];
	Y[12] = V[0]/M[0];
	M[0] = 1.;
	V[0] = Y[15]+Y[11]*Y[12];
	Y[22] = V[0]/M[0];
	M[0] = 2.;
	V[0] = -(1.+tanh(314159.265358979324*Y[4]-955544.51357303793));
	Y[18] = V[0]/M[0];
	M[0] = 0.001;
	V[0] = Y[5]+0.1*Y[4];
	Y[19] = V[0]/M[0];
	M[0] = 1.;
	V[0] = Y[18]*Y[19];
	Y[20] = V[0]/M[0];
	M[0] = 2.;
	V[0] = -(1.+tanh(-31415.9265358979324-314159.265358979324*Y[4]));
	Y[16] = V[0]/M[0];
	M[0] = 0.001;
	V[0] = Y[5]+0.1*Y[4];
	Y[17] = V[0]/M[0];
	M[0] = 1.;
	V[0] = Y[20]+Y[16]*Y[17];
	Y[23] = V[0]/M[0];
	for(ti1=1;ti1<=3;ti1++)
		for(ti2=1;ti2<=3;ti2++)
			M[(ti1-1)*3+ti2-1] = 0.;
	for(ti1=1;ti1<=3;ti1++)
		V[ti1-1] = 0.;
	Z[0] = cos(Y[0]);
	Z[1] = cos(Y[2]);
	Z[2] = cos(Y[4]);
	Z[3] = sin(Y[2]);
	Z[4] = sin(Y[4]);
	Z[5] = Z[1]*Z[2]-Z[3]*Z[4];
	Z[6] = sin(Y[0]);
	Z[2] = Z[3]*Z[2]+Z[1]*Z[4];
	Z[4] = Z[0]*Z[5]-Z[6]*Z[2];
	Z[7] = 0.225*Z[1];
	Z[8] = 0.025*Z[5];
	Z[9] = Z[7]+Z[8]+0.31;
	Z[10] = -0.025*Z[2]-0.225*Z[3];
	Z[11] = Z[10]*Z[6];
	Z[12] = Z[9]*Z[0]+Z[11];
	Z[13] = Z[6]*Z[5]+Z[0]*Z[2];
	Z[9] = Z[10]*Z[0]-Z[9]*Z[6];
	M[0] = 0.001+0.00875*(Z[4]*Z[12]-Z[13]*Z[9]);
	Z[7] = (Z[7]+Z[8])*Z[0]+Z[11];
	Z[8] = -(Z[6]*Z[1]+Z[0]*Z[3]);
	Z[10] = 0.025*Z[13];
	Z[11] = -Z[10]+0.225*Z[8];
	M[1] = 0.001+0.00875*(Z[4]*Z[7]-Z[13]*Z[11]);
	Z[14] = 0.025*Z[4];
	M[2] = 0.001+0.00875*(Z[4]*Z[14]+Z[13]*Z[10]);
	Z[15] = Y[1]+Y[3];
	Z[16] = -Z[8];
	Z[17] = Y[1]+Y[3]+Y[5];
	Z[18] = Y[1]*Y[1];
	Z[17] = Z[17]*Z[17];
	Z[15] = Z[15]*Z[15];
	Z[19] = 0.225*Z[15];
	Z[20] = 0.31*Z[18];
	Z[21] = -(Z[20]*Z[6]+Z[19]*Z[16]+Z[10]*Z[17]);
	Z[22] = Z[0]*Z[1]-Z[6]*Z[3];
	Z[19] = -(Z[20]*Z[0]+Z[19]*Z[22]+Z[14]*Z[17]);
	V[0] = Y[29]+Y[23]+0.00875*(Z[13]*Z[19]-Z[4]*Z[21]);
	Z[1] = 0.19755*Z[1];
	Z[5] = 0.00875*Z[5];
	Z[20] = -(Z[1]+Z[5])-0.5177;
	Z[2] = 0.00875*Z[2]+0.19755*Z[3];
	Z[3] = Z[6]*Z[2];
	Z[23] = Z[20]*Z[0]+Z[3];
	Z[2] = Z[0]*Z[2];
	Z[20] = Z[2]-Z[20]*Z[6];
	Z[13] = 0.00875*Z[13];
	Z[24] = -Z[13]-0.04725*Z[16];
	Z[4] = 0.00875*Z[4];
	Z[25] = Z[4]+0.04725*Z[22];
	Z[26] = 0.09*(Z[16]*Z[20]-Z[22]*Z[23]);
	Z[12] = Z[12]*Z[25];
	Z[9] = Z[24]*Z[9];
	M[3] = Z[9]+Z[26]+Z[12]+0.013;
	Z[27] = -(Z[1]+Z[5])*Z[0]+Z[3];
	Z[8] = Z[13]-0.19755*Z[8];
	Z[28] = 0.09*(Z[16]*Z[8]-Z[22]*Z[27]);
	Z[7] = Z[7]*Z[25];
	Z[11] = Z[24]*Z[11];
	M[4] = Z[11]+Z[28]+Z[7]+0.013;
	Z[29] = 0.09*(Z[22]*Z[4]+Z[16]*Z[13]);
	Z[14] = Z[14]*Z[25];
	Z[10] = -Z[24]*Z[10];
	M[5] = Z[10]+Z[29]+Z[14]+0.001;
	Z[30] = Z[13]*Z[17];
	Z[15] = 0.19755*Z[15];
	Z[31] = Z[15]*Z[16];
	Z[32] = 0.5177*Z[18];
	Z[33] = Z[32]*Z[6]+Z[31]+Z[30];
	Z[17] = Z[4]*Z[17];
	Z[15] = Z[15]*Z[22];
	Z[32] = Z[32]*Z[0]+Z[15]+Z[17];
	Z[19] = Z[19]*Z[24];
	Z[16] = 0.09*(Z[22]*Z[33]-Z[16]*Z[32]);
	Z[21] = Z[25]*Z[21];
	V[1] = Z[16]+Y[27]+Y[22]-Z[21]-Z[19];
	Z[1] = -(Z[1]+Z[5])-0.81685;
	M[6] = Z[9]+Z[26]+Z[12]+0.155*(-(Z[23]+Z[0]*Z[1]+Z[3])*Z[0]-(Z[6]*Z[1]-Z[20]-Z[2])*Z[6])+0.0271;
	M[7] = Z[11]+Z[28]+Z[7]+0.013+0.31*(Z[8]*Z[6]-Z[27]*Z[0]);
	M[8] = Z[10]+Z[29]+Z[14]+0.001+0.31*(Z[4]*Z[0]+Z[13]*Z[6]);
	Z[1] = 0.81685*Z[18];
	V[2] = Z[16]+Y[28]+Y[21]-Z[21]-Z[19]+0.155*((Z[33]+Z[6]*Z[1]+Z[31]+Z[30])*Z[0]-(Z[32]+Z[0]*Z[1]+Z[15]+Z[17])*Z[6]);
	DecompCInc(3,M,3,P);
	SolveCInc(3,M,3,P,V);
	YP[1] = V[0];
	YP[3] = V[1];
	YP[5] = V[2];
}

void otp(real_T T, real_T *Y, real_T *YP)
{
	Y[24] = -Y[21];
	Y[25] = -Y[22];
	Y[26] = -Y[23];
}

static void mdlInitializeSizes(SimStruct *S)
{
	ssSetNumSFcnParams(S, 1);
	if( ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S) ) return;
	ssSetNumContStates(S, NDIFF);
	ssSetNumDiscStates(S, 0);

	if( !ssSetNumInputPorts(S, 1) ) return;
	ssSetInputPortWidth(S, 0, NINP);
	ssSetInputPortDirectFeedThrough(S, 0, 1);

	if( !ssSetNumOutputPorts(S, 1) ) return;
	ssSetOutputPortWidth(S, 0, NOUT);

	ssSetNumSampleTimes(S, 1);
	ssSetNumRWork(S, 1+2*NEQ+NPAR+NDIFF+NEVT);
	ssSetNumIWork(S, 2*NEVT+2*NZC);
	ssSetNumPWork(S, 0);
	ssSetNumModes(S, 0);
	ssSetNumNonsampledZCs(S, 0);

	ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
	ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
	ssSetOffsetTime(S, 0, 0.0);
	ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

static void InitializeConditions(SimStruct *S, int_T useinputs)
{
	real_T *w = ssGetRWork(S);
	real_T t = ssGetT(S);
	real_T *ic = mxGetData(ssGetSFcnParam(S,0));
	real_T *x = ssGetContStates(S);
	InputRealPtrsType i;
	int_T j;

	w[0]=0.0;
	for(j=0;j<NDIFF;j++) {
		x[j]=ic[j];
		w[j+1]=x[j];
	}
	w[7] =  0.00000000000000000e+00;
	w[8] =  0.00000000000000000e+00;
	w[9] =  0.00000000000000000e+00;
	w[10] =  0.00000000000000000e+00;
	w[11] =  0.00000000000000000e+00;
	w[12] =  0.00000000000000000e+00;
	w[13] =  7.85398163397448315e+01;
	w[14] =  0.00000000000000000e+00;
	w[15] =  7.85398163397448315e+01;
	w[16] =  0.00000000000000000e+00;
	w[17] =  0.00000000000000000e+00;
	w[18] =  7.85398239999999959e+01;
	w[19] =  0.00000000000000000e+00;
	w[20] =  7.85398239999999959e+01;
	w[21] =  0.00000000000000000e+00;
	w[22] =  0.00000000000000000e+00;
	w[23] =  0.00000000000000000e+00;
	w[24] =  0.00000000000000000e+00;
	w[25] = -0.00000000000000000e+00;
	w[26] = -0.00000000000000000e+00;
	w[27] = -0.00000000000000000e+00;
	w[28] =  1.00000000000000000e+00;
	w[29] =  1.00000000000000000e+00;
	w[30] =  1.00000000000000000e+00;
	if(useinputs) {
		i = ssGetInputPortRealSignalPtrs(S,0);
		for(j=0; j<NINP; j++) w[j+NDIFF+NIX1-NINP+1]=i[j][0];
	}
	if(NIX1>0) fp(NEQ,w[0],&w[1],&w[NEQ+NPAR+1]);
}

#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S)
{
	InitializeConditions(S,0);
}

static real_T *GetUpdatedWork(SimStruct *S)
{
	int_T j,flag;

	real_T *w = ssGetRWork(S);
	real_T t = ssGetT(S);
	real_T *x = ssGetContStates(S);
	InputRealPtrsType i = ssGetInputPortRealSignalPtrs(S,0);

	flag=0;
	if( w[0]!=t ) { flag=1; w[0]=t; }
	for(j=0; j<NDIFF; j++) if( w[j+1]!=x[j] ) { flag=1; w[j+1]=x[j]; }
	for(j=0; j<NINP; j++) if( w[j+NDIFF+NIX1-NINP+1]!=i[j][0] ) { flag=1; w[j+NDIFF+NIX1-NINP+1]=i[j][0]; }

	if(flag) fp(NEQ,w[0],&w[1],&w[NEQ+NPAR+1]);

	return(w);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
	real_T *w, *y = ssGetOutputPortRealSignal(S,0);

	w=GetUpdatedWork(S);
	otp(w[0],&w[1],&w[NEQ+NPAR+1]);
	y[ 0]=w[ 3];
	y[ 1]=w[ 1];
	y[ 2]=w[ 5];
	y[ 3]=w[25];
	y[ 4]=w[26];
	y[ 5]=w[27];
}

#define MDL_DERIVATIVES
static void mdlDerivatives(SimStruct *S)
{
	real_T *w,*dx = ssGetdX(S);
	int_T i;

	w=GetUpdatedWork(S);
	for(i=0;i<NDIFF;i++) dx[i]=w[NEQ+NPAR+1+i];
}

static void mdlTerminate(SimStruct *S) { UNUSED_ARG(S); }

#ifdef  MATLAB_MEX_FILE    /* Being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
