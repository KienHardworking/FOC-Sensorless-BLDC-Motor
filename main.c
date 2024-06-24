/*
 * main.c
 *
 *  Created on: 2023 Aug 28 13:45:36
 *  Author: dell
 */




#include "DAVE.h"     //Declarations from DAVE Code Generation (includes SFR declaration)
#include "math.h"
/**

 * @brief main() - Application entry point
 *
 * <b>Details of function</b><br>
 * This routine is the application entry point. It is invoked by the device startup code. It is responsible for
 * invoking the APP initialization dispatcher routine - DAVE_Init() and hosting the place-holder for user application
 * code.
 */
XMC_VADC_RESULT_SIZE_t current_phaseU,current_phaseV;
#define IsqRef_OPENLOOP (float)0.3
#define END_SPEED_RADS_PER_SEC_ELEC_IN_LOOPTIME (float)((float)((float)(((float)(500)/60.0f)*(float)(2.0f*3.141592))*(float)2.0f)*(float)0.00005)
#define SpeedRef (float)((float)((float)(((float)(1000)/60.0f)*(float)(2.0f*3.141592))*(float)2.0f))
#define sqrt_2 ((float)1.41421)
#define sqrt_3 ((float)1.73205)
#define one_on_sqrt_2 ((float)0.7071)
#define one_on_sqrt_3 ((float)0.57735)
#define PI ((float)3.141592)
#define Iqref_Openloop ((float)0.8)
#define VDC 24
#define Polepairs 2
#define PWM_FREQ (float)(20000.0)
#define Ts (float)(1/20000)
#define Ls ((float)0.00234)
#define Rs ((float)3.234)
#define CHEB_SIN_1 (float)0.999978675
#define CHEB_SIN_3 (float)-0.1664971
#define CHEB_SIN_5 (float)0.00799224
#define currentObserver_gain (float)(8000.0)
#define boundary_I (float)0.5
#define bemfObserver_gain (float)(-1000.0)


float OPENLOOP_RAMPSPEED_INCREASERATE=END_SPEED_RADS_PER_SEC_ELEC_IN_LOOPTIME/((float)(5)*(float)(0.00005));


float aIObs=(float)(exp(F));
float b1IObs=(float)((exp(F)-1.0f)/Rs);
float b2IObs=(float)(-((exp(F)-1.0f)/Rs));
float b3IObs=(float)(Ls*(-((exp(F)-1.0f)/Rs)));
float F=((float)(-Rs/(Ls*Ts)));
float EalphaHat=0, EbetaHat=0, IalphaHat=0, IbetaHat=0, Balpha=0, Bbeta=0;
float a1BEMFob=0, a2BEMFobs=0, b1BEMFobs=0, b2BEMFobs=0;


uint16_t da=0,db=0,dc=0;
float d0=0,d1=0,d2=0;


float tmp,tmp1;
uint16_t count=0,count1=0,flag=1,flag1=0,flag2=0;


float SineTheta=0, CosineTheta=0,Theta=0;
float Startup_Ramp_Angle_Rads_Per_Sec=0;
float positionTH=0,positiondTH=0,positionTHI=0,positionTHO=0,positionzTH=0;
float closeLoopTHI=0,closeLoopTHO=0,deviationTH=0,decayRateTH=0;
float SUMdTH=0,SpeedMea1=0,SpeedMea=0,FIFOdTH[21];


float current_u=0,current_v=0,current_alpha=0,current_beta=0;
float current_sd=0,current_sq=0;
float SpeedError=0,sum_IsqRef=0,IsqRef1=0,IsqRef=0;
float IsdError=0,sum_Usd=0,Usd1=0,Usd=0,Kp_Usd,Ki_Usd;
float IsqError=0,sum_Usq=0,Usq1=0,Usq=0,Kp_Usq,Ki_Usq;
float Usalpha=0,Usbeta=0;
float Usa=0,Usb=0,Usc=0;


float SineTable[256]=
{
0.0f,
0.024541f,
0.049068f,
0.073565f,
0.098017f,
0.122411f,
0.14673f,
0.170962f,
0.19509f,
0.219101f,
0.24298f,
0.266713f,
0.290285f,
0.313682f,
0.33689f,
0.359895f,
0.382683f,
0.405241f,
0.427555f,
0.449611f,
0.471397f,
0.492898f,
0.514103f,
0.534998f,
0.55557f,
0.575808f,
0.595699f,
0.615232f,
0.634393f,
0.653173f,
0.671559f,
0.689541f,
0.707107f,
0.724247f,
0.740951f,
0.757209f,
0.77301f,
0.788346f,
0.803208f,
0.817585f,
0.83147f,
0.844854f,
0.857729f,
0.870087f,
0.881921f,
0.893224f,
0.903989f,
0.91421f,
0.92388f,
0.932993f,
0.941544f,
0.949528f,
0.95694f,
0.963776f,
0.970031f,
0.975702f,
0.980785f,
0.985278f,
0.989177f,
0.99248f,
0.995185f,
0.99729f,
0.998795f,
0.999699f,
1.0f,
0.999699f,
0.998795f,
0.99729f,
0.995185f,
0.99248f,
0.989177f,
0.985278f,
0.980785f,
0.975702f,
0.970031f,
0.963776f,
0.95694f,
0.949528f,
0.941544f,
0.932993f,
0.92388f,
0.91421f,
0.903989f,
0.893224f,
0.881921f,
0.870087f,
0.857729f,
0.844854f,
0.83147f,
0.817585f,
0.803208f,
0.788346f,
0.77301f,
0.757209f,
0.740951f,
0.724247f,
0.707107f,
0.689541f,
0.671559f,
0.653173f,
0.634393f,
0.615232f,
0.595699f,
0.575808f,
0.55557f,
0.534998f,
0.514103f,
0.492898f,
0.471397f,
0.449611f,
0.427555f,
0.405241f,
0.382683f,
0.359895f,
0.33689f,
0.313682f,
0.290285f,
0.266713f,
0.24298f,
0.219101f,
0.19509f,
0.170962f,
0.14673f,
0.122411f,
0.098017f,
0.073565f,
0.049068f,
0.024541f,
0.0f,
-0.024541f,
-0.049068f,
-0.073565f,
-0.098017f,
-0.122411f,
-0.14673f,
-0.170962f,
-0.19509f,
-0.219101f,
-0.24298f,
-0.266713f,
-0.290285f,
-0.313682f,
-0.33689f,
-0.359895f,
-0.382683f,
-0.405241f,
-0.427555f,
-0.449611f,
-0.471397f,
-0.492898f,
-0.514103f,
-0.534998f,
-0.55557f,
-0.575808f,
-0.595699f,
-0.615232f,
-0.634393f,
-0.653173f,
-0.671559f,
-0.689541f,
-0.707107f,
-0.724247f,
-0.740951f,
-0.757209f,
-0.77301f,
-0.788346f,
-0.803208f,
-0.817585f,
-0.83147f,
-0.844854f,
-0.857729f,
-0.870087f,
-0.881921f,
-0.893224f,
-0.903989f,
-0.91421f,
-0.92388f,
-0.932993f,
-0.941544f,
-0.949528f,
-0.95694f,
-0.963776f,
-0.970031f,
-0.975702f,
-0.980785f,
-0.985278f,
-0.989177f,
-0.99248f,
-0.995185f,
-0.99729f,
-0.998795f,
-0.999699f,
-1.0f,
-0.999699f,
-0.998795f,
-0.99729f,
-0.995185f,
-0.99248f,
-0.989177f,
-0.985278f,
-0.980785f,
-0.975702f,
-0.970031f,
-0.963776f,
-0.95694f,
-0.949528f,
-0.941544f,
-0.932993f,
-0.92388f,
-0.91421f,
-0.903989f,
-0.893224f,
-0.881921f,
-0.870087f,
-0.857729f,
-0.844854f,
-0.83147f,
-0.817585f,
-0.803208f,
-0.788346f,
-0.77301f,
-0.757209f,
-0.740951f,
-0.724247f,
-0.707107f,
-0.689541f,
-0.671559f,
-0.653173f,
-0.634393f,
-0.615232f,
-0.595699f,
-0.575808f,
-0.55557f,
-0.534998f,
-0.514103f,
-0.492898f,
-0.471397f,
-0.449611f,
-0.427555f,
-0.405241f,
-0.382683f,
-0.359895f,
-0.33689f,
-0.313682f,
-0.290285f,
-0.266713f,
-0.24298f,
-0.219101f,
-0.19509f,
-0.170962f,
-0.14673f,
-0.122411f,
-0.098017f,
-0.073565f,
-0.049068f,
-0.024541f
};
float CosineTable[256]=
{
1.0f,
0.999699f,
0.998795f,
0.99729f,
0.995185f,
0.99248f,
0.989177f,
0.985278f,
0.980785f,
0.975702f,
0.970031f,
0.963776f,
0.95694f,
0.949528f,
0.941544f,
0.932993f,
0.92388f,
0.91421f,
0.903989f,
0.893224f,
0.881921f,
0.870087f,
0.857729f,
0.844854f,
0.83147f,
0.817585f,
0.803208f,
0.788346f,
0.77301f,
0.757209f,
0.740951f,
0.724247f,
0.707107f,
0.689541f,
0.671559f,
0.653173f,
0.634393f,
0.615232f,
0.595699f,
0.575808f,
0.55557f,
0.534998f,
0.514103f,
0.492898f,
0.471397f,
0.449611f,
0.427555f,
0.405241f,
0.382683f,
0.359895f,
0.33689f,
0.313682f,
0.290285f,
0.266713f,
0.24298f,
0.219101f,
0.19509f,
0.170962f,
0.14673f,
0.122411f,
0.098017f,
0.073565f,
0.049068f,
0.024541f,
0.0f,
-0.024541f,
-0.049068f,
-0.073565f,
-0.098017f,
-0.122411f,
-0.14673f,
-0.170962f,
-0.19509f,
-0.219101f,
-0.24298f,
-0.266713f,
-0.290285f,
-0.313682f,
-0.33689f,
-0.359895f,
-0.382683f,
-0.405241f,
-0.427555f,
-0.449611f,
-0.471397f,
-0.492898f,
-0.514103f,
-0.534998f,
-0.55557f,
-0.575808f,
-0.595699f,
-0.615232f,
-0.634393f,
-0.653173f,
-0.671559f,
-0.689541f,
-0.707107f,
-0.724247f,
-0.740951f,
-0.757209f,
-0.77301f,
-0.788346f,
-0.803208f,
-0.817585f,
-0.83147f,
-0.844854f,
-0.857729f,
-0.870087f,
-0.881921f,
-0.893224f,
-0.903989f,
-0.91421f,
-0.92388f,
-0.932993f,
-0.941544f,
-0.949528f,
-0.95694f,
-0.963776f,
-0.970031f,
-0.975702f,
-0.980785f,
-0.985278f,
-0.989177f,
-0.99248f,
-0.995185f,
-0.99729f,
-0.998795f,
-0.999699f,
-1.0f,
-0.999699f,
-0.998795f,
-0.99729f,
-0.995185f,
-0.99248f,
-0.989177f,
-0.985278f,
-0.980785f,
-0.975702f,
-0.970031f,
-0.963776f,
-0.95694f,
-0.949528f,
-0.941544f,
-0.932993f,
-0.92388f,
-0.91421f,
-0.903989f,
-0.893224f,
-0.881921f,
-0.870087f,
-0.857729f,
-0.844854f,
-0.83147f,
-0.817585f,
-0.803208f,
-0.788346f,
-0.77301f,
-0.757209f,
-0.740951f,
-0.724247f,
-0.707107f,
-0.689541f,
-0.671559f,
-0.653173f,
-0.634393f,
-0.615232f,
-0.595699f,
-0.575808f,
-0.55557f,
-0.534998f,
-0.514103f,
-0.492898f,
-0.471397f,
-0.449611f,
-0.427555f,
-0.405241f,
-0.382683f,
-0.359895f,
-0.33689f,
-0.313682f,
-0.290285f,
-0.266713f,
-0.24298f,
-0.219101f,
-0.19509f,
-0.170962f,
-0.14673f,
-0.122411f,
-0.098017f,
-0.073565f,
-0.049068f,
-0.024541f,
0.0f,
0.024541f,
0.049068f,
0.073565f,
0.098017f,
0.122411f,
0.14673f,
0.170962f,
0.19509f,
0.219101f,
0.24298f,
0.266713f,
0.290285f,
0.313682f,
0.33689f,
0.359895f,
0.382683f,
0.405241f,
0.427555f,
0.449611f,
0.471397f,
0.492898f,
0.514103f,
0.534998f,
0.55557f,
0.575808f,
0.595699f,
0.615232f,
0.634393f,
0.653173f,
0.671559f,
0.689541f,
0.707107f,
0.724247f,
0.740951f,
0.757209f,
0.77301f,
0.788346f,
0.803208f,
0.817585f,
0.83147f,
0.844854f,
0.857729f,
0.870087f,
0.881921f,
0.893224f,
0.903989f,
0.91421f,
0.92388f,
0.932993f,
0.941544f,
0.949528f,
0.95694f,
0.963776f,
0.970031f,
0.975702f,
0.980785f,
0.985278f,
0.989177f,
0.99248f,
0.995185f,
0.99729f,
0.998795f,
0.999699f
};


int main(void)
{
  DAVE_STATUS_t status;

  status = DAVE_Init();           /* Initialization of DAVE APPs  */

  if (status != DAVE_STATUS_SUCCESS)
  {
    /* Placeholder for error handler code. The while loop below can be replaced with an user error handler. */
    XMC_DEBUG("DAVE APPs initialization failed\n");

    while(1U)
    {

    }
  }
  /* Placeholder for user application code. The while loop below can be replaced with user application code. */
  while(1U)
  {

  }
}

void Adc_Measurement_Handler(){
	/*volt_phaseU=ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_Voltage_PhaseU);
	volt_phaseV=ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_Voltage_PhaseV);
	volt_phaseW=ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_Voltage_PhaseW);
    volt_u=(volt_phaseU*5*12.2)/(2.2*4096);
    volt_v=(volt_phaseV*5*12.2)/(2.2*4096);
    volt_w=(volt_phaseW*5*12.2)/(2.2*4096);*/
	current_phaseU=ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_Current_PhaseU);
	current_phaseV=ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_Current_PhaseV);
	current_u=(current_phaseU*5)/(4096*current_OPAMP*shunt_resistor);
	current_v=(current_phaseV*5)/(4096*current_OPAMP*shunt_resistor);
}
/*Timer_0->INTERRUPT_0->1/20000(50us)*/
void NgatHandler(){
    count++;
    /*Alignment*/
    if (count==0 && flag=1){
    	da=10000;
    	db=0;
    	dc=0;
    }
    if (count==40000){
        flag=0;
        flag1=1;
    }
    /*OpenLoop*/
    if (flag1==1){
    	count1++;
    	theta+=Startup_Ramp_Angle_Rads_Per_Sec;
    	if (theta>=(2*PI)){
    		theta=theta-2*PI;
    	}
        if (Startup_Ramp_Angle_Rads_Per_Sec < END_SPEED_RADS_PER_SEC_ELEC_IN_LOOPTIME)
        {
        	Startup_Ramp_Angle_Rads_Per_Sec += OPENLOOP_RAMPSPEED_INCREASERATE;
        }
        IsqRef=IsqRef_OPENLOOP;
        SinCosCheck();
        AlphaBetaTrans_Current();
        DQTrans_Current;
        UsdCal();
        UsqCal();
        UsdqtoUsalphabeta();
        UsalphabetatoUsabc();
        SVMAlgorithm();
        UpdatePWMDutyCycle();
    }
    if (count1==100000){
    	flag2=1;
    }
    if (flag2=1){
    	SpeedMea1=END_SPEED_RADS_PER_SEC_ELEC_IN_LOOPTIME;
    	AlphabetaBEMF();
    	ThetaCalculation();
    }
    if (count1==130000){
    	flag2=0;
    	flag1=2;
    }
    /*closed loop*/
    if (flag1==2){
    	IsqRef=0;
    	AlphaBetaTrans_Current();
    	AlphabetaBEMF();
    	ThetaCalculation();
    	SpeedCalculation();
    	SpeedMea1=SpeedMea;
    	tmp = Theta - positionTH;
    	if((float)-M_PI > tmp){
    	deviationTH = tmp + RL_2PI;
    	}
    	else if((float)M_PI < tmp){
    	deviationTH = tmp - 2*PI;
    	}
    	else{
        deviationTH = tmp;
    	}
        decayRateTH = deviationTH * (1.0f/2*PWM_FREQ);
        if(((0.0<decayRateTH) && (deviationTH>decayRateTH)) ||
        ((0.0>decayRateTH) && (deviationTH<decayRateTH)))
        {
            deviationTH = deviationTH - decayRateTH;
            tmp1 = deviationTH + positionTHI;
            closeLoopTHI =WrapFrom0To2Pi(tmp1);
            tmp1 = deviationTH + positionTHO;
            closeLoopTHO =WrapFrom0To2Pi(tmp1);
        }
        else
        {
        closeLoopTHI =  positionTHI;
        closeLoopTHO =  positionTHO;
        }
        Theta=closeLoopTHI;
        SinCosCheck();
    	DQTrans_Current;
    	IsqRefCal();
    	UsdCal();
    	UsqCal();
    	Theta=closeLoopTHO;
    	SinCosCheck();
        UsdqtoUsalphabeta();
        UsalphabetatoUsabc();
        SVMAlgorithm();
        UpdatePWMDutyCycle();
    }
}
float floatAbs(float x)
{
    float tmp;

    if(0.0 < x)
    {
        tmp = x;
    }
    else
    {
        tmp = -x;
    }

    return tmp;
}
float sinChebyshevF(float x)
{
    float y;
    float tmp1;
    float tmp2;

    if((float)M_PI_2 < x)
    {
        y = (float)M_PI - x;
    }
    else if((float)(-M_PI_2) > x)
    {
        y = (float)(-M_PI) - x;
    }
    else
    {
        y = x;
    }

    tmp1 = y * y;
    tmp2 = y * (CHEB_SIN_1 + tmp1 * (CHEB_SIN_3 + (tmp1 * CHEB_SIN_5)));

    return tmp2;
}
float cosChebyshevF(float x)
{
    float y;
    float tmp1;

    if(0.0 > x)
    {
        y = (float)M_PI_2 + x;
    }
    else
    {
        y = (float)M_PI_2 - x;
    }

    tmp1 = sinChebyshevF(y);

    return tmp1;
}
float polynmApproxAtanf(float tanVal)
{
    float z, zz;

    if(1.0f < tanVal)
    {
        z = 1.0f;
    }
    else if(-1.0f > tanVal)
    {
        z = -1.0f;
    }
    else
    {
        z = tanVal;
    }

    zz = z * z;

    return ((((float)0.0776509570923569)*zz + ((float)-0.287434475393028))*zz + (float)((PI/4) - ((float)0.0776509570923569) - ((float)-0.287434475393028)))*z;
}



float polynmApproxAtan2f(float y, float x)
{
    float tmp1, tmp2, z;

    tmp1 = floatAbs(y);
    tmp2 = floatAbs(x);
    z = (tmp1 > tmp2) ? (tmp1) : (tmp2);

    if(z < (float)0.0000001)
    {
        return 0.0f;
    }

    if(tmp2 >= tmp1)
    {  /* abs(x) >= abs(y) */
        z = y / x;
        tmp2 = polynmApproxAtanf(z);
        if(x > 0.0f)
        {  /* -pi/2 ~ pi/2 */
            tmp1 = tmp2;
        }
        else if(y > 0.0f)
        {
            tmp1 = tmp2 + PI;
        }
        else
        {
            tmp1 = tmp2 - PI;
        }
    }
    else
    {  /* abs(x) < abs(y) */
        z = x / y;
        tmp2 = polynmApproxAtanf(z);
        if(y > 0.0f)
        {
            tmp1 = -tmp2 + (float)(PI/2);
        }
        else
        {
            tmp1 = -tmp2 - (float)(PI/2);
        }
    }

    return tmp1;
}
void SinCosCheck(void){
        int16_t y0_Index;
        int16_t y0_IndexNext;
        float x0, y0, y1, temp;
	    y0_Index = (int16_t)((float)(Theta * (float)((float)256 / (2.0f*PI))));
	    y0_IndexNext = y0_Index + 1;

	    if(255 <= y0_Index)
	    {
	        y0_Index = 255;
	        y0_IndexNext = 0;
	    }
	    else
	    {
	        y0_Index = 0;
	        y0_IndexNext = 1;
	    }
	    x0 = ((float)y0_Index * (float)((2.0f*PI) / (float)256));
		temp = ((Theta - x0) * (float)((float)256 / (2.0f*PI)));

		// Find Sine
	    y0 = SineTable[y0_Index];
	    y1 = SineTable[y0_IndexNext];
	    SineTheta = y0 + ((y1 - y0)*temp);

	    // Find Cosine
	    y0 = CosineTable[y0_Index];
	    y1 = CosineTable[y0_IndexNext];
	    CosineTheta = y0 + ((y1 - y0)*temp);
}
float WrapFrom0To2Pi(float raw){
    float tmp;

    if(0.0 > raw)
    {
        tmp = raw + (float)(2.0f*PI);
    }
    else if((float)(2.0f*PI) < raw)
    {
        tmp = raw - (float)(2.0f*PI);
    }
    else
    {
        tmp = raw;
    }

    return tmp;
}
void UpdatePWMDutyCycle(void){
	PWM_CCU8_Start(&PWM_CCU8_0);
	PWM_CCU8_Start(&PWM_CCU8_1);
	PWM_CCU8_Start(&PWM_CCU8_2);
	PWM_CCU8_SetDutyCycleSymmetric(&PWM_CCU8_0, XMC_CCU8_SLICE_COMPARE_CHANNEL_1, da);
	PWM_CCU8_SetDutyCycleSymmetric(&PWM_CCU8_1, XMC_CCU8_SLICE_COMPARE_CHANNEL_1, db);
	PWM_CCU8_SetDutyCycleSymmetric(&PWM_CCU8_2, XMC_CCU8_SLICE_COMPARE_CHANNEL_1, dc);
}
void currentObserver(float Current, float U, float Ehat, float * Ihat, float * B)
{
    float tmp1;

    tmp1 = Current - *Ihat;

    if(boundary_I < tmp1)
    {
        *B = currentObserver_gain;
    }
    else if(boundary_I > tmp1)
    {
        *B = -currentObserver_gain;
    }
    else
    {
        *B = tmp1 *(float)(currentObserver_gain/boundary_I) ;
    }

    *Ihat = (aIObs * (*Ihat)) + (b1IObs * Ehat) + (b2IObs * U) + (b3IObs * (*B));
}
void BEMFobsCoeffCal(float We)
{
    float tmp1;
    float tmp2;

    if((0.000001 > We) && (0.000001 < We))
    {
        return;
    }

    tmp1 = We * Ts;
    a1BEMFobs = cosChebyshevF(tmp1);
    a2BEMFobs = sinChebyshevF(tmp1);
    tmp1 = 1.0f - a1BEMFobs;
    tmp2 = bemfObserver_gain / We;
    b1BEMFobs = ((tmp2 * a2BEMFobs) + tmp1) * Ls;
    b2BEMFobs = (a2BEMFobs - (tmp2 * tmp1)) * Ls;
}
void BEMFobserver(float * E1hat, float E2hat, float B1, float B2)
{
     *E1hat =  (a1BEMFobs * (*E1hat)) + (a2BEMFobs * E2hat) + (b1BEMFobs * B1) + (b2BEMFobs * B2);
}
void AlphabetaBEMF(void){
	float zEalphaHat;
	currentObserver(current_alpha, Usalpha, EalphaHat, &IalphaHat, &Balpha);
	currentObserver(current_beta, Usbeta, EbetaHat, &IbetaHat, &Bbeta);
	zEalphaHat = EalphaHat;
	BEMFobsCoeffCal(SpeedMea1);
	BEMFobserver(&EalphaHat, -EbetaHat, Balpha, Bbeta);
	BEMFobserver(&EbetaHat, zEalphaHat, Bbeta, -Balpha);
}
void ThetaCalculation(void){
	float tmp, tmp1, tmp2;
    tmp=polynmApproxAtan2f(EbetaHat,EalphaHat);
    if(0.0f > tmp)
    {
        positionTH = tmp + 2*PI;
    }
    else
    {
        positionTH = tmp;
    }

    tmp1 = (positionTH) - (positionzTH);
    if(-M_PI > tmp1)
    {
        positiondTH = tmp1 + RL_2PI;
    }
    else if(M_PI < tmp1)
    {
        positiondTH = tmp1 - RL_2PI;
    } else
    {
        positiondTH = tmp1;
    }

    positionTHI = positionTH;

    tmp2 = positionTHI + positiondTH;
    if(0.0f > tmp2)
    {
        positionTHO = tmp2 + 2*PI;
    }
    else if(RL_2PI < tmp2)
    {
        positionTHO = tmp2 - 2*PI;
    }
    else
    {
        positionTHO = tmp2;
    }

    positionzTH = positionTH;

}
void SpeedCalculation(void){
    float tmp1;
    uint16_t tmp2, count;

    tmp1 = positiondTH;
    tmp2 = count;
    FIFOdTH[tmp2] = tmp1;

    if(22 > (float)tmp2)
    {
        count++;
    }
    else
    {
        count = 0;
    }

    tmp2 = count;
    SUMdTH +=  tmp1 - FIFOdTH[tmp2];
    SpeedMea = SUMdTH * 1000;
}
void AlphaBetaTrans_Current(void){
	current_alpha=current_u;
	current_beta=one_on_sqrt_3*(current_u+2*current_v);
}
void DQTrans_Current(void){
	current_sd=current_alpha*CosineTheta+current_beta*SineTheta;
    current_sq=(-1)*current_alpha*SineTheta+current_beta*CosineTheta;
}
void IsqRefCal(void){
	SpeedError=SpeedRef-SpeedMea;
	sum_IsqRef=sum_IsqRef+Error*Ts;
	IsqRef1=Kp_IsqRef*SpeedError+Ki_IsqRef*sum_IsqRef;
    IsqRef=IsqRef+IsqRef1;
}
void UsdCal(void){
    IsdError=0-current_sd;
    sum_Usd=sum_Usd+IsdError*Ts;
    Usd1=Kp_Usd*IsdError+Ki_Usd*sum_Usd;
    Usd=Usd+Usd1;
}
void UsqCal(void){
    IsqError=IsqRef-current_sq;
    sum_Usq=sum_Usq+IsqError*Ts;
    Usq1=Kp_Usq*IsqError+Ki_Usq*sum_Usq;
    Usq=Usq+Usq1;
}
void UsdqtoUsalphabeta(void){
	Usalpha=Usd*CosineTheta-Usq*SineTheta;
	Usbeta=Usq*CosineTheta+Usd*SineTheta;
}
void UsalphabetatoUsabc(void){
	Usa=Usalpha;
	Usb=(-Usalpha/2)+(sqrt_3/2)*Usbeta;
    Usc=(-Usalpha/2)-(sqrt_3/2)*Usbeta;
}
void SVMAlgorithm(void){
	int16_t sector_a, sector_b, sector_c, sector, k;
	if (Usa>=Usb){
		sector_a=4;
	}
	else{
		sector_a=0;
	}
	if (Usb>=Usc){
		sector_b=2;
	}
	else{
		sector_b=0;
	}
	if (Usc>=Usa){
		sector_c=1;
	}
	else{
		sector_c=0;
	}
	k=sector_a+sector_b+sector_c;
    switch (k){
    case 0:
    	sector=4;
    	break;
    case 1:
    	sector=4;
    	break;
    case 2:
    	sector=2;
    	break;
    case 3:
    	sector=3;
    	break;
    case 4:
    	sector=6;
    	break;
    case 5:
    	sector=5;
    	break;
    case 6:
    	sector=1;
    	break;
    case 7:
    	sector=1;
    	break;
    }
    switch (sector){
    case 1:
    	d1=(float)((3*Usalpha-sqrt_3*Usbeta)/(2*VDC));
    	d2=(float)((sqrt_3*Usbeta)/VDC);
    	d0=1-d1-d2;
    	da=(uint16_t)((d0/2)*10000);
    	db=(uint16_t)(((d0/2)+d1)*10000);
    	dc=(uint16_t)(((d0/2)+d1+d2)*10000);
    	break;
    case 2:
    	d1=(float)(((-3)*Usalpha+sqrt_3*Usbeta)/(2*VDC));
    	d2=(float)((3*Usalpha+sqrt_3*Usbeta)/(2*VDC));
    	d0=1-d1-d2;
    	da=(uint16_t)(((d0/2)+d1)*10000);
    	db=(uint16_t)((d0/2)*10000);
    	dc=(uint16_t)(((d0/2)+d1+d2)*10000);
    	break;
    case 3:
    	d1=(float)((sqrt_3*Usbeta)/VDC);
    	d2=(float)(((-3)*Usalpha-sqrt_3*Usbeta)/(2*VDC));
    	d0=1-d1-d2;
    	da=(uint16_t)(((d0/2)+d1+d2)*10000);
    	db=(uint16_t)((d0/2)*10000);
    	dc=(uint16_t)(((d0/2)+d1)*10000);
    	break;
    case 4:
    	d1=(float)(((-sqrt_3)*Usbeta)/VDC);
    	d2=(float)(((-3)*Usalpha+sqrt_3*Usbeta)/(2*VDC));
    	d0=1-d1-d2;
    	da=(uint16_t)(((d0/2)+d1+d2)*10000);
    	db=(uint16_t)(((d0/2)+d1)*10000);
    	dc=(uint16_t)((d0/2)*10000);
    	break;
    case 5:
    	d1=(float)(((-3)*Usalpha-sqrt_3*Usbeta)/(2*VDC));
    	d2=(float)((3*Usalpha-sqrt_3*Usbeta)/(2*VDC));
    	d0=1-d1-d2;
    	da=(uint16_t)(((d0/2)+d1)*10000);
    	db=(uint16_t)(((d0/2)+d1+d2)*10000);
    	dc=(uint16_t)((d0/2)*10000);
    	break;
    case 6:
        d1=(float)((3*Usalpha+sqrt_3*Usbeta)/(2*VDC));
        d2=(float)((-3)*Usbeta/VDC);\
        d0=1-d1-d2;
        da=(uint16_t)((d0/2)*10000);
        db=(uint16_t)(((d0/2)+d1+d2)*10000);
        dc=(uint16_t)(((d0/2)+d1)*10000);
        break;
    }
}





















/*void NgatHandler(void){
	  Error=SpeedRef-SpeedMea;
	  sum=sum+Error*0.0001;
      duty1=Kp*Error+Ki*sum;
	  duty2=duty2+duty1;
	  if(duty2<0){
			duty2=0;
			}
	  if(duty2>10000){
			duty2=10000;
	        }
		a=a+0.0001;



		  	result=ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_Current_PhaseU);
}*/

/*void HallEvent_ISR(void)
{
  HALL_POSIF_SpeedCalculation(&HALL_POSIF_0,&SpeedMea);
}*/

/*hallposition = HALL_POSIF_GetHallPosition(&HALL_POSIF_0);
	  if (hallposition==1U){
	  		  sector=4;

	  		  PWM_CCU4_Stop(&PWM_HU);
	  		  		  PWM_CCU4_Stop(&PWM_LU);
	  		  		  PWM_CCU8_Stop(&PWM_HV);
	  		  		  PWM_CCU8_Stop(&PWM_LW);
	  		  			  PWM_CCU8_Start(&PWM_LV);
	  		  			  PWM_CCU8_Start(&PWM_HW);
	  		  			PWM_CCU8_SetDutyCycleSymmetric(&PWM_HW, XMC_CCU8_SLICE_COMPARE_CHANNEL_1, duty);
	  	  }
	  	  if (hallposition==2U){
	  		  sector=2;

	  		  PWM_CCU4_Stop(&PWM_HU);
	  		  PWM_CCU8_Stop(&PWM_LV);
	  		  PWM_CCU8_Stop(&PWM_HW);
	  		  PWM_CCU8_Stop(&PWM_LW);
	  			  PWM_CCU4_Start(&PWM_LU);
	  			  PWM_CCU8_Start(&PWM_HV);
	  			PWM_CCU8_SetDutyCycleSymmetric(&PWM_HV, XMC_CCU8_SLICE_COMPARE_CHANNEL_1, duty);

	  	  }
	  	  if (hallposition==3U){
	  		  sector=3;

	  		  PWM_CCU4_Stop(&PWM_HU);
	  		  		  PWM_CCU8_Stop(&PWM_HV);
	  		  		  PWM_CCU8_Stop(&PWM_LV);
	  		  		  PWM_CCU8_Stop(&PWM_LW);
	  		  			  PWM_CCU4_Start(&PWM_LU);
	  		  			  PWM_CCU8_Start(&PWM_HW);
	  		  			PWM_CCU8_SetDutyCycleSymmetric(&PWM_HW, XMC_CCU8_SLICE_COMPARE_CHANNEL_1, duty);
	  	  }
	  	  if (hallposition==4U){
	  		  sector=6;

	  		  PWM_CCU4_Stop(&PWM_LU);
	  		  		  PWM_CCU8_Stop(&PWM_HV);
	  		  		  PWM_CCU8_Stop(&PWM_LV);
	  		  		  PWM_CCU8_Stop(&PWM_HW);
	  		  			  PWM_CCU4_Start(&PWM_HU);
	  		  			  PWM_CCU8_Start(&PWM_LW);
	  			  		PWM_CCU4_SetDutyCycle(&PWM_HU, duty);
	  	  }
	  	  if (hallposition==5U){
	  		  sector=5;

	  		  PWM_CCU4_Stop(&PWM_LU);
	  		  PWM_CCU8_Stop(&PWM_HV);
	  		  PWM_CCU8_Stop(&PWM_HW);
	  		  PWM_CCU8_Stop(&PWM_LW);
	  			  PWM_CCU4_Start(&PWM_HU);
	  			  PWM_CCU8_Start(&PWM_LV);
			  		PWM_CCU4_SetDutyCycle(&PWM_HU, duty);
	  	  }
	  	  if (hallposition==6U){
	  		  sector=1;

	  		  PWM_CCU4_Stop(&PWM_HU);
	  		  		  PWM_CCU4_Stop(&PWM_LU);
	  		  		  PWM_CCU8_Stop(&PWM_LV);
	  		  		  PWM_CCU8_Stop(&PWM_HW);
	  		  			  PWM_CCU8_Start(&PWM_HV);
	  		  			  PWM_CCU8_Start(&PWM_LW);
	  			  		PWM_CCU8_SetDutyCycleSymmetric(&PWM_HV, XMC_CCU8_SLICE_COMPARE_CHANNEL_1, duty);
	  	  }*/
