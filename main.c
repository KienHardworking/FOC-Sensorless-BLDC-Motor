/*
 * main.c
 *
 *  Created on: 2024 Aug 05 09:00:14
 *  Author: dell
 */




#include "DAVE.h"                 //Declarations from DAVE Code Generation (includes SFR declaration)
#include "parameter.h"
/**

 * @brief main() - Application entry point
 *
 * <b>Details of function</b><br>
 * This routine is the application entry point. It is invoked by the device startup code. It is responsible for
 * invoking the APP initialization dispatcher routine - DAVE_Init() and hosting the place-holder for user application
 * code.
 */
XMC_VADC_RESULT_SIZE_t current_phaseU,current_phaseV;
float ADC_scale=max_motor_current/1433;
float cumulative_sum_phaseU=0;
float moving_average_phaseU=0;

float OPENLOOP_RAMPSPEED_INCREASERATE=END_SPEED_RADS_PER_SEC_ELEC_IN_LOOPTIME/((float)(5)*(float)(0.00005));


float aIObs=(float)(exp(F));
float b1IObs=(float)((exp(F)-1.0f)/Rs);
float b2IObs=(float)(-((exp(F)-1.0f)/Rs));
float b3IObs=(float)(Ls*(-((exp(F)-1.0f)/Rs)));
float F=((float)(-Rs/(Ls*Ts)));
float EalphaHat=0;
float EbetaHat=0;
float IalphaHat=0;
float IbetaHat=0;
float Balpha=0;
float Bbeta=0;
float a1BEMFob=0;
float a2BEMFobs=0;
float b1BEMFobs=0;
float b2BEMFobs=0;


uint16_t da=0,db=0,dc=0;
float d0=0,d1=0,d2=0;


float tmp,tmp1;
uint16_t count=0,count1=0,flag=1,flag1=0,flag2=0;


float SineTheta=0;
float CosineTheta=0;
float Theta=0;
float Startup_Ramp_Angle_Rads_Per_Sec=0;
float positionTH=0;
float positiondTH=0;
float positionTHI=0;
float positionTHO=0;
float positionzTH=0;
float closeLoopTHI=0;
float closeLoopTHO=0;
float deviationTH=0;
float decayRateTH=0;
float SUMdTH=0;
float SpeedMea1=0;
float SpeedMea=0;
float FIFOdTH[21];


float current_u=0;
float current_v=0;
float current_alpha=0;
float current_beta=0;
float current_sd=0;
float current_sq=0;

float SpeedError=0;
float Kp_IsqRef;
float Ki_IsqRef;
float sum_IsqRef=0;
float IsqRef1=0;
float IsqRef=0;
float IsdError=0;
float sum_Usd=0;
float Usd1=0;
float Usd=0;
float Kp_Usd;
float Ki_Usd;
float IsqError=0;
float sum_Usq=0;
float Usq1=0;
float Usq=0;
float Kp_Usq;
float Ki_Usq;

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

/*
 Function: Doc ADC dong do ve tren dai 4095 bit qua bo loc moving average filter, tra ve gia tri thuc (A)
 Input:current_phaseU, current_phaseV
 Output:current_u, current_v
 */
void Adc_Measurement_Handler(){
	current_phaseU=ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_Current_PhaseU);
	current_phaseV=ADC_MEASUREMENT_GetResult(&ADC_MEASUREMENT_Current_PhaseV);

	cumulative_sum_phaseU  =  (uint32_t)((int32_t)((int32_t)cumulative_sum_phaseU + current_phaseU - moving_average_phaseU));
	moving_average_phaseU  = (int32_t)((uint32_t)(cumulative_sum_phaseU >> MOVING_AVG_WINDOW_SIZE));
	if (moving_average_phaseU > CURRENT_OFFSET_MAX)
	    {
	        moving_average_phaseU = CURRENT_OFFSET_MAX;
	    }
	if (moving_average_phaseU < CURRENT_OFFSET_MIN)
	    {
	        moving_average_phaseU = CURRENT_OFFSET_MIN;
	    }

	cumulative_sum_phaseV  = (uint32_t)((int32_t)((int32_t)cumulative_sum_phaseV + current_phaseV - moving_average_phaseV));
	moving_average_phaseV  = (int32_t)((uint32_t)(cumulative_sum_phaseV >> MOVING_AVG_WINDOW_SIZE));

	if (moving_average_phaseV > CURRENT_OFFSET_MAX)
	    {
	        moving_average_phaseV = CURRENT_OFFSET_MAX;
	    }
	if (moving_average_phaseV < CURRENT_OFFSET_MIN)
	    {
	        moving_average_phaseV = CURRENT_OFFSET_MIN;
	    }
    current_u = ((current_phaseU - (int16_t)moving_average_phaseU)*ADC_scale);
    current_v = ((current_phaseV - (int16_t)moving_average_phaseV)*ADC_scale);
}
/*
 Function:Cap nhat duty cycle o tung nhanh van
 Input:da, db, dc
 Output:Duty cycle nhanh van pha U,V,W
 */
void UpdatePWMDutyCycle(void){
	PWM_CCU8_Start(&PWM_CCU8_0);
	PWM_CCU8_Start(&PWM_CCU8_1);
	PWM_CCU8_Start(&PWM_CCU8_2);
	PWM_CCU8_SetDutyCycleSymmetric(&PWM_CCU8_0, XMC_CCU8_SLICE_COMPARE_CHANNEL_1, da);
	PWM_CCU8_SetDutyCycleSymmetric(&PWM_CCU8_1, XMC_CCU8_SLICE_COMPARE_CHANNEL_1, db);
	PWM_CCU8_SetDutyCycleSymmetric(&PWM_CCU8_2, XMC_CCU8_SLICE_COMPARE_CHANNEL_1, dc);
}
/*
 Function:Chuyen he toa dong dien do abc sang alpha-beta
 Input:current_u, current_v
 Output:current_alpha, current_beta
 */
void AlphaBetaTrans_Current(void){
	current_alpha=current_u;
	current_beta=one_on_sqrt_3*(current_u+2*current_v);
}
/*
 Function:Chuyen he toa do dong dien alpha-beta sang he toa do dq
 Input:current_alpha, current_beta
 Output:current_sd, current_sq
 */
void DQTrans_Current(void){
	current_sd=current_alpha*CosineTheta+current_beta*SineTheta;
    current_sq=(-1)*current_alpha*SineTheta+current_beta*CosineTheta;
}
/*
 Function:Tinh toan gia tri q axis stator current reference su dung bdk PI
 Input:SpeedRef(toc do dat), SpeedMea(toc do do ve),Ts(chu ky trich mau), Kp_IsqRef, Ki_IsqRef
 Output:IsqRef
 */
void IsqRefCal(void){
	SpeedError=SpeedRef-SpeedMea;
	sum_IsqRef=sum_IsqRef+Error*Ts;
	IsqRef1=Kp_IsqRef*SpeedError+Ki_IsqRef*sum_IsqRef;
    IsqRef=IsqRef+IsqRef1;
}
/*
 Function:Tinh toan gia tri d axis stator voltage su dung bdk PI
 Input:current_sd, Ts, Kp_Usd, Ki_Usd
 Output:Usd
 */
void UsdCal(void){
    IsdError=0-current_sd;
    sum_Usd=sum_Usd+IsdError*Ts;
    Usd1=Kp_Usd*IsdError+Ki_Usd*sum_Usd;
    Usd=Usd+Usd1;
}
/*
 Function:Tinh toan gia tri q axis stator voltage su dung bdk PI
 Input:IsqRef, current_sq, Ts, Kp_Usq, Ki_Usq
 Output:Usq
 */
void UsqCal(void){
    IsqError=IsqRef-current_sq;
    sum_Usq=sum_Usq+IsqError*Ts;
    Usq1=Kp_Usq*IsqError+Ki_Usq*sum_Usq;
    Usq=Usq+Usq1;
}
/*
 Function:Chuyen he toa do dien ap dq sang alpha-beta
 Input:Usd, Usq
 Output:Usalpha, Usbeta
 */
void UsdqtoUsalphabeta(void){
	Usalpha=Usd*CosineTheta-Usq*SineTheta;
	Usbeta=Usq*CosineTheta+Usd*SineTheta;
}
/*
 Function:Chuyen he toa do dien ap alpha-beta sang abc
 Input:Usalpha, Usbeta
 Output:Usa, Usb, Usc
 */
void UsalphabetatoUsabc(void){
	Usa=Usalpha;
	Usb=(-Usalpha/2)+(sqrt_3/2)*Usbeta;
    Usc=(-Usalpha/2)-(sqrt_3/2)*Usbeta;
}
/*
 Function:Thuat toan SVM tinh toan duty cycle cho 3 nhanh van, nhanh van direct tuong ung voi da, db, dc. Nhanh van invert tuong ung voi 100-da, 100-db, 100-dc (%)
 Input:Usa, Usb, Usc, Usalpha, Usbeta, VDC
 Output:da, db, dc
 */
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
/*
 Function:Tinh toan sine, cosine cua goc dong co quay, check xem goc dong co quay >2pi dua ve trong gia tri vong tron luong giac
 Input:Theta
 Output:SineTheta, CosineTheta
 */
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
