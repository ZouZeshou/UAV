#include  "pid.h"

/*************增量式PID****************/

void PID_IncrementMode(PID_IncrementType *pid)//相比位置环计算量减少
{
	 if(pid->kp<0) pid->kp=-pid->kp;
	 if(pid->ki<0) pid->ki=-pid->ki;
	 if(pid->kd<0) pid->kd=-pid->kd;
	
	 if(pid->errNow >5 && pid->errNow<-5)
   {
	    pid->errNow=0;
	 }
	 
	 pid->dErrP=pid->errNow-pid->errOld1;
	 pid->dErrI=pid->errNow;
	 pid->dErrD=pid->errNow-2*pid->errOld1+pid->errOld2;
	
	 pid->errOld2=pid->errOld1;
	 pid->errOld1=pid->errNow;
	
	 pid->dCtrOut=pid->kp*pid->dErrP+pid->ki*pid->dErrI+pid->kd*pid->dErrD;
	
	 if(pid->dCtrOut>pid->dOutMAX)
   {
     pid->dCtrOut=pid->dOutMAX;
   }
   else if(pid->dCtrOut<-pid->dOutMAX)
   {
	   pid->dCtrOut=-pid->dOutMAX;
	 }
	
	 if(pid->kp==0 && pid->ki==0 && pid->kd==0)
	    pid->ctrOut=0;
	 else
		 pid->ctrOut+=pid->dCtrOut;//结果做个累加就是一个位置环 
	 
	 if(pid->ctrOut>pid->OutMAX)
   {
     pid->ctrOut=pid->OutMAX;
   }
   else if(pid->ctrOut<-pid->OutMAX)
   {
	   pid->ctrOut=-pid->OutMAX;
	 }
}

/**************绝对式PID*****************/
void PID_AbsoluteMode(PID_AbsoluteType *pid)
{
	
	 pid->ErrP = pid->errNow;
	 pid->ErrI += pid->errNow;
	 pid->ErrD = pid->errNow - pid->errOld;
	 pid->errOld = pid->errNow;
	
	 if(pid->ErrI > pid->errILim)
		 pid->ErrI =  pid->errILim;
	 else if(pid->ErrI < -pid->errILim)
		 pid->ErrI =  -pid->errILim;
	 
	 pid->ctrOut = pid->kp*pid->ErrP + pid->ki*pid->ErrI + pid->kd*pid->ErrD;
	 
	 if(pid->ctrOut > pid->OutMAX)
		 pid->ctrOut=pid->OutMAX;
	 else if(pid->ctrOut < -pid->OutMAX)
		 pid->ctrOut = -pid->OutMAX;
	 
}









