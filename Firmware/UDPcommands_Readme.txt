Granulobot V14 - Wifi commands

format (see regexp/lua)
%d+ any number character of any lenght
%s space
%a any character
%+?%-? can have "+" or "-"
%
/////
IP[%d+].[%d+].[%d+].[%d+]
change IP adress of datastream target (computer gathering data)
      
Po
Poke Robots and turn on LED blue for 1sec

Cal
enter calibration mode => PWM swipe from 0 to 255 by steps of 5
C_time[%d+];
time between 2 steps for calibration mode in ms

HandInit [/i/i+1/.../N/];

HandC[%+?%-?%d+],[%+?%-?%d+],[%+?%-?%d+],[%+?%-?%d+],[%+?%-?%d+];

PIDSP[%d+]I[%d+]D[%d+];

HandT[%d+]A[%d+]SG[%d+];

HandStop;

TS[%d+];

M[%+?%-?%d+];

HandRoll[%d/%+?%-?%d+];

MR[%+?%-?%d+];

DR[%+?%-?%d+];

DRF[%+?%-?%d+];

HandF[%d+]/P[%+?%-?%d+]Y[%d+]SG[%d+];

Stiff
use PID and encoder to fix rotor position

Soft
All motor off in fast decay

Spr[%s%a%d+]b[%d+];

Sl
enter sleep mode

Sp[%d+];

A[%d+];

MA[%+?%-?%d+];

MS[%+?%-?%d+]P[%+?%-?%d+];

PWMF[%d+];
change PWM frequency (typical range [20-5000]hz)
lower frequency  better for low speed, high freq betterfor high speed

PIDSP[%d+]I[%d+]D[%d+];
change PID parameters for speed

SF[%+?%-?%d+]; 
Simple PWM control - Fast current decay

SFS[%+?%-?%d+]; 
Simple PWM control - Slow current decay

Br[%+?%-?%d+];
Braking - AOUT 1 and 2 of DC motor equal
Change the amount of current that goes in the Slow decay loop vs Fast one
255=full slod decay, 0= full fast decay

DiffPWM[%+?%-?%d+],[%+?%-?%d+]; 
direct contrl of PWM value in DRV8833'S AOUT1 and AOUT2

