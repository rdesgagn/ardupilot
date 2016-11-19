function angMeas = calcMagAng(decl,gPhi,gPsi,gTheta,magX,magY,magZ,q0,q1,q2,q3)
%CALCMAGANG
%    ANGMEAS = CALCMAGANG(DECL,GPHI,GPSI,GTHETA,MAGX,MAGY,MAGZ,Q0,Q1,Q2,Q3)

%    This function was generated by the Symbolic Math Toolbox version 5.8.
%    14-Jan-2015 16:51:18

% Define rotation from magnetometer to sensor using a 312 rotation sequence
Tms = calcTms(gPhi,gPsi,gTheta);
% Define rotation from sensor to NED
Tsn = Quat2Tbn([q0;q1;q2;q3]);
% Define rotation from magnetometer to NED axes
Tmn = Tsn*Tms;
% rotate magentic field measured at top plate into nav axes
magMeasNED = Tmn*[magX;magY;magZ];
% the predicted measurement is the angle wrt magnetic north of the horizontal
% component of the measured field
angMeas = atan2(magMeasNED(2),magMeasNED(1)) - decl;