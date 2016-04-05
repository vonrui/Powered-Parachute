%
%计算附加质量
%

global b cBar emax x r Mtheta

%计算大气密度
[airDens,airPres,temp,soundSpeed] = Atmos(-x(6));

%准备所需数据
ka = 0.85;			%三维效应修正因子
kb = 1.0;			%三维效应修正因子
AR = b/cBar;		%展弦比
%平直翼附加质量公式
mf11 = airDens * ka * pi * (emax*emax*b/4);
mf22 = airDens * kb * pi * (emax*emax*cBar/4);
mf33 = airDens * (AR/(1+AR)) * pi *(cBar*cBar*b/4);
mf44 = 0.055 * airDens * (AR/(1+AR)) * b^3 * cBar^2;
mf55 = 0.0308 * airDens * (AR/(1+AR)) * cBar^4 * b;
mf66 = 0.055 * airDens * b^3 * emax^2;

%计算俯仰和滚转中心
Zpc = r * sin(Mtheta) / (Mtheta)			%俯仰中心
Zrc = Zpc * mf22 / (mf22 + mf44/r^2)		%滚转中心
Zpr = Zpc - Zrc;							%两个中心之间的距离

%计算伞翼平动附加质量和转动惯量分量
hstar = r*(1-cos(Mtheta))/(2*r*sin(Mtheta));	%衡量翼型弯度的参数

mp11 = (1 + (3/8)*(hstar*hstar)) * mf11;
mp22 = (r^2 * mf22 + mf44) / Zpc^2;
mp33 = mf33;
Ip11 = (Zpr^2/Zpc^2) * r^2 * mf22 + (Zrc^2/Zpc^2) * mf44;
Ip22 = mf55;
Ip33 = (1 + 8*(hstar^2)) * mf66;

%附加质量和转动惯量在质心的投影
Select = [0 0 0 		%定义选择矩阵
		  0 1 0
		  0 0 0];







