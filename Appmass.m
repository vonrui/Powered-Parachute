%
%计算附加质量
%

global b cBar 

%计算大气密度
[airDens,airPres,temp,soundSpeed] = Atmos(-x(6));

%准备所需数据
ka = 0.85;
kb = 1.0;
AR = b/cBar;

a11 = airDens * ka * pi * (e*e*b/4);
a22 = airDens * kb * pi * (e*e*cBar/4);
a33 = airDens * (AR/(1+AR)) * pi *(cBar*cBar*b/4);
a44 = 0.055 * airDens * (AR/(1+AR)) * b^3 * cBar^2;
a55 = 0.0308 * airDens * (AR/(1+AR)) * cBar^4 * b;
a66 = 0.055 * airDens * b^3 * e^2;





