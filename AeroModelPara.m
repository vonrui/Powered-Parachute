function [CD, CL,Thrust]	=	AeroModel(x,u,V)
%
%
%

% disp('***************** ENTER IN AEROMODEL *******************')
	global V cBar S CBody CRoll CPitch
	global AlphaTable CDTable0 CDTable45 CDTable90 CLTable0 CLTable45 CLTable90 

% 限制攻角幅度 
	alphadeg	=	57.2957795 * alphar;
	if alphadeg <=-8
	 alphadeg= -8;
  	end
  	if alphadeg >= 30
	   alphadeg = 30;
  	end
        
%	Thrust Properties(海平面静推力, 默认值取50kg)
	StaticThrust	=	5*10^2;	% Static Thrust @ Sea Level, N	
    %	Current Thrust(计算特定海拔高度下的推力)   	% Thrust at Altitude, N
	[airDens,airPres,temp,soundSpeed] = Atmos(-x(6));
	Thrust			=	u(3) * StaticThrust * (airDens / 1.225)^0.7 ...
						* (1 - exp((-x(6) - 17000) / 2000));        

%	Current Longitudinal Characteristics(纵向特征)
%	====================================

    %Lift Coefficient(升力系数)
    CL = zero(1,2);
    u_norm = zero(1,2);
	CLStatic0    =	interp1(AlphaTable,CLTable0,alphadeg);	% Static Lift Coefficient(静态升力系数)
	CLStatic45   =	interp1(AlphaTable,CLTable45,alphadeg);
	CLStatic90   =	interp1(AlphaTable,CLTable90,alphadeg);
	%对u进行归一化处理
	for i = [1:1:2]
		u_norm(i) = u(i) / 0.5;
		if u_norm(i) < 0.5 && u_norm(i) >= 0
			CL(i) = u_norm(i) * (CLStatic0 + CLStatic45) / 2;
		else
			CL(i) = u_norm(i) * (CLStatic45 + CLStatic90) / 2;
		end
	end

	%Drag Coefficient(阻力系数)
	CD = zero(1,2);

	CDStatic0    =	interp1(AlphaTable,CDTable0,alphadeg);	% Static Drag Coefficient(静态阻力系数)
	CDStatic45   =	interp1(AlphaTable,CDTable45,alphadeg);
	CDStatic90   =	interp1(AlphaTable,CDTable90,alphadeg);
	for i = [1:1:2]
		if u_norm(i) < 0.5 && u_norm(i) >= 0
			CD = u_norm(i) * (CDStatic0 + CDStatic45) / 2;
		else
			CD = u_norm(i) * (CDStatic45 + CDStatic90) / 2;
		end
	end