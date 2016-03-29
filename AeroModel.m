function [CD,CL,CY,Cl,Cm,Cn,Thrust]	=	AeroModel(x,u,Mach,alphar,betar,V)
%   [CD,CL,CY,Cl,Cm,Cn,Thrust]	=	AeroMode(x,u,Mach,alphar,betar,V)
%   FLIGHT Aerodynamic Coefficients of the Aircraft, Thrust Model,
%   and Geometric and Inertial Properties
%   根据姿态参数x, 控制参数u, 马赫数Mach, 攻角alphar, 侧滑角betar, 和速度V
%   计算出 阻力, 升力, 此函数在EOM中调用


   % disp('***************** ENTER IN AEROMODEL *******************')
   global V cBar S CBody CRoll CPitch
    %global m Ixx Iyy Izz Ixz S b cBar SMI CONHIS u tuHis deluHis uInc  RUNNING 
    global AlphaTable CDTable CLTable CYBetaTable ClBetaTable CldATable CmTable CndATable CnBetaTable
    %load ('/home/ldp/app/R2013b_UNIX/ldp workspace/powered parachute/InerGeo.mat')
    %load ('/home/ldp/app/R2013b_UNIX/ldp workspace/powered parachute/DataTable.mat')

    
    alphadeg	=	57.2957795 * alphar;
    if alphadeg <=-8
        alphadeg= -8;
    end
    if alphadeg >=20
        alphadeg = 20;
    end
        
    
    %	Thrust Properties(海平面静推力, 默认值取50kg)
	StaticThrust	=	5*10^2;	% Static Thrust @ Sea Level, N	
    %	Current Thrust(计算特定海拔高度下的推力)   	% Thrust at Altitude, N
	[airDens,airPres,temp,soundSpeed] = Atmos(-x(6));
	Thrust			=	u(3) * StaticThrust * (airDens / 1.225)^0.7 ...
						* (1 - exp((-x(6) - 17000) / 2000));
	        

    %	Current Longitudinal Characteristics(纵向特征)
    %	====================================

    %	Lift Coefficient(升力系数)
	CLStatic    =	interp1(AlphaTable,CLTable,alphadeg);
									% Static Lift Coefficient(静态升力系数)
    %计算左右副翼升力系数
    CLA1 = interp1(AlphaTable,CLTable, (alphadeg+u(1)));
    CLA2 = interp1(AlphaTable,CLTable, (alphadeg+u(2)));
    CLA = 0.1*(CLA1+CLA2);
    %计算总升力系数 Total Lift Coefficient	
    CL     =	CLStatic+CLA;
    
    
    
	%	Drag Coefficient(阻力系数)
	CDStatic	=	interp1(AlphaTable,CDTable,alphadeg);
									% Static Drag Coefficient
    %计算左右副翼阻力系数
    CDA1 = interp1(AlphaTable,CDTable, (alphadeg+u(1)));
    CDA2 = interp1(AlphaTable,CDTable, (alphadeg+u(2)));
    CDA = 0.1*(CDA1+CDA2);
	CD          =	CDStatic;		% Total Drag Coefficient
	
    
    %	Side-Force Coefficient(侧向力系数)     % Side-Force Slope, per rad
	CYBr	=	interp1(AlphaTable,CYBetaTable,alphadeg);
    betar    ;                                   
	CYdRr	=	0.1574;              % Rudder Effect, per rad	
	CY	=	(CYBr*betar + CYdRr*(u(2)-u(1)) ) ;    %这里原先在用方向舵计算侧向力系数
                                                                               %现在使用副翼
                                           % Total Side-Force Coefficient
    
    
    %	Pitching Moment Coefficient(俯仰力矩系数)
	CmStatic	=	interp1(AlphaTable,CmTable,alphadeg);   %攻角越大低头力矩越大
									% Static Pitching Moment Coefficient(静态俯仰力矩系数)

	Cm          =	CmStatic;
									% Total Pitching Moment Coefficient	
                                    
                                    
%	Current Lateral-Directional Characteristics
%	===========================================

%	Rolling Moment Coefficient(滚转力矩系数)
%CldAr	=	interp1(AlphaTable,CldATable,alphadeg);
									% Aileron Effect, per rad	      (副翼效应)
%Cl      =	 CldAr* ( u(2)-u(1))/2;
CldA1r	=	interp1(AlphaTable,CldATable,alphadeg+u(1))
CldA2r	=	interp1(AlphaTable,CldATable,alphadeg+u(2))
									% Aileron Effect, per rad	      (副翼效应)
Cl      =	 2*(  CldA1r*u(1)-CldA2r*u(2)  )     %右侧操纵绳下拉时产生顺时针力矩

    
									% Total Rolling-Moment Coefficient
                                                     
%	Yawing Moment Coefficient(偏航力矩系数)
    %CnBr	=	interp1(AlphaTable, CnBetaTable, alphadeg);    %考虑侧滑角影响 rad
	%CndAr	=	interp1(AlphaTable, CndATable, alphadeg);      %考虑副翼对偏航的影响  rad

    %Cn	= CnBr*betar-CndAr*(u(2)-u(1)) ;                  % Total Yawing-Moment Coefficient
    %侧滑角影响过大,需要修改
    Cn = (CDA2-CDA1)*0.1;
									

end
  
    