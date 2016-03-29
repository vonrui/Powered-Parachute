function  windbody =WindField(height,phir,thetar,psir)
%    windbody =WindField(height,phir,thetar,psir)
%	FLIGHT Wind Field Interpolation for 3-D wind as a Function of Altitude
%   height:插值计算风场矢量的高度参数
%   phir:     地面坐标系下滚转角
%   thetar:  地面坐标系下俯仰角
%   psir:     地面坐标系下偏航角
%	2016/3/5
%	===============================================================

%定义高度, 三维风场矢量, 一共设置了十个点
    windh	=	[-10 0 100 200 500 1000 2000 4000 8000 16000];
    windx	=	[0 0 0 0 0 0 0 0 0 0];	% Northerly wind, m/s
	windy	=	[0 0 0 0 0 0 0 0 0 0];	% Easterly wind, m/s
	windz	=	[0 0 0 0 0 0 0 0 0 0];	% Vertical wind. m/s
    
 %插值求解地面风场矢量(wind earth)
    winde	=	[ interp1(windh,windx,height)
                        interp1(windh,windy,height)
                        interp1(windh,windz,height)];
                    
 %计算地面->体轴系方向余弦矩阵                    
    HEB		=	DCM(phir,thetar,psir);
    
 %将地面坐标系下的风场转换到体轴系
	windbody	=	HEB * winde;					% Body-axis frame
    