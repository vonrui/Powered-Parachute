function H = DCM(Phi, Theta, Psi)
%H = DCM(Phi, Theta, Psi)
%define Earth frame to Body axis Direction-Cosine Matrix

%2016/3/5
%================================================

%       Euler Angles:
%		Phi,	   Roll Angle,      (X)       rad    
%		Theta,	 Pitch Angle,    (Y)       rad
%		Psi,	   Yaw Angle,      (Z)       rad


    sinPhi = sin(Phi);
	cosPhi = cos(Phi);
	sinTheta = sin(Theta);
	cosTheta = cos(Theta);
	sinPsi = sin(Psi);
	cosPsi = cos(Psi);

	H(1,1) = cosTheta * cosPsi;
	H(1,2) = cosTheta * sinPsi;
	H(1,3) = -sinTheta;
	H(2,1) = sinPhi * sinTheta * cosPsi - cosPhi * sinPsi;
	H(2,2) = sinPhi * sinTheta * sinPsi + cosPhi * cosPsi;
	H(2,3) = sinPhi * cosTheta;
	H(3,1) = cosPhi * sinTheta * cosPsi + sinPhi * sinPsi;
	H(3,2) = cosPhi * sinTheta * sinPsi - sinPhi * cosPsi;
	H(3,3) = cosPhi * cosTheta;
