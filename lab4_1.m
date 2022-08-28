%% ejercicio 4
% En un modelo en Simulink, diseñe usando bloques de operadores matemáticos, la conversión de los
% Angulos de Euler “Yaw, Pitch y Roll�? a Quaternions.
clear all; close all; clc

AN = matrix_R(45,-5,1);
BN = matrix_R(-88,1,-3);

AB = AN*BN'
[r,p,y] = var_RPY(AB);
disp("Cuerpo A respecto B")
disp("roll: " + r)
disp("pitch: " + p)
disp("yaw: " + y)

BA = BN*AN'
[r2,p2,y2] = var_RPY(BA);
disp("Cuerpo B respecto A")
disp("roll: " + r2)
disp("pitch: " + p2)
disp("yaw: " + y2)


% [thba kba]=angvect_me(BA);
% quaternio = quater(thba,kba);
% 
% aaa=matrix_R(45,45,45)
% [thh kk]=angvect_me(aaa)

function R = matrix_R(th1,th2,th3)
th1=th1*pi/180;
th2=th2*pi/180;
th3=th3*pi/180;
R = [cos(th2)*cos(th1) cos(th2)*sin(th1) -sin(th2);
sin(th3)*sin(th2)*cos(th1)-cos(th3)*sin(th1) sin(th3)*sin(th2)*sin(th1)+cos(th3)*cos(th1) sin(th3)*cos(th2);
cos(th3)*sin(th2)*cos(th1)+sin(th3)*sin(th1) cos(th3)*sin(th2)*sin(th1)-sin(th3)*cos(th1) cos(th3)*cos(th2)];
end

function [r,p,y] = var_RPY(matrix)
% matrix = [C11 C12 C13;
%           C21 C22 C23;
%           C31 C32 C33];
C11 =matrix(1,1);
C12 =matrix(1,2);
C13 =matrix(1,3);
C23 =matrix(2,3);
C33 =matrix(3,3);

y = atan(C12/C11)*180/pi;
p = -asin(C13)*180/pi;
r = atan(C23/C33)*180/pi;
end

function [th,k] = angvect_me(R)
r32 = R(3,2);
r23 = R(2,3);
r13 = R(1,3);
r31 = R(3,1);
r21 = R(2,1);
r12 = R(1,2);

th = acos((trace(R)-1)/2);
k = (1/(2*sin(th)))*[r32-r23;
                     r13-r31;
                     r21-r12];
end

function Q = quater(th,k)

Q =[cos(th/2) sin(th/2)*k'];
end