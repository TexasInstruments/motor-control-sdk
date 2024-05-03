
%
%  Copyright (C) 2024 Texas Instruments Incorporated
%
%  Redistribution and use in source and binary forms, with or without
%  modification, are permitted provided that the following conditions
%  are met:
%
%    Redistributions of source code must retain the above copyright
%    notice, this list of conditions and the following disclaimer.
%
%    Redistributions in binary form must reproduce the above copyright
%    notice, this list of conditions and the following disclaimer in the
%    documentation and/or other materials provided with the
%    distribution.
%
%    Neither the name of Texas Instruments Incorporated nor the names of
%    its contributors may be used to endorse or promote products derived
%    from this software without specific prior written permission.
%
%  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
%  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
%  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
%  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
%  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
%  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
%  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
%  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
%  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
%  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
%  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%
% ###########################################################################

clc;
clear all;
close all;

%Read the FRA Plant data from the excel sheet%
Freq=xlsread('SFRA','Sheet1','A2:A102');
Phase_H=xlsread('SFRA','Sheet1','E2:E102');
Mag_H=xlsread('SFRA','Sheet1','D2:D102');

%Convert the FRA data to a bode object in MATLAB%
Mag_H=(10.^(Mag_H./20));
Phase_H=pi.*Phase_H./180;
Plant_Bode_Obj=frd(Mag_H.*exp(i*(Phase_H)),Freq,'FrequencyUnit','Hz');

%Curve fit the data to get a transfer function%
% user may choose to fit 3p3z or any other depending on what works best%
[Hn,Hd] = invfreqs(Mag_H.*exp(i*(Phase_H)),(2*pi).*Freq,2,3);
H = tf(Hn,Hd);
 
% plot both the measured and curve fitted TF to compare accuracy
figure(1);bode(H,Plant_Bode_Obj);title('Control-to-Output Measured Vs Curve Fitted');
legend('Curve Fitted','Measured')

%call sisotool to design the compensation with the transfer function%
sisotool(H)
