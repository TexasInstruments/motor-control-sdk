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

P=bodeoptions; 
P.Grid='on';
P.FreqUnits='Hz';
P.PhaseWrapping='on';
P.YlimMode='auto';
P.XlimMode='manual';
P.Xlim=[2,40000];
P.Title.FontSize=12;
P.Title.FontWeight='bold';
P.XLabel.FontSize=11;
P.XLabel.FontWeight='bold';
P.YLabel.FontSize=11;
P.YLabel.FontWeight='bold';
P.TickLabel.FontSize=10;


%Read the FRA Plant data from the excel sheet%
Freq=xlsread('SFRA','Sheet1','A2:A102');
Phase_H=xlsread('SFRA','Sheet1','E2:E102');
Mag_H=xlsread('SFRA','Sheet1','D2:D102');
Phase_GH=xlsread('SFRA','Sheet1','C2:C102');
Mag_GH=xlsread('SFRA','Sheet1','B2:B102');

%Convert the FRA data to a bode object in MATLAB%
Mag_H=(10.^(Mag_H./20));
Phase_H=pi.*Phase_H./180;
Plant_Bode_Obj=frd(Mag_H.*exp(i*(Phase_H)),Freq,'FrequencyUnit','Hz');

Mag_GH=(10.^(Mag_GH./20));
Phase_GH=pi.*Phase_GH./180;
OL_Bode_Obj=frd(Mag_GH.*exp(i*(Phase_GH)),Freq,'FrequencyUnit','Hz');

s=tf('s');
Gi=0.35035*(s+199.89)/s;
Fs=100000;
Ts=1/Fs;

%from the pole zer format get the Kp and Ki for the DCL series form PI
alpha=0.35035;
beta=199.89;
Kp=alpha*(1-beta*Ts/2);
Ki=alpha*beta*Ts;
%if using seried form
Ki=Ki/Kp;
[n,d]=pade(Ts,4);
Gd=tf(n,d);

% plot both the measured and curve fitted TF to compare accuracy
figure(1);bode(Plant_Bode_Obj,Gd,P);title('Plant Frequency Response');
h=findobj(gcf,'type','line');
set(h,'linewidth',2);
legend('boxoff');
legend('Measured','Modelled');
 
figure(2);bode(OL_Bode_Obj,Gi*Gd,P);title('Open Loop Frequency Response');
legend('Curve Fitted','Measured');
h=findobj(gcf,'type','line');
set(h,'linewidth',2);
legend('boxoff');
legend('Measured','Modelled');

