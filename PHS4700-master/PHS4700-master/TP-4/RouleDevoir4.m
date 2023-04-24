%
% Rouler Devoir 4
% Effets sonores
% 
%
format long
clear;
clf;
%
%  Définir les cas
%
tfac=1;
NSimulations=5;
vtrain(:,1)=[0;0;0];  % km/h
vtrain(:,2)=[0;0;0];  % km/h
vtrain(:,3)=[0;-250;0];  % km/h
vtrain(:,4)=[-250;0;0];  % km/h
vtrain(:,5)=[250;0;0];  % km/h
favion=[750 1500 750 750 750];
FileNamesf={'Test1f.eps','Test2f.eps','Test3f.eps','Test4f.eps','Test5f.eps'};
FileNamesI={'Test1I.eps','Test2I.eps','Test3I.eps','Test4I.eps','Test5I.eps'};
fprintf('\\begin{table}[h] \\begin{center} \\caption{R\\''{e}sultats des simulations.}\\label{SimRes}\n\\vspace{0.2cm}\n');
fprintf('\\begin{tabular}{|l||c|c|c||c|c|c||} \n \\hline &\\multicolumn{3}{c||}{Son initial}&\\multicolumn{3}{c||}{Son final} \\\\ \\cline{2-7}\n');
fprintf('Test & $t$ (s)  & $\\nu$ (Hz)& $I$ (dB)  & $t$ (s)& $\\nu$ (Hz)& $I$  (dB) \\\\ \\hline\n');
for isimulation=1:NSimulations
  clear ftrain Itrain;
  [tps ftrain Itrain]=Devoir4(vtrain(:,isimulation),favion(isimulation));
  ifirst=int8((tps+0.5)*tfac)+1;
  nstep=length(ftrain);
  plot([0:nstep-1]/tfac,ftrain(:),'k-',[0 nstep-1]/tfac,[favion(isimulation) favion(isimulation)],'r-');
  xlabel('t (s)');
  ylabel('\nu (Hz)');
  print('-depsc2',FileNamesf{isimulation})
  % pause; % ajout -STPHAM
  plot([0:nstep-1]/tfac,Itrain(:),'k-');
  xlabel('t (s)');
  ylabel('I (dB)');
  print('-depsc2',FileNamesI{isimulation})
  fprintf(' %4.0f  &  %5.0f &  %8.2f &  %8.2f &  %5.0f &  %8.2f &  %8.2f  \\\\ \n',isimulation,tps,ftrain(ifirst),Itrain(ifirst),(nstep-1)/tfac,ftrain(nstep),Itrain(nstep));
  % pause; % ajout -STPHAM
end;
fprintf('\\hline \n \\end{tabular}  \\end{center} \\end{table} \n \n \n');

