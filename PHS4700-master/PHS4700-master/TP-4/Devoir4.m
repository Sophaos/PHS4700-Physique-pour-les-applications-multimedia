% Simulation 1: [tps ftrain Itrain]=Devoir4([0;0;0],750)
% Simulation 2: [tps ftrain Itrain]=Devoir4([0;0;0],1500)
% Simulation 3: [tps ftrain Itrain]=Devoir4([0;-250;0],750)
% Simulation 4: [tps ftrain Itrain]=Devoir4([-250;0;0],750)
% Simulation 5: [tps ftrain Itrain]=Devoir4([250;0;0],750)

function [tps ftrain Itrain]=Devoir4(vtrainkmh,fAvion)

  # NOMENCLATURE
  % _s   -> source
  % _r   -> recepteur
  % L,i  -> intensite
  % va|vt-> vitesse
  % r    -> distance
  % f    -> frequence
  % ra   -> position avion
  % rt   -> position train
  % MS   -> m/s
  % KH   -> km/h

  # CONSTANTES
  % theta = 10;                               % centigrade
  % cson = (331.3 + 0.606*theta);             % m/s
  % CAv = (0.8 + 0.0041*fAvion);              % db/km
  % r0t = [10; 10; 0];                        % km
  vaKH = 300*[cos(pi/18); 0; sin(pi/18)];     % km/h
  vt = (1000/3600)*(vtrainkmh);               % m/s
  va = (1000/3600)*(vaKH);                    % m/s

  # DEBUT remplissage de ftrain et Itrain
  t = 0.0;                  % temps commence a 0;
  dt = 1.0;                 % delta t = 1         (specifier par enonce)
  ftrain = [];              % tableau pour accueillir les valeurs ftrain
  Itrain = [];              % tableau pour accueillir les valeurs Itrain
  IL = 20;                  % intensite limite a 20 dB (doit etre inferieur)
  while true

    % frequences...
    ft = fr(t, va, vt, fAvion);             % calcul de la frequence du recepteur (train)
    ftrain = [ftrain, ft];                  % remplissage du tableau ftrain

    % intensites...
    ra = rat(t);                            % calcul pos pour avion
    rt = rtt(t, vt);                        % calcul pos pour train
    r = norm(rt - ra);                      % calcul distance avion -> train
    It = Li(r, fAvion);                     % calcul de l'intensite
    Itrain = [Itrain, It];                  % remplissage du tableau Itrain

    if (length(Itrain) >= 2 )               % necessite deux valeurs pour trouver la pente
      di = Itrain(end) - Itrain(end - 1);     % delta i entre les deux derniere valeurs
      if ((di < 0) && (Itrain(end) < IL))     % delta i negatif ET en bas de valeur limite
        break;                                  % FIN
      endif
    endif
    t = t + dt;                             % incrementation du temps par 1 (offert par enonce)
  endwhile
  # FIN remplissage de ftrain et Itrain

  # DEBUT determination du tps
  r0a = [0; 0; 0];                % position initial en m
  theta = 10;                     % necessaire pour son en centigrade
  cson = (331.3 + 0.606*theta);   % vitesse du son en m/s
  tps = 0;                        % temps du premier bip qui atteint le recepteur
  dt = 1;                         % delta t (pas preciser dans enonce) en s (pris 1 car res montre en sec)
  while true
    rt = rtt(tps, vt);            % calcul position du train
    rtps = norm(rt - r0a);        % calcul distance entre train et position initial de l'avion
    dbip = cson*tps;              % calcul distance parcouru par le son
    if (dbip >= rtps)             % si distance parcouru >= distance a->t
      break;                        % FIN
    endif
    tps = tps + dt;
  endwhile
  # FIN determination du tps

  % trajet physiques (pour analyse des resultats)
  % faireTrajetAvion(ra);
  % faireTrajetTrain(rt);

  % sorties temporaire
  % ftrain = 1;
  % Itrain = 1;
  % tps = 1;
endfunction


% coefficient (disp et pertes d'energie) ceci inclu les deux voir note chap 8 p.(21/108)
% fs en Hz
function CAv = A(fs)
  CAv = (1/1000)*(0.8 + 0.0041*fs);      % dB/m   -- conversion db/km -> db/m
endfunction

% voir note chap 8 p.(20/108) et % doc p.197 (27)
% intensite sonore 
% r en m --- fs en Hz
function L_I = Li(r, fs)
  % intensite sonore constante de 160dB a 100m
  Li100 =  160;   % dB
  r0 = 100;       % m
  L_I = Li100 - 20*log(r/r0) - A(fs)*(r - r0);     % en dB
endfunction

% voir note chap 8 p.(29/108)
% calcul de la direction unitaire
% t en s  % vr en m/s
function U_SR = usr(t, vr)
  rr = rtt(t, vr);
  rs = rat(t);
  U_SR = (rr - rs)/(norm(rr - rs));
endfunction

% voir note chap 8 p.(29/108)
% calcul du beta pour r ou s
% v en m/s --- vr en m/s --- t en s
function B_SR = bsr(v, vr, t)
  theta = 10;                           % centigrade
  cson = (331.3 + 0.606*theta);         % m/s
  u = usr(t, vr);
  B_SR = (dot(v, u))/cson;
endfunction

% voir note chap 8 p.(29/108)
% calcul la frequence percue par le recepteur
% t en s --- vs en m/s --- vr en m/s --- fs en Hz
function F_R = fr(t, vs, vr, fs)
  br = bsr(vr, vr, t);
  bs = bsr(vs, vr, t);
  F_R = ((1 - br)/(1 - bs))*fs;   % Hz
endfunction

% position avion en fonction du temps (m/s)
% t en s
function ra = rat(t)
  r0a = [0; 0; 0];                          % m
  vaKH = 300*[cos(pi/18); 0; sin(pi/18)];   % km/h
  vaMS = (1000/3600)*(vaKH);                % m/s
  ra = r0a + vaMS*t;                        % m
endfunction

% position train en fonction du temps (m/s)
% t en s --- vtrainms en m/s
function rt = rtt(t, vtrainms)
  r0t = 1000*[10; 10; 0];                  % m
  rt = r0t + vtrainms*t;                   % m
endfunction

% Tracer le trajet de l'avions
% pour analyse
% rfa en m
function faireTrajetAvion(rfa)
  r0a = [0; 0; 0];      % m
  tableauX = [r0a(1), rfa(1)];
  tableauY = [r0a(2), rfa(2)];
  tableauZ = [r0a(3), rfa(3)];
  hold on
  plot3(tableauX, tableauY, tableauZ);
  hold off
endfunction

% Tracer le trajet du train
% pour analyse
% rft en m
function faireTrajetTrain(rft)
  r0t = [10000; 10000; 0];   % m
  tableauX = [r0t(1), rft(1)];
  tableauY = [r0t(2), rft(2)];
  tableauZ = [r0t(3), rft(3)];
  title('Simulation avion et train');
  xlabel('x(m)');
  ylabel('y(m)');
  zlabel('z(m)');
  hold on
  plot3(tableauX, tableauY, tableauZ);
  hold off
endfunction
