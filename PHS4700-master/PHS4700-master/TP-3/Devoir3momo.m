% Tir 1 : [Coup tf vbaf vbof wbof rbaf rbof] = Devoir3([6.85; 0.0; 6.85],[0; 0; 0],0.66)
% Tir 2 : [Coup tf vbaf vbof wbof rbaf rbof] = Devoir3([28; 0.5; 10],[0; 0; 0],1.1)
% Tir 3 : [Coup tf vbaf vbof wbof rbaf rbof] = Devoir3([6.85; 0.0; 6.85],[0; 2.3; 0],0.66)
% Tir 4 : [Coup tf vbaf vbof wbof rbaf rbof] = Devoir3([28; 0.5; 10],[0; 2.3; 0],1.1)

% liste d'informations de sorties (a enlever a la fin) -stpham
% TIR 1: collision B
% TIR 2: collisions cylindrique
% TIR 3: collision AH
% TIR 4: collision cylindrique
function [Coup tf vbaf vbof wbof rbaf rbof] = Devoir3momo(vbal,wboi,tl)

  ## CONSTANTES
  k = 0.1;          % kg/(m^2*s)
  g = 9.8;          % m/s^2
  e = 0.5;          % coefficient de resitution
  t_i = 0;          % s
  epsilon = 0.5;

  % constantes de la boite
  m_bo = 0.075;     % kg
  h_bo = 0.15;      % m
  R_bo = 0.061237;  % m     = h_bo/(6^(1/2)) = 6.1237 cm
  A_bo = R_bo^2 + h_bo^2; %m^2

  % constate de la balle
  R_ba = 0.0335;    % m
  m_ba = 0.058;     % kg    = 58g
  A_ba = pi*R_ba^2; % m^2

  % position initiales offertes
  r0_ba = [0 0 2];
  r0_bo = [3 0 10];

  % liste de constantes calculable
  fg_bo = [0 0 -m_bo*g];    % N
  fg_ba = [0 0 -m_ba*g];    % N
  # fin CONSTANTES

  t = 0;          % temps du systeme au complet
  tba = 0;        % temps de la balle (commence a tl pour le t)
  ar = wboi(2)*t; % angle en radian (determine la rotation de la boite)
  % cette position est celui dans le systeme du cylindre
  rc = [0 0 0];
  % cette position est celle du point d'impact dans le systeme d'origine
  rco = [0 0 0];
  Coup = 0;
  deltat = 0.0005;        % deltat de depart puisque tout est statique avant la simulation
  facteurPrecision = 3;   % 1 pour precision en 0.1m, 2 pour 0.01m, 3 pour 0.001m(1mm) et 4 pour 0.0001m(0.1mm)

  while true

    # CM DE LA BOITE
    rzboi = rzbo(t);
    rbo = [r0_bo(1) r0_bo(2) rzboi];

    # MOUVEMENT PROJECTILE DE LA BALLE
    if (t >= tl)

      # POSITION DU CM DE LA BALLE
      rxbaf = rxyba(tba, vbal(1), r0_ba(1));
      rybaf = rxyba(tba, vbal(2), r0_ba(2));
      rzbaf = rzba(tba, vbal(3), r0_ba(3));
      rba = [rxbaf rybaf rzbaf];

      % position de la balle rotater et translater
      rbart = systemeCylindre(rbo, rba, ar);

      # CONDITION DE COLLISION
      % collision par arete du haut
      if (collisionAH(rbart))
        Coup = 1;
        rc = rcAH(rbart);
        % STPHAM changement potentiel :
        normal = (rbart - rc)/norm(rbart - rc);
        break;
      endif
      % collision par arete du bas
      if (collisionAB(rbart))
        rc = rcAB(rbart);
        Coup = 1;
        % STPHAM changement potentiel :
        normal = (rbart - rc)/norm(rbart - rc);
        break;
      endif
      % collision par surface haut
      if (collisionSH(rbart))
        rc = rcSH(rbart);
        Coup = 1;
        normal = [0 0 1];
      endif
      % collision par surface du bas
      if (collisionSB(rbart))
        rc = rcSB(rbart);
        Coup = 1;
        normal = [0 0 -1];
      endif
      % collision par surface cylindrique
      if (collisionSC(rbart))
        rc = rcSC(rbart);
        Coup = 1;
        % normal = (rba - rc)/norm(rba - rc);
        % STPHAM changement potentiel :
        normal = (rbart - rc)/norm(rbart - rc);
      endif
      % Verifier si la balle depasse le sol
      if (rzbaf <= 0)
        break;
      endif
      # FIN CONDITION DE COLLISION
      
      % On calcule le delta en fonction du deplacement du la boite
      vbo = [0 0 vzbo(t)];
      deltat = ajusterDeltat(facteurPrecision, vbo);

      # CONDITION DE PRECISION
      if (Coup == 1)
        % Si collision et on est precis au millimetre pres, on peut arreter la simulation
        if (facteurPrecision == 4)
          break;
        % Si collision mais pas encore precis au millimetre pres,
        % on retourne dans le temps pour recalculer le point avant cette collision
        % on calcule le nouveau delta avec un facteur de precision pour parcourir une distance plus petite(donc plus precis)
        else
          vbaf = [vxyba(tba, vbal(1)) vxyba(tba, vbal(2)) vzba(tba, vbal(3))];  % on calcule la vit de l'iteration courante
          tba = tba - deltat;                                                   % On revient sur le dernier point avant la collision
          facteurPrecision = facteurPrecision + 1;
        endif;
      % Si Aucune collision, on poursuit à la prochaine itération/intervalle de temps
      else
        % incrementaton du temps de la balle, On passe au prochaine intervalle
        % Pas besoin de sauver le point puisque on travaille avec l'integrale. A la place on va decrementer le temps pour trouver la position precedente
        vbaf = [vxyba(tba, vbal(1)) vxyba(tba, vbal(2)) vzba(tba, vbal(3))];
      endif
      # FIN CONDITION DE PRECISION

      % On calcule le deltat en fonction du deplacement de la balle
      deltatvBa = ajusterDeltat(facteurPrecision, vbaf);          % On calcule delta t en fonction de la precision que la balle va touche la boite sur les axes de la vitesse

      % On compare le deltat entre celui calculee avec la balle et celui avec la boite
      if (deltatvBa <= deltat)
        deltat = deltatvBa;
      endif
      % incrementaton du temps de la balle
      tba = tba + deltat;
    endif

    # FIN MOUVEMENT PROJECTILE DE LA BALLE
    # incrementation du temps et de ar
    t = t + deltat;
    ar = wboi(2)*t;
  endwhile
  
  vbaf = [vxyba(tba, vbal(1)) vxyba(tba, vbal(2)) vzba(tba, vbal(3))]';
  vbof = [0 0 vzbo(t)]';
  rbof = [r0_bo(1) r0_bo(2) rzbo(t)];
  rbaf = [rxbaf rybaf rzbaf];
  wbof = wboi;

  if (Coup == 1)
    tf = t;
    % vitesseA_P = vbaf;
    vitesseA_P = rotater(-ar, vbaf');
    % vitesseB_P = vitesseRelativeP(vbof', wboi, rbof, rc);
    vitesseB_P = rotater(-ar,vbof') + (cross(rotater(-ar,wboi'),rc'));
    % vitesseB_P = rotater(-ar,vbof') + (cross(wboi,rc'))
    vitesseAvantCollision = dot(normal, (vitesseA_P - vitesseB_P));
    Ic_ba = CalculerIcSphere(m_ba, R_ba);
    Ic_bo = CalculerIcCylindre(m_bo, R_bo, h_bo);
    rbp = rc;
    rap = rbof+rbp-rbaf;
    G_ba = calculerG(normal, Ic_ba, rap);
    G_bo = calculerG(normal, Ic_bo, rbp);
    impulsion = calculeJ(epsilon, m_bo, m_ba, vitesseAvantCollision, calculerAlpha(m_ba, m_bo, G_ba, G_bo));
    
    % Doit devenir vbaf(1:3,2)
    % STPHAM changement potentiel :
    %  vitesseApresCollisionBalle = vitesseA_P + impulsion*((normal/m_ba)' + cross( (inv(Ic_ba) * cross(rap',normal')),rap'));
    vitesseApresCollisionBalle = vitesseA_P + impulsion*((normal/m_ba)' + cross( (inv(Ic_ba) * cross(rap',normal')),rap'));
    
    % Doit devenir vbof(1:3,2)
    % STPHAM changement potentiel :
    % vitesseApresCollisionBoite = vitesseB_P' - impulsion*((normal/m_bo)'+cross((inv(Ic_bo)*cross(rbp',normal')),rbp'));
    vitesseApresCollisionBoite = vitesseB_P - impulsion*((normal/m_bo)'+cross((inv(Ic_bo)*cross(rbp',normal')),rbp'));
    
    % Vitesse angulaire finale
    wbofcyl = rotater(-ar,wboi') - (impulsion*inv(Ic_bo)*(cross(rbp',normal')));
    wbof = rotater(ar,wbofcyl');
    % wbof = wboi - (impulsion*inv(Ic_bo)*(cross(rbp',normal')));
    % Vitesse angulaire finale de la balle (pas une sortie)
    wbaf = 0 + impulsion*inv(Ic_ba)*cross(rap',normal');
   
    vbaf(1:3,1) = vbaf;
    % vbaf(1:3,2) = vitesseApresCollisionBalle - (cross(wbaf, rap'));
    % vbafcyl = vitesseApresCollisionBalle - (cross(wbaf', rap'));
    vbafcyl = vitesseApresCollisionBalle - (cross (wbaf, rap'));
    vbaf(1:3,2) = rotater(ar, vbafcyl');
    
    vbof(1:3,1) = vbof;
    % vbof(1:3,2) = vitesseApresCollisionBoite - (cross(wbof, rbp'));
    vbofcyl = vitesseApresCollisionBoite - (cross(wbofcyl, rbp'));
    vbof(1:3,2) = rotater(ar, vbofcyl');
    
  endif

  % Tracer les objets lors de la l'arret de la simulation dont
  tracerBoite(rbof, ar);
  tracerBalle(rbaf);

endfunction

function rbart = systemeCylindre(rbo, rba, ar)
  # AJUSTEMENT A LORIGINE ET POINT DE REFERENCE CM DU CYLINDRE
  % direction du cm boite vers cm balle
  vboba = direction(rbo, rba);
  % rotater vers axe orgine du cylindre (-ar car retour vers etat initial)
  vbobar = rotater(-ar, vboba)';
  % position rotater de la balle
  rbar = rbo + vbobar;
  % direction du cm du cm vers origine
  vboo = direction(rbo, [0 0 0]);
  % position de la balle refere a lorigine
  rbart = rbar + vboo;
endfunction

function estCollision = collisionSH(rbart)
  h_bo = 0.15;      % m
  R_bo = 0.061237;  % m     = h_bo/(6^(1/2)) = 6.1237 cm
  R_ba = 0.0335;    % m
  dxy = distxy(rbart);
  estDansPlan = dxy < R_bo;
  estH = rbart(3) > 0;
  estCollisionSH = rbart(3) - (h_bo/2) <= R_ba;
  estCollision = estDansPlan && estH && estCollisionSH;
  if (estCollision)
    fprintf('Collision a la surface du haut. \n');
  endif
endfunction

function rc = rcSH(rbart)
  h_bo = 0.15;      % m
  rc = [rbart(1) rbart(2) (h_bo/2)];
endfunction

function estCollision = collisionSB(rbart)
  h_bo = 0.15;      % m
  R_bo = 0.061237;  % m     = h_bo/(6^(1/2)) = 6.1237 cm
  R_ba = 0.0335;    % m
  dxy = distxy(rbart);
  estDansPlan = dxy < R_bo;
  estB = rbart(3) < 0;
  estCollisionSB = rbart(3) + (h_bo/2) >= -R_ba;
  estCollision = estDansPlan && estB && estCollisionSB;
  if (estCollision)
    fprintf('Collision a la surface du bas. \n');
  endif
endfunction

function rc = rcSB(rbart)
  h_bo = 0.15;      % m
  rc = [rbart(1) rbart(2) (-h_bo/2)];
endfunction

function estCollision = collisionSC(rbart)
  h_bo = 0.15;      % m
  R_bo = 0.061237;  % m     = h_bo/(6^(1/2)) = 6.1237 cm
  R_ba = 0.0335;    % m
  dxy = distxy(rbart);
  estEntrePlansZ = (rbart(3) < h_bo/2) && (rbart(3) > -h_bo/2);
  estCollisionCylindrique = dxy <= R_bo + R_ba;
  estCollision = estEntrePlansZ && estCollisionCylindrique;
  if (estCollision)
    fprintf('Collision a la surface cylindrique. \n');
  endif
endfunction

function rc = rcSC(rbart)
  R_bo = 0.061237;  % m     = h_bo/(6^(1/2)) = 6.1237 cm
  dxy = distxy(rbart);
  C = R_bo/dxy;
  x = rbart(1)*C;
  y = rbart(2)*C;
  rc = [x y rbart(3)];
endfunction

function estCollision = collisionAH(rbart)
  h_bo = 0.15;      % m
  R_bo = 0.061237;  % m     = h_bo/(6^(1/2)) = 6.1237 cm
  R_ba = 0.0335;    % m
  dxy = distxy(rbart);
  cst1 = rbart(3) - (h_bo/2);
  cdn1 = (cst1 >= 0) && (cst1 <= R_ba);
  cdn2 = (dxy >= R_bo) && (dxy <= R_bo + R_ba);
  Rcontact = (((R_ba)^2)-((rbart(3)-(h_bo/2))^2))^(1/2);
  estCollisionA = dxy <= R_bo + Rcontact;
  estCollision = cdn1 && cdn2 && estCollisionA;
  if (estCollision)
    fprintf('Collision a arete du haut. \n');
  endif
endfunction

function rc = rcAH(rbart)
  R_bo = 0.061237;  % m     = h_bo/(6^(1/2)) = 6.1237 cm
  h_bo = 0.15;      % m
  dxy = distxy(rbart);
  C = R_bo/dxy;
  x = rbart(1)*C;
  y = rbart(2)*C;
  rc = [x y (h_bo/2)];
endfunction

function estCollision = collisionAB(rbart)
  h_bo = 0.15;      % m
  R_bo = 0.061237;  % m     = h_bo/(6^(1/2)) = 6.1237 cm
  R_ba = 0.0335;    % m
  dxy = distxy(rbart);
  cst1 = rbart(3) + (h_bo/2);
  cdn1 = (cst1 >= -R_ba) && (cst1 < 0);
  cdn2 = (dxy >= R_bo) && (dxy <= R_bo + R_ba);
  Rcontact = (((R_ba)^2)-((rbart(3)+(h_bo/2))^2))^(1/2);
  estCollisionA = dxy <= R_bo + Rcontact;
  estCollision = cdn1 && cdn2 && estCollisionA;
  if (estCollision)
    fprintf('Collision a arete du bas. \n');
  endif
endfunction

function rc = rcAB(rbart)
  R_bo = 0.061237;  % m     = h_bo/(6^(1/2)) = 6.1237 cm
  h_bo = 0.15;      % m
  dxy = distxy(rbart);
  C = R_bo/dxy;
  x = rbart(1)*C;
  y = rbart(2)*C;
  rc = [x y (-h_bo/2)];
endfunction

function dxy = distxy(rbart)
  dxy = (rbart(1)^2 + rbart(2)^2)^(1/2);
endfunction

% fonction pour obtenir la direction(vecteur) entre deux point
function v = direction(i, f)
  v = f - i;
endfunction

% fonction pour effectuer une rotation en y (car le w est juste sur le y)
function vdirection = rotater(ar, translation)
  Ry = [cos(ar) 0 sin(ar); 0 1 0; -sin(ar) 0 cos(ar)];
  vdirection = Ry*translation';
endfunction

% determine la vitesse de la boite en z par rapport au temps
function vz = vzbo(t)
  k = 0.1;          % kg/(m^2*s)
  g = 9.8;          % m/s^2

  m_bo = 0.075;     % kg
  h_bo = 0.15;      % m
  R_bo = 0.061237;  % m     = h_bo/(6^(1/2)) = 6.1237 cm
  A_bo = R_bo^2 + h_bo^2; %m^2

  % calculer a la main ou calculable sur un calculateur d'integral
  vz = ((m_bo*g)/(k*A_bo))*((exp((-k*A_bo*t)/(m_bo)))-1);
endfunction

% determine la position en z de la boite par rapport au temps
function rz = rzbo(t)
  k = 0.1;          % kg/(m^2*s)
  g = 9.8;          % m/s^2

  m_bo = 0.075;     % kg
  h_bo = 0.15;      % m
  R_bo = 0.061237;  % m     = h_bo/(6^(1/2)) = 6.1237 cm
  A_bo = R_bo^2 + h_bo^2; %m^2
  r0_bo = [3 0 10];

  % calculer a la main ou calculable sur un calculateur d'integral
  rz = (((m_bo^2)*g)/((k^2)*(A_bo^2)))*(1 - (exp((-k*A_bo*t)/(m_bo)))) - ((m_bo*g*t)/(k*A_bo)) + r0_bo(3);
endfunction

% determine la vitesse de la balle en x ou ypar rapport au temps
function vxy = vxyba(t, v0xy)
  k = 0.1;          % kg/(m^2*s)
  g = 9.8;          % m/s^2

  R_ba = 0.0335;    % m
  m_ba = 0.058;     % kg    = 58g
  A_ba = pi*R_ba^2; % m^2

  % calculer a la main ou calculable sur un calculateur d'integral
  vxy = v0xy*exp((-k*A_ba*(t))/(m_ba));

endfunction

% determine la position de la balle en x ou ypar rapport au temps
function rxy = rxyba(t, v0xy, xy0)
  k = 0.1;          % kg/(m^2*s)
  g = 9.8;          % m/s^2

  R_ba = 0.0335;    % m
  m_ba = 0.058;     % kg    = 58g
  A_ba = pi*R_ba^2; % m^2

  % calculer a la main ou calculable sur un calculateur d'integral
  rxy = ((m_ba*v0xy)/(k*A_ba))*(1 - exp((k*A_ba*(-t))/(m_ba))) + xy0;

endfunction

% determine la vitesse de la balle en z par rapport au temps
function vz = vzba(t, v0z)
  k = 0.1;          % kg/(m^2*s)
  g = 9.8;          % m/s^2

  R_ba = 0.0335;    % m
  m_ba = 0.058;     % kg    = 58g
  A_ba = pi*R_ba^2; % m^2

  % calculer a la main ou calculable sur un calculateur d'integral
  vz = ((k*A_ba*v0z + m_ba*g)*exp((-k*A_ba*(t))/(m_ba)) - m_ba*g)/(k*A_ba);

endfunction

% determine la position de la balle en z par rapport au temps
function rz = rzba(t, v0z, z0)
  k = 0.1;          % kg/(m^2*s)
  g = 9.8;          % m/s^2

  R_ba = 0.0335;    % m
  m_ba = 0.058;     % kg    = 58g
  A_ba = pi*R_ba^2; % m^2

  % calculer a la main ou calculable sur un calculateur d'integral
  rz = ((m_ba)/((k*A_ba)^2))*(k*A_ba*v0z + m_ba*g)*(1 - exp((k*A_ba*(-t))/(m_ba))) - (m_ba*g*(t))/(k*A_ba) + z0;

endfunction

%vitesse relative au point P
function vitesseRelP = vitesseRelativeP(vitesse, vAngulaire, pointA, pointP)
 % de a � p
 % rxp = pointP - pointA;
 vitesseRelP = vitesse + (cross(vAngulaire',pointP));
endfunction

% Calcul de l'impulsion
function impulsion = calculeJ(epsilon, masseA, masseB, vitesse, alpha)
  % 1 + epsilon = 1.5
impulsion = (-1)*alpha*((1+epsilon)*vitesse);
endfunction

function alpha = calculerAlpha(masseA, masseB, Ga, Gb)
  alpha = 1/((1/masseA)+(1/masseB)+Ga+Gb);
endfunction

function G = calculerG(normale, Ic, position)
  % test = inv(Ic)*cross(position',normale');
  % cross = cross(test,position');
  % G = dot(normale', cross);
  G = dot(normale', cross((inv(Ic)*cross(position',normale')),position'));
endfunction

function IcSphere = CalculerIcSphere(masse, rayon)
  I  = [1, 0, 0; 0, 1, 0; 0, 0, 1];
  IcSphere = ((2*masse/3)*rayon^2)*I;
endfunction

function IcCylindre = CalculerIcCylindre(masse, rayon, longueur)
  xy = (masse/2)*rayon^2 + (masse/12)*longueur^2;
  z = masse*rayon^2;
  IcCylindre = [xy, 0, 0; 0, xy, 0; 0, 0, z];
endfunction

function tracerBoite(boite, ar)
  hold on;
  [boxx, boyy, bozz] = cylinder([0.061237 0.061237]);
  hSurfTop = surf(boxx+boite(1), boyy+boite(2), bozz*0.075+boite(3));
  hSurfBot = surf(boxx+boite(1), boyy+boite(2), bozz*-0.075+boite(3));
  direction = [0 1 0];
  rotate(hSurfTop, direction, rad2deg(ar), boite);
  rotate(hSurfBot, direction, rad2deg(ar), boite);
  title('Simulation de la balle avec une boite de conserve');
  xlabel('x(m)');
  ylabel('y(m)');
  zlabel('z(m)');
  hold off;
endfunction

function tracerBalle(balle)
  hold on;
  [baxx, bayy, bazz] = sphere();
  surf(baxx*0.0335+balle(1), bayy*0.0335+balle(2), bazz*0.0335+balle(3));
  hold off;
endfunction

function deltat = ajusterDeltat(facteurPrecision, vitesse)

  % On prend la vitesse la plus grande sur les 3 axes (x, y et z)
  % Elle sera celle qui demandera plus de precision
  vitMax = 0;
  if (abs(vitesse(1)) >= vitMax)
    vitMax = vitesse(1);
  endif
  if (abs(vitesse(2)) >= vitMax)
    vitMax = vitesse(2);
  endif
  if (abs(vitesse(3)) >= vitMax)
    vitMax = vitesse(3);
  endif

  % On determine l'intervalle de temps selon la precision de distance et la vitesse :
  % x = vit*deltat =>  deltat = x/vit
  if (facteurPrecision == 1)
    deltat = abs(0.1 / vitMax);
  elseif (facteurPrecision == 2)
    deltat = abs(0.01 / vitMax);
  elseif (facteurPrecision == 3)
    deltat = abs(0.001 / vitMax);
  elseif (facteurPrecision == 4)
    deltat = abs(0.0001 / vitMax);
  endif
endfunction
