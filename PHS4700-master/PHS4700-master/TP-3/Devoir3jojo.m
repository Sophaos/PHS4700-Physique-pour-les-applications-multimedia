% Tir 1 : [Coup tf vbaf vbof wbof rbaf rbof] = Devoir3([6.85; 0.0; 6.85],[0; 0; 0],0.66)
% Tir 2 : [Coup tf vbaf vbof wbof rbaf rbof] = Devoir3([28; 0.5; 10],[0; 0; 0],1.1)
% Tir 3 : [Coup tf vbaf vbof wbof rbaf rbof] = Devoir3([6.85; 0.0; 6.85],[0; 2.3; 0],0.66)
% Tir 4 : [Coup tf vbaf vbof wbof rbaf rbof] = Devoir3([28; 0.5; 10],[0; 2.3; 0],1.1)
function [Coup tf vbaf vbof wbof rbaf rbof] = Devoir3jojo(vbal,wboi,tl)

  # CONSTANTES
  k = 0.1;          % kg/(m^2*s)
  g = 9.8;          % m/s^2
  e = 0.5;          % coefficient de resitution
  t_i = 0;          % s

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



  view(30, 20);
  #axis([-10 10 -10 10 0 10]);
  title('Boite de conserve et balle');



  boite = [3, 0, 10];

  balle = [0, 0, 2];

  ar = [0, 0, 0];
  t = 0;
  vitBo = [0 0 0];
  vitBa = [vbal(1) vbal(2) vbal(3)];
  deltat = 0.0005;
  facteurPrecision = 1;

  collision = false; #remettre a false

  while (t < tl)
  	#tracerBoite(boite, ar);
  	vitBo = calculerVitBo(vitBo, t, deltat);
  	boite = boite + vitBo*deltat;
  	#t = round((t + deltat) .* 1000) ./ 1000;
    t = t + deltat;
  	ar = wboi*t;
  endwhile

  deltat = 0.001;

  dernierPosBo = boite;
  dernierVitBo = vitBo;
  dernierPosBa = r0_ba;
  dernierVitBa = vitBa;
  while true#not(collision) && facteurPrecision==3


  	vitBo = calculerVitBo(vitBo, t, deltat);
  	vitBa = calculerVitBa(vitBa, t, deltat);

    #boite = calculerPosBo(vitBo, t, deltat);
    #balle = calculerPosBa(vitBa, t, deltat);
  	boite = boite + vitBo*deltat;
  	balle = balle + vitBa*deltat;


  	rbart = systemeCylindre(boite, balle, ar(2));

  	Coup = verifierCollisions(rbart);
    if (boite(3) <= 0 || balle(3) <= 0)
      break;
    endif;

  	# Decommenter les deux lignes si on tracer la objects durant leur trajectoire
  	#tracerBoite(boite, ar);
  	#tracerBalle(balle);


    # CONDITION DE PRECISION
    if (Coup == 1)
      # Si collision et on est precis au millimetre pres, on peut arreter la simulation
      if (facteurPrecision == 3)
        break;
      # Si collision mais pas encore precis au millimetre pres,
      # on retourne dans le temps pour recalculer le point avant cette collision
      # on calcule le nouveau delta avec un facteur de precision pour parcourir une distance plus petite(donc plus precis)
      else
        t = t - deltat;
        facteurPrecision++;                       % On augmente la precision de fois 10
        deltat = ajusterDeltat(facteurPrecision, vitBa(1));          % On calcule delta t en fonction de la precision que la balle va touche la boite sur l'axe des x

        boite = dernierPosBo;         % On retourne à l'itération avant la collision
        vitBo = dernierVitBo;
        balle = dernierPosBa;
        vitBa = dernierVitBa;
      endif;
    # Si Aucune collision, on poursuit à la prochaine itération/intervalle de temps
    else
      % incrementaton du temps de la balle, On passe au prochaine intervalle
      deltat = ajusterDeltat(facteurPrecision, vitBa(1))          % On calcule delta t en fonction de la precision que la balle va touche la boite sur l'axe des x
      t = t + deltat#0.0005;
      % On sauve les données de la boite et de la balle
      dernierPosBo = boite;
      dernierVitBo = vitBo;
      dernierPosBa = balle;
      dernierVitBa = vitBa;
    endif
    # FIN CONDITION DE PRECISION

  	ar = wboi*t;




  endwhile

  tracerBoite(boite, ar);
  tracerBalle(balle);

  # sorties temporaires (incomplet)
  Coup = 1;
  # positionContact
  tf = t;
  vbaf = vitBa;
  vbof = vitBo;
  wbof = [0 0 0]; # A DETERMINER
  rbaf = balle;
  rbof = boite;
endfunction

function pos = calculerPosBo(vitBo, t0, deltat)
  vitPrecise = SEDRK4t0(vitBo, t0, deltat, 'calculVitBo');
  pos = pos + vitPrecise*deltat;
endfunction

function pos = calculerPosBa(vitBa, t0, deltat)
  vitPrecise = SEDRK4t0(vitBa, t0, deltat, 'calculVitBa');
  pos = pos + vitPrecise*deltat;
endfunction



function deltat = ajusterDeltat(facteurPrecision, vitesse)
  if (facteurPrecision == 1)
    deltat = 0.1 / vitesse;
  elseif (facteurPrecision == 2)
    deltat = 0.01 / vitesse;
  elseif (facteurPrecision == 3)
    deltat = 0.001 / vitesse;
  endif
endfunction

function collision = collisionSurfaceHautBas(vOrigineBa)
  posBa = vOrigineBa;
  rayonBa = 0.0335;
  rayonBo = 0.061237;
  hBo = 0.15;
  collision = false;
  if ((((posBa(1)^2 + posBa(2)^2)^(0.5) < rayonBo)) &&
		(posBa(3) <= hBo/2+rayonBa && posBa(3) >= -hBo/2-rayonBa))
			collision = true;
      fprintf('collision Haut bas. \n');
  endif
endfunction

function collision = collisionSurfacesCylindrique(vOrigineBa)
  posBa = vOrigineBa;
  rayonBa = 0.0335;
  rayonBo = 0.061237;
  hBo = 0.15;
  collision = false;
  if ((posBa(3) <= hBo/2 && posBa(3) >= -hBo/2) &&
		((posBa(1)^2 + posBa(2)^2)^(1/2) < rayonBa + rayonBo))
			collision = true;
      fprintf('collision surf cylindre. \n');
  endif
endfunction

function collision = collisionCerclesSphere(vOrigineBa);
  posBa = vOrigineBa;
  rayonBa = 0.0335;
  rayonBo = 0.061237;
  hBo = 0.15;
  collision = false;
  if (((posBa(1)^2 + posBa(2)^2)^(1/2) < rayonBa + rayonBo) &&
        ((posBa(3) < hBo/2 + rayonBa) && (posBa(3) > -hBo/2 - rayonBa) ))
        fprintf('collision arete. \n');
        collision = true;
  % if ((posBa(3) < hBo/2+rayonBa && posBa(3) > -hBo/2-rayonBa) &&
	% 	((posBa(1)^2 + posBa(2)^2)^(1/2) < rayonBa + rayonBo))
	%\ 		collision = true;
  endif
endfunction

function collision = verifierCollisions(vOrigineBa)
  colHauBas = collisionSurfaceHautBas(vOrigineBa);
  colSurCyl = collisionSurfacesCylindrique(vOrigineBa);
  colAretes = collisionCerclesSphere(vOrigineBa);

  collision = (colAretes || colSurCyl || colHauBas);
endfunction

function vf=calculerVitBo(vitBo, t0, deltat)
  accel = SEDRK4t0(vitBo, t0, deltat, 'calculAccelBo');
  vf = vitBo + accel*deltat;
endfunction

function vf=calculerVitBa(vitBa, t0, deltat)
  accel = SEDRK4t0(vitBa, t0, deltat, 'calculAccelBa');
  vf = vitBa + accel*deltat;
endfunction

% Solution equations differentielles par methode de RK4
% Equation a resoudre : dq/dt=g(q,t)
% avec
%   qs        : solution [q(to+Deltat)]
%   q0        : conditions initiales [q(t0)]
%   Deltat    : intervalle de temps
%   g         : membre de droite de ED.
%               Cest un m-file de matlab
%               qui retourne la valeur de g au temps choisi
% Dans cette fonction, on trouve la moyenne dans lordre 4 des accelerations
% que l'on calcule : k1 k2 k3 k4.
% Avec cette nouvelle acceleration plus precises, on determine la prochaine vitesse
% en l'utilisant comme  pente.
function qs=SEDRK4t0(q0,t0,Deltat,g)
  k1=feval(g,q0,t0);
  tmp = q0 + [k1*Deltat/2];
  k2=feval(g,tmp,t0+Deltat/2);
  tmp = q0 + [k2*Deltat/2];
  k3=feval(g,tmp,t0+Deltat/2);
  tmp = q0 + [k3*Deltat/2];
  k4=feval(g,tmp,t0+Deltat);
  qs=(k1+2*k2+2*k3+k4)/6;
endfunction

function accel = calculAccelBo(vit, t)
  fv = calculerFvisqBo(vit);
  fg = [0 0 0.075*-9.8];
  accel = (fv+fg)/0.075;
endfunction

function accel = calculAccelBa(vit, t)
  fv = calculerFvisqBa(vit);
  fg = [0 0 0.058*-9.8];
  accel = (fv+fg)/0.058;
endfunction

function tracerBoite(boite, ar)
  hold on;
  [boxx, boyy, bozz] = cylinder([0.061237 0.061237]);
  hSurfTop = surf(boxx+boite(1), boyy+boite(2), bozz*0.075+boite(3));
  hSurfBot = surf(boxx+boite(1), boyy+boite(2), bozz*-0.075+boite(3));
  direction = [0 1 0];
  rotate(hSurfTop, direction, rad2deg(ar(2)), boite);
  rotate(hSurfBot, direction, rad2deg(ar(2)), boite);
  hold off;
endfunction

function tracerBalle(balle)
  hold on;
  [baxx, bayy, bazz] = sphere();
  surf(baxx*0.0335+balle(1), bayy*0.0335+balle(2), bazz*0.0335+balle(3));
  hold off;
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


% fonction pour obtenir la direction(vecteur) entre deux point
function v = direction(i, f)
  v = f - i;
endfunction

% fonction pour effectuer une rotation en y (car le w est juste sur le y)
function vdirection = rotater(ar, translation)
  Ry = [cos(ar) 0 sin(ar); 0 1 0; -sin(ar) 0 cos(ar)];
  vdirection = Ry*translation';
endfunction


% inutile pour l'instant
function fVisqBa = calculerFvisqBa(v)
  k = 0.1;              % kg/(m^2*s)
  R_ba = 0.0335;        % m
  m_ba = 0.058;         % kg    = 58g
  A_ba = pi*R_ba^2;     % m^2

  fVisqBa = -k*A_ba*v;  % N

endfunction

% inutile pour l'instant
function fVisqBo = calculerFvisqBo(v)
  k = 0.1;                  % kg/(m^2*s)
  h_bo = 0.15;              % m
  R_bo = 0.061237;          % m     = h_bo/(6^(1/2)) = 6.1237 cm
  A_bo = R_bo^2 + h_bo^2;   % m^2

  fVisqBo = -k*A_bo*v;      % N

endfunction
