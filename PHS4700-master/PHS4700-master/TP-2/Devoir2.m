% pour run
% soyez dans le bon dossier pour run
% condition 1 : [But tf rf vf] = Devoir2([0.2,89.8,0.11],[5.3,-21,16.5],[0,0,6.3])
% condition 2 : [But tf rf vf] = Devoir2([0.2,89.8,0.11],[5.3,-21,16.5],[0,-5,-6.3])
% condition 3 : [But tf rf vf] = Devoir2([0.2,89.8,0.11],[5.3,-21,16.5],[0,0,-6.5])
% condition 4 : [But tf rf vf] = Devoir2([0.2,89.8,0.11],[5.3,-21,18.3],[0,0,-6.3])
% condition 5 : [But tf rf vf] = Devoir2([119.8,89.8,0.11],[-5.3,-21,16.5],[0,-3,6.3])

#parametres
%ri 3x1 position du cm
%vi 3x1 vitesse du ballon
%wi 3x1 vitesse angulaire du ballon

#sorties
%But est un entier
%tf temps de la simulation
%rf cm du ballon a la fin de la simulation
%vf du ballon a la fin de la simulation 

function [But tf rf vf ] = Devoir2 (ri, vi, wi)

  r_ballon = 0.11;          % m
  m_ballon = 0.45;          % kg
  d_but = 7.32;             % m
  g = 9.8;                  % m/s^2
  p_air = 1.2754;           % kg/m^3
  mu_air = 1.8*10^-5;       % kg/m*s
  F_g = [0 0 -m_ballon*g];  % N = kg*m/s^2
  A_ballon = pi*r_ballon^2; % m^2

  z_ballon_sol = r_ballon;   % m
  z_t = 2.44;               % m
  d_poteaux = 7.32;         % m
  y_manquant = 90/2 - d_poteaux/2 - 5.5 - 11;
  y_poteau_1 = y_manquant + 11 + 5.5;
  y_poteau_2 = y_poteau_1 + d_poteaux;
  
  % zones de collision
  x_0_plane = 0;
  x_120_plane = 120;
  y_0_plane = 0;
  y_90_plane = 90;
  z_0_11_plane = r_ballon;


  % Données initiales
  q0 = [vi wi];
  t0 = 0;
  rf = ri;
  tableauX = [rf(1)];
  tableauY = [rf(2)];
  tableauZ = [rf(3)];
  enCollision = false;
  Delta = 0.1;
  precision = 0.01;
  But = 5;
  tf = t0;
  
  initialiserTerrainSoccer();
  FIN = false;
  while (!FIN)
    a_courante = SEDRK4t0(q0,t0,Delta, 'calculAccel');
    vf = calculVitesse(q0,a_courante,Delta);
    rf = calculPosition(ri, vf, Delta);  
    But = verificationCollisions(rf);
    
    % Precision pour l'erreur maximale
    if (But < 5 && 
      (abs(rf(1) - ri(1)) <= precision &&
      abs(rf(2) - ri(2)) <= precision &&
      abs(rf(3) - ri(3)) <= precision))
      tableauX = [tableauX, rf(1)];
      tableauY = [tableauY, rf(2)];
      tableauZ = [tableauZ, rf(3)];
      faireTrajet(tableauX, tableauY, tableauZ);
      FIN = true;
    elseif (But < 5)
      Delta /= 2;
    else 
      q0 = [vf wi];
      ri = rf;
      tableauX = [tableauX, rf(1)];
      tableauY = [tableauY, rf(2)];
      tableauZ = [tableauZ, rf(3)];
      faireTrajet(tableauX, tableauY, tableauZ);
      tf += Delta;
    endif
    
  endwhile
  
end

% Fonction qui demarre les verifications de collisions
% Returne le type de collision
function col = verificationCollisions(rf)
	col = 5;
    if (estContactAvecMontants(rf))
      col = -1;
    elseif (estContactAvecButs(rf))
      col = 1;
    elseif (estContactsAvecDehors(rf))
      col = -2;
    elseif (estContactAvecSol(rf))
      col = 0;
	  else
	    col = 5;
    endif
end


% Initialiser la vue du terrain du soccer une seule fois.
function initialiserTerrainSoccer()
   % Limites du graphe
  
  view(30, 20);
  
  axis([0 120 0 90 -40 40]);

  % dessin du terrain de soccer qui est vert
  patch([0, 120, 120, 0],[0, 0, 90, 90],[0, 0, 0, 0], 'green');
  title('Terrain de soccer avec trajectoire');

  hold on
  
  z_t = 2.44;               % m
  d_poteaux = 7.32;         % m
  y_manquant = 90/2 - d_poteaux/2 - 5.5 - 11;
  y_poteau_1 = y_manquant + 11 + 5.5;
  y_poteau_2 = y_poteau_1 + d_poteaux;

  % le dessin du poteau vertical 1
  plot3([0 0],[y_poteau_1 y_poteau_1],[0 z_t]);
  % le dessin du poteau vertical 2
  plot3([0 0],[y_poteau_2 y_poteau_2],[0 z_t]);
  % le dessin du poteau horizontal entre les poteaux 1 et 2
  plot3([0 0],[y_poteau_1 y_poteau_2],[z_t z_t]);
  
  % les memes dessins des poteaux pour l'autre but
  plot3([120 120],[y_poteau_1 y_poteau_1],[0 z_t]);
  plot3([120 120],[y_poteau_2 y_poteau_2],[0 z_t]);
  plot3([120 120],[y_poteau_1 y_poteau_2],[z_t z_t]);
  
  xlabel('x(m)');
  ylabel('y(m)');
  zlabel('z(m)');

  hold off

end

% Tracer le trajet du ballon sur le prochain nouveau deltat
function faireTrajet(tableauX, tableauY, tableauZ)
  hold on 
  % le trajet du ballon calcule sur deltat
  plot3(tableauX,tableauY,tableauZ);
  hold off
end

function estDansMontants = estContactAvecMontants(rf)
  r_ballon = 0.11;          % m
  z_t = 2.44;               % m
  d_poteaux = 7.32;         % m
  y_manquant = 90/2 - d_poteaux/2 - 5.5 - 11;
  y_poteau_1 = y_manquant + 11 + 5.5;
  y_poteau_2 = y_poteau_1 + d_poteaux;
  
  estDansPoteauxZ = (rf(3) > r_ballon && rf(3) <= z_t);
  estDansBarreY = ((rf(2) > y_poteau_1) && (rf(2) < y_poteau_2));
  estDansCoin1G = (norm(rf - [0;y_poteau_1;z_t]) <= r_ballon);
  estDansCoin2G = (norm(rf - [0;y_poteau_2;z_t]) <= r_ballon);
  estDansBarreG = (estDansBarreY && (((rf(1)- 0)^2 + (rf(3) - z_t)^2)^(1/2) <= r_ballon));
  estDansPoteau1G = (estDansPoteauxZ && (((rf(1)- 0)^2 + (rf(2) - y_poteau_1)^2)^(1/2) <= r_ballon));
  estDansPoteau2G = (estDansPoteauxZ && (((rf(1)- 0)^2 + (rf(2) - y_poteau_2)^2)^(1/2) <= r_ballon));
  
  estDansCoin1D = (norm(rf - [120;y_poteau_1;z_t]) <= r_ballon);
  estDansCoin2D = (norm(rf - [120;y_poteau_2;z_t]) <= r_ballon);
  estDansBarreD = (estDansBarreY && (((rf(1)- 0)^2 + (rf(3) - z_t)^2)^(1/2) <= r_ballon));
  estDansPoteau1D = (estDansPoteauxZ && (((rf(1)- 120)^2 + (rf(2) - y_poteau_1)^2)^(1/2) <= r_ballon));
  estDansPoteau2D = (estDansPoteauxZ && (((rf(1)- 120)^2 + (rf(2) - y_poteau_2)^2)^(1/2) <= r_ballon));
  
  estDansMontantG = estDansCoin1G || estDansCoin2G || estDansBarreG || estDansPoteau1G || estDansPoteau2G;
  estDansMontantD = estDansCoin1D || estDansCoin2D || estDansBarreD || estDansPoteau1D || estDansPoteau2D;
  
  estDansMontants = estDansMontantG || estDansMontantD;
end

function estDansButs = estContactAvecButs(rf)
  r_ballon = 0.11;          % m
  z_t = 2.44;               % m
  d_poteaux = 7.32;         % m
  y_manquant = 90/2 - d_poteaux/2 - 5.5 - 11;
  y_poteau_1 = y_manquant + 11 + 5.5;
  y_poteau_2 = y_poteau_1 + d_poteaux;
  x_0_plane = 0;
  x_120_plane = 120;
  z_0_11_plane = r_ballon;
  estDansButDx = (rf(1) >= x_120_plane);
  estDansButGx = (rf(1) <= x_0_plane);
  estDansButY = (rf(2) > y_poteau_1 + r_ballon && rf(2) < y_poteau_2 - r_ballon);
  estDansButZ = (rf(3) > z_0_11_plane && rf(3) < z_t - r_ballon);
  estDansButG = estDansButGx && estDansButY && estDansButZ;
  estDansButD = estDansButDx && estDansButY && estDansButZ;
  estDansButs = estDansButG || estDansButD;
end

function estHorsDuTerrain = estContactsAvecDehors(rf)
  estHorsDeX = (rf(1) < 0 || rf(1) > 120);
  estHorsDeY = (rf(2) < 0 || rf(2) > 90);
  estHorsDuTerrain = estHorsDeX || estHorsDeY;
end

function estAuSol = estContactAvecSol(rf)
  r_ballon = 0.11;          % m
  z_0_11_plane = r_ballon;
  estAuSol = rf(3) <= z_0_11_plane;
end

% retourne le nombre de Reynolds (nombre)
function Re = calculateRe(v)
  p_air = 1.2754;           % kg/m^3
  r_ballon = 0.11;          % m
  mu_air = 1.8*10^-5;       % kg/m*s
  Re = p_air*norm(v)*r_ballon/mu_air;
end

% retourne le coefficient de visq (nombre)
function C_vis = calculateCvis(v)
  Re = calculateRe(v);
  if (Re < 100000)
    C_vis = 0.235*norm(v);
  elseif (Re > 100000 & Re < 135000)
    C_vis = 0.235*norm(v) -0.125*norm(v)*((Re-100000)/35000);
  else
    C_vis = 0.110*norm(v);
  endif
end

% retourne la force visq (vecteur)
function F_vis = calculateFvis(v)
  C_vis = calculateCvis(v);
  r_ballon = 0.11;          % m
  A_ballon = pi*r_ballon^2; % m^2
  p_air = 1.2754;           % kg/m^3
  F_vis = -A_ballon*p_air*C_vis*v;
end

% retourne la constante de Magnus (nombre)
function C_M = calculateCM(v,w)
  r_ballon = 0.11;          % m
  C_M = 0.1925*((norm(w)*r_ballon)/(norm(v)))^(1/4);
end

% retourne la force de Magnus (vecteur)
function F_M = calculateFM(v,w)
  C_M = calculateCM(v,w);
  p_air = 1.2754;           % kg/m^3
  r_ballon = 0.11;          % m
  A_ballon = pi*r_ballon^2; % m^2
  F_M = p_air * C_M * A_ballon * (norm(v)^2) * (cross(w,v)/norm(cross(w,v)));
end

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
  tmp = q0 + [k1*Deltat/2 0 0 0];
  k2=feval(g,tmp,t0+Deltat/2);
  tmp = q0 + [k2*Deltat/2 0 0 0];
  k3=feval(g,tmp,t0+Deltat/2);
  tmp = q0 + [k3*Deltat/2 0 0 0];
  k4=feval(g,tmp,t0+Deltat);
  
  qs=(k1+2*k2+2*k3+k4)/6;
end

% permet de calculer l'acceleration lineaire courante
% en dependance des vitesses lineaire et angulaire
function res=calculAccel(q0,t0)
  v_courante = [q0(1) q0(2) q0(3)];
  w_courante = [q0(4) q0(5) q0(6)];     % toujours constante
  m_ballon = 0.45;          % kg
  g = 9.8;                  % m/s^2
  F_g = [0 0 -m_ballon*g];

  % Sommes des forces
  F_total = F_g + calculateFvis(v_courante) + calculateFM(v_courante, w_courante);

  % Acceleration courante
  accel = F_total / m_ballon;

  res = accel;
end

function vitesse=calculVitesse(q0,acc,Delta)
  v_prec = [q0(1) q0(2) q0(3)];
  v_courante = v_prec + acc * Delta;
  vitesse = v_courante;
end

function position=calculPosition(r_prec,vit,Delta)
  r_courante = r_prec + vit * Delta;
  position = r_courante;
end