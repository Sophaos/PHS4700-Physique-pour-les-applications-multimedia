function [pcm, MI, aa] = Devoir1(posA,ar,va,Forces)


%% Ailes
% parallelipipedes
L_aile = 10.6;   % en m
l_aile = 1.14;   % en m
e_aile = 0.25;   % en m
m_aile = 3.25*1000;   % en kg

%cm
x_aile_g = 10.54;
y_aile_g = L_aile / 2;
z_aile_g = e_aile / 2;
aile_g_cm = [x_aile_g y_aile_g z_aile_g]
x_aile_d = 10.54;
y_aile_d = -L_aile / 2;
z_aile_d = e_aile / 2;
aile_d_cm = [x_aile_d y_aile_d z_aile_d]

%mi
mi_aile_g = mi_p_aile(m_aile, L_aile, l_aile, e_aile)
mi_aile_d = mi_aile_g

%% Fuselage
% cylindre plein horizontal
h_fuselage = 22.95;      % en m
r_fuselage = 1.345;   % en m
m_fuselage = 15.1*1000;       % en kg

%cm
x_fuselage = h_fuselage / 2;
y_fuselage = 0;
z_fuselage = e_aile + r_fuselage;
fuselage_cm = [x_fuselage y_fuselage z_fuselage]

%mi
mi_fuselage = mi_cy(m_fuselage, r_fuselage, h_fuselage)

%% Cabine de pilotage
% cone plein horizontal
h_cabine = 3.82;   % en m
r_cabine = r_fuselage; % en m
m_cabine = 0.7*1000;    % en kg

%cm
x_cabine = h_fuselage + 0.25 * h_cabine;
y_cabine = 0;
z_cabine = e_aile + r_cabine;
cabine_cm = [x_cabine y_cabine z_cabine]

%mi
mi_cabine = mi_co(m_cabine, r_cabine, h_cabine)

%% Aileron
%paralllipipedes
h_aileron = 2.1;    % en m
l_aileron = 1.28;   % en m
e_aileron = 0.07;   % en m
m_aileron = 0.5*1000;    % en kg

%cm
x_aileron = l_aileron / 2;
y_aileron = 0;
z_aileron = 2*r_fuselage + e_aile + h_aileron / 2;
aileron_cm = [x_aileron y_aileron z_aileron]

% mi incertain
mi_aileron = mi_p_aileron(m_aileron, h_aileron, l_aileron, e_aileron)

%% Moteurs
% cylindres plein horizontaux
L_moteur = 3.68;   % en m
r_moteur = 0.724;  % en m
m_moteur = 1.7*1000;    % en kg

%cm
x_moteur_g = 5;
y_moteur_g = r_fuselage + r_moteur;
z_moteur_g = r_fuselage + e_aile;
moteur_g_cm = [x_moteur_g y_moteur_g z_moteur_g]

x_moteur_d = 5;
y_moteur_d = -(r_fuselage + r_moteur);
z_moteur_d = r_fuselage + e_aile;
moteur_d_cm = [x_moteur_d y_moteur_d z_moteur_d]

%mi
mi_moteur_g = mi_cy(m_moteur, r_moteur, L_moteur)
mi_moteur_d = mi_moteur_g

%% cm avion
s_m_x = m_aile*(x_aile_g + x_aile_d) + m_fuselage*x_fuselage + m_cabine*x_cabine + m_aileron*x_aileron + m_moteur*(x_moteur_d + x_moteur_g)
s_m_y = m_aile*(y_aile_g + y_aile_d) + m_fuselage*y_fuselage + m_cabine*y_cabine + m_aileron*y_aileron + m_moteur*(y_moteur_d + y_moteur_g)
s_m_z = m_aile*(z_aile_g + z_aile_d) + m_fuselage*z_fuselage + m_cabine*z_cabine + m_aileron*z_aileron + m_moteur*(z_moteur_d + z_moteur_g)
m_totale = (2 * m_aile) + m_fuselage + m_cabine + m_aileron + (2 * m_moteur)
x_bar = s_m_x / m_totale;
y_bar = s_m_y / m_totale;
z_bar = s_m_z / m_totale;

cmi = [x_bar; y_bar; z_bar]

%% position du bec initial
bec_x = h_fuselage + h_cabine;
bec_y = 0;
bec_z = r_cabine + e_aile;
bec = [bec_x; bec_y; bec_z]

%% TROUVER LE PCM
% vecteur cmi vers bec
v_cmi_bec = bec - cmi;
% cmi vers bec recoit une rotation en premier
% le cmi ne recoit pas de rotation
% le bec en recoit une
v_cmi_bec_rotay = rotay(ar)*v_cmi_bec;
n_bec = cmi + v_cmi_bec_rotay;
% n_bec vers posA
v_bec_posA = posA - n_bec;
% translation du cmi par le v_bec_posA
pcm = cmi + v_bec_posA;
% FIN PCM

mi_r_aile_g = rotay(ar)*mi_aile_g*rotay(ar).';
mi_r_aile_d = rotay(ar)*mi_aile_d*rotay(ar).';
mi_r_fuselage = rotay(ar)*mi_fuselage*rotay(ar).';
mi_r_cabine = rotay(ar)*mi_cabine*rotay(ar).';
mi_r_aileron = rotay(ar)*mi_aileron*rotay(ar).';
mi_r_moteur_g = rotay(ar)*mi_moteur_g*rotay(ar).';
mi_r_moteur_d = rotay(ar)*mi_moteur_d*rotay(ar).';

mi_adj_aile_g1 = mi_adj(m_aile, mi_r_aile_g, cmi, aile_g_cm);
mi_adj_aile_d1 = mi_adj(m_aile, mi_r_aile_d, cmi, aile_d_cm);
mi_adj_fuselage1 = mi_adj(m_fuselage, mi_r_fuselage, cmi, fuselage_cm);
mi_adj_cabine1 = mi_adj(m_cabine, mi_r_cabine, cmi, cabine_cm);
mi_adj_aileron1 = mi_adj(m_aileron, mi_r_aileron, cmi, aileron_cm);
mi_adj_moteur_g1 = mi_adj(m_moteur, mi_r_moteur_g, cmi, moteur_g_cm);
mi_adj_moteur_d1 = mi_adj(m_moteur, mi_r_moteur_d, cmi, moteur_d_cm);

MI = mi_adj_aile_g1 + mi_adj_aile_d1 + mi_adj_fuselage1 + mi_adj_cabine1 + mi_adj_aileron1 + mi_adj_moteur_g1 + mi_adj_moteur_d1;

%% aa : Acceleration angulaire 

%% I^-1 
Iinv = inv(MI);
%% L 
L = calc_L(MI, va');
%% tau : faire la somme des moments de force s'appliquant sur le systeme 
a= v_bec_posA
  %% force_md : force exercee par le moteur de droite
  %% Dans la direction de l'axe du moteur et exercee a l'arriere du moteur en son centre
   vec_position_force_md = [5 - (L_moteur/2), (r_moteur + r_fuselage), r_fuselage + e_aile] - cmi.'; 
   vec_cf_force_md = mtimes(vec_position_force_md, rotay(ar)) + (v_bec_posA.') - pcm'
   tau_force_md = calc_tau(vec_cf_force_md, [Forces(1)*cos(ar) 0 Forces(1)*sin(ar)])

  %% force_mg : force exercee par le moteur de gauche
  %% Dans la direction de l'axe du moteur et exercee a l'arriere du moteur en son centre
  vec_position_force_mg = [5 - (L_moteur/2), -(r_moteur + r_fuselage), (r_fuselage + e_aile)] - cmi';
  vec_cf_force_mg = mtimes(vec_position_force_mg, rotay(ar)) + (v_bec_posA') - pcm';
  tau_force_mg = calc_tau(vec_cf_force_mg, [Forces(2)*cos(ar) 0 Forces(2)*sin(ar)]); 
  
  %% force_p : force de portee sous l'aile (xa,ya,0)
  pos_force_p = [x_aile_g 0 0];
  vec_force_p_cmi = pos_force_p - cmi';
  vec_cf_force_p = mtimes(vec_force_p_cmi, rotay(ar));
  
  pos_force_p_ajuste = pcm + vec_cf_force_p;
  #%#&#&#pos_force_p_ajuste = pos_force_p + vec_force_p_cmi_ajuste - pcm';
  
  tau_force_mf = calc_tau(vec_cf_force_p, [0 0 Forces(3)]);
  
  tauTotal = tau_force_md + tau_force_mg + tau_force_mf;
  aa = (mtimes(Iinv, ( tauTotal + (cross(L, va')))'));
  
end

%% Liste de fonctions

%MIs (ajuster)

function mi_cone = mi_co(m, r, h)
mi_cone = [m*(3*r^2)/10 0 0; 0 m*(12*r^2+3*h^2)/80 0; 0 0 m*(12*r^2+3*h^2)/80]
end

% mi_fuselage = mi_cy(m_fuselage, r_fuselage, h_fuselage)
function mi_cylindre = mi_cy(m, r, h)
mi_cylindre = [(m*r^2)/2 0 0; 0 m*(r^2)/4+(h^2)/12 0; 0 0 m*(r^2)/4+(h^2)/12]
end

% para pour ailes
% mi_aile_g = mi_p_aile(m_aile, L_aile, l_aile, e_aile)
function mi_parallelepipede_aile = mi_p_aile(m, L, l, h) % le h correspond au e 
mi_parallelepipede_aile = [m*(L^2+h^2)/12 0 0; 0 m*(l^2+h^2)/12 0; 0 0 m*(L^2+l^2)/12]
end

% para pour aileron
% mi_aileron = mi_p_aileron(m_aileron, h_aileron, l_aileron, e_aileron)
function mi_parallelepipede_aileron = mi_p_aileron(m, L, l, h) % le h correspond au e 
mi_parallelepipede_aileron = [m*(L^2+h^2)/12 0 0; 0 m*(l^2+L^2)/12 0; 0 0 m*(h^2+l^2)/12]
end

% matrice mi par rapport a un pt arbitraire (d)
function matrice_pcm_cmi = mat_p_c(pt, cm)
v_cm = pt.' - cm;
matrice_pcm_cmi = [v_cm(2)^2+v_cm(3)^2 -v_cm(1)*v_cm(2) -v_cm(1)*v_cm(3); -v_cm(2)*v_cm(1) v_cm(1)^2+v_cm(3)^2 -v_cm(2)*v_cm(3); -v_cm(3)*v_cm(1) -v_cm(3)*v_cm(2) v_cm(1)^2+v_cm(2)^2];
end

% le mi ajuste par rapport a pt arbitraire (I_d = I_c + md)
function mi_adjusted = mi_adj(m, mi, pt, cm)
mi_adjusted = mi + m*mat_p_c(pt, cm);
end

function m_rotay = rotay(ar)
  m_rotay = [cos(ar) 0 sin(ar); 0 1 0; -sin(ar) 0 cos(ar)];
end

% OCTAVE
% POUR TESTER
% [pcm, MI, aa] = Devoir1o([26.77;0;1.5950],(pi/4),1,1)

% Fonction permettant de calculer le moment cinetique L dans le but de 
% calculer l'accceleration angulaire
% - I : Inertie
% - w : vitesse angulaire (donnee du probleme)
 function calculer_L = calc_L(L, w)
 Lxx = L(1,1);
 Lxy = L(1,2);
 Lxz = L(1,3);
 Lyx = L(2,1);
 Lyy = L(2,2);
 Lyz = L(2,3);
 Lzx = L(3,1);
 Lzy = L(3,2);
 Lzz = L(3,3);
 wx  = w(1); 
 wy  = w(2);
 wz  = w(3); 
 Li_x = (Lxx * wx) + (Lxy * wy) + (Lxz * wz);
 Li_y = (Lyx * wx) + (Lyy * wy) + (Lyz * wz);
 Li_z = (Lzx * wx) + (Lzy * wy) + (Lzz * wz); 
 vec_L = [Li_x Li_y Li_z];
 calculer_L = vec_L;
 end

% Calcule du moment de force (tau) 
% - r : point ou la force est appliquée par rapport au cm 
% - f : force (N) 
 function calculer_tau = calc_tau(r, f)
 calculer_tau = cross(r,f);
 end % a completer 

 function distance = dis(posA, pcm)
   distance = ((posA(1)-pcm(1))^2+(posA(2)-pcm(2))^2+(posA(3)-pcm(3))^2)^(1/2)
 end

