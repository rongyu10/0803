function  [accele_ACC,angry] = predictAccel_by_ACC(v_free, mycar, followingcar, fromEgocar)

%---- Settings for intelligent driver model --------------
v0    = v_free;   % desired_velocity[mm/s]
%T     = 1.5;      % 1.5 safetime_headway[s]
%a_max = 1000;     % 1.0 maximum acceleration [mm/s^2]
%b_max = 3000;     % 3.0 confotable decceletion  [mm/s^2]
%[car_size, ~] = get_carsize(); % length of car [mm]
%car_size = 5;   %   5.0 length of car [m]
%delta = 4.0;      % 4.0 acceleration exponent [-]
%s0    = 2000;      % 2.0 linear jam distance [mm]
%s1    = 3000;    % 3000  non-linear jam distance [mm]
c = 0.99;          % coolness facotor
%--------------------------------------------------------

if fromEgocar
   T     = 1.5;      % 1.5 safetime_headway[s]
   a_max = 1000;     % 1.0 maximum acceleration [mm/s^2]
   b_max = 3000;     % 3.0 confotable decceletion  [mm/s^2]
   car_size = 5000;  %   5.0 length of car [m]
   delta = 4.0;      % 4.0 acceleration exponent [-]
   s0    = 2000;     % 2.0 linear jam distance [mm]
   %s1    = 3000;    % 3000  non-linear jam distance [mm]    
else
   T        = followingcar.IDM.T;       % 1.5 safetime_headway[s]
   a_max    = followingcar.IDM.a_max;   % 1.0 maximum acceleration [mm/s^2]
   b_max    = followingcar.IDM.b_max;   % 3.0 confotable decceletion  [mm/s^2]
   car_size = followingcar.W;           % length of car [mm]
   delta    = followingcar.IDM.delta;   % 4.0 acceleration exponent [-]
   s0       = followingcar.IDM.s0;      % 2.0 linear jam distance [mm]
   %s1    = 3000;    % 3000  non-linear jam distance [mm]    
end

v   = followingcar.vel(1);
x   = followingcar.pos(1);

% -----IDM model-------------------------------
if ~isempty(mycar)
  v_1 = mycar.vel(1);
  x_1 = mycar.pos(1);

  deltaV  = v - v_1;
  s_alpha = x_1 - x - car_size;

  s_star       = s0 + max(0.0, v*T + (v*deltaV)/(2.0*sqrt(a_max*b_max)));
  relativeGap  = s_star/s_alpha;

else
   relativeGap = 0; % no leading car
end

relativeVelocity = v/v0;
accele_IDM      = a_max*(1.0 - relativeVelocity.^delta - relativeGap.^2);
%---------------------------------------------

if ~isempty(mycar)
   % -----ACC model------------------------------------------
   aLead = 0;  % this value need to be modified !!
   aLeadRestricted = min(aLead,a_max);
   dvp = max(deltaV,0);
   vLead = v_1;

   denomCAH = vLead*vLead - 2*s_alpha*aLeadRestricted;

   if (vLead*dvp < -2*s_alpha*aLeadRestricted)&&(denomCAH~=0)
       accele_CAH = v*v*aLeadRestricted/denomCAH;
   else
       accele_CAH = aLeadRestricted - 0.5*dvp*dvp/max(s_alpha,0.1);
   end

   if accele_IDM > accele_CAH
       accele_ACC = accele_IDM;
   else
       accele_ACC = (1-c)*accele_IDM + c*( accele_CAH + b_max*tanh((accele_IDM - accele_CAH)/b_max));
   end

   if accele_ACC < -9000
      accele_ACC = -9000;
   end
   %-------------------------------------------------------
else
    accele_ACC = accele_IDM; % no leading car
end

if (accele_ACC < -3000)&&(accele_ACC > -7000)
    angry = 1;
elseif (accele_ACC <= -7000)
    angry = 2;
else
    angry = 0;            
end

return
end