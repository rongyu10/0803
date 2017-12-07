function acceleration = calculate_acceleration_IDM(objectcar, frontcar, pos_frontcar_est, idm, rel_degree)
% calculate acceleration by IDM (from preceding and object car's information) 


A3 = norm(pos_frontcar_est(1:2) - objectcar.pos(1:2)) - idm.l;
if A3 < idm.s0
    A3 = idm.s0;
end

% if rel_degree < 3
%     A2 = (idm.s0 + objectcar.vel(1)*idm.T + objectcar.vel(1) * (objectcar.vel(1) - frontcar.vel(1)*cos(rel_degree*pi/180))/2/sqrt(idm.a*idm.b))/A3;
% elseif rel_degree < 5
%     A2 = (idm.s0 + objectcar.vel(1)*idm.T + objectcar.vel(1) * (objectcar.vel(1) - ((5 - rel_degree)/2)*frontcar.vel(1)*cos(rel_degree*pi/180))/2/sqrt(idm.a*idm.b))/A3;
% else
%     A2 = (idm.s0 + objectcar.vel(1)*idm.T + objectcar.vel(1) * objectcar.vel(1)/2/sqrt(idm.a*idm.b))/A3;
% end

A2 = (idm.s0 + objectcar.vel(1)*idm.T + objectcar.vel(1) * (objectcar.vel(1) - frontcar.vel(1)*cos(rel_degree*pi/180))/2/sqrt(idm.a*idm.b))/A3;
A1 = objectcar.vel(1)/idm.v0;
accele_IDM = idm.a*(1 - A1^idm.delta - A2^2);

% -----ACC model-----------
aLead = 0;  % this value need to be modified !!
aLeadRestricted = min(aLead,idm.a);

if rel_degree < 3
    dvp = max(objectcar.vel(1) - frontcar.vel(1)*cos(rel_degree*pi/180),0);
    vLead = frontcar.vel(1)*cos(rel_degree*pi*pi/180);
elseif rel_degree < 5
    dvp = max(objectcar.vel(1) - ((5 - rel_degree)/2)*frontcar.vel(1)*cos(rel_degree*pi/180),0);
    vLead = frontcar.vel(1)*cos(rel_degree*pi*pi/180);
else
    dvp = max(objectcar.vel(1),0);
    vLead = 0;
end

denomCAH = vLead*vLead - 2*A3*aLeadRestricted;

if (vLead*dvp < -2*A3*aLeadRestricted)&&(denomCAH~=0)
    accele_CAH = objectcar.vel(1)*objectcar.vel(1)*aLeadRestricted/denomCAH;
else
    accele_CAH = aLeadRestricted - 0.5*dvp*dvp/max(A3,0.1);
end

if accele_IDM > accele_CAH
    acceleration = accele_IDM;
else
    acceleration = (1-idm.coolness)*accele_IDM + idm.coolness*( accele_CAH + idm.b*tanh((accele_IDM - accele_CAH)/idm.b));
end

end

