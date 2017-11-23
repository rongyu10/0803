function acceleration = calculate_acceleration_IDM_Follow_or_Closs(objectcar, frontcar, pos_frontcar_est, idm, FLAG_Follow_or_Cross)
% calculate acceleration by IDM (from preceding and object car's information) 


A3_TTC = norm(pos_frontcar_est(1:2) - objectcar.pos(1:2)) - idm.l;
if A3_TTC < idm.s0
    A3_TTC = idm.s0;
end

if FLAG_Follow_or_Cross
    A2 = (idm.s0 + objectcar.vel(1)*idm.T + objectcar.vel(1) * (objectcar.vel(1) - (frontcar.vel(1)*cos((frontcar.pos(3)-objectcar.pos(3))*pi/180)))/2/sqrt(idm.a*idm.b))/A3_TTC;
else
    A2 = (idm.s0 + objectcar.vel(1)*idm.T + objectcar.vel(1) * objectcar.vel(1)/2/sqrt(idm.a*idm.b))/A3_TTC;
end

A1 = objectcar.vel(1)/idm.v0;
accele_IDM = idm.a*(1 - A1^idm.delta - A2^2);

% -----ACC model-----------
aLead = 0;  % this value need to be modified !!
aLeadRestricted = min(aLead,idm.a);

if FLAG_Follow_or_Cross
        dvp = max(objectcar.vel(1) - frontcar.vel(1)*cos((frontcar.pos(3)-objectcar.pos(3))*pi/180),0);
        vLead = frontcar.vel(1)*cos((frontcar.pos(3)-objectcar.pos(3))*pi/180);
    
else
    dvp = max(objectcar.vel(1),0);
    vLead = 0;
end

denomCAH = vLead*vLead - 2*A3_TTC*aLeadRestricted;

if (vLead*dvp < -2*A3_TTC*aLeadRestricted)&&(denomCAH~=0)
    accele_CAH = objectcar.vel(1)*objectcar.vel(1)*aLeadRestricted/denomCAH;
else
    accele_CAH = aLeadRestricted - 0.5*dvp*dvp/max(A3_TTC,0.1);
end

if accele_IDM > accele_CAH
    acceleration = accele_IDM;
else
    acceleration = (1-idm.coolness)*accele_IDM + idm.coolness*( accele_CAH + idm.b*tanh((accele_IDM - accele_CAH)/idm.b));
end

% if acceleration < -2940
%     fprintf(2, 'acceleration = [%4d]', acceleration);
% end
if objectcar.pos(1) > 146000
    fprintf(2, 'acceleration = [%4d]', acceleration);
end

end

