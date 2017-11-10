function acceleration = calculate_acceleration_IDM(objectcar, frontcar, pos_frontcar_est, idm, FLAG_Follow_or_Cross)
% calculate acceleration by IDM (from preceding and object car's information) 


A3_TTC = norm(pos_frontcar_est(1:2) - objectcar.pos(1:2)) - idm.l;
if A3_TTC < idm.s0
    A3_TTC = idm.s0;
end

if FLAG_Follow_or_Cross
    if idx_observedcar(j) == 0 % for mycar
        A2 = (idm.s0 + othercars.car{i}.vel(1)*idm.T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - (mycar.vel(1)*cos((mycar.pos(3)-othercars.car{i}.pos(3))*pi/180)))/2/sqrt(idm.a*idm.b))/A3_TTC;
    else % for othercar
        A2 = (idm.s0 + othercars.car{i}.vel(1)*idm.T + othercars.car{i}.vel(1) * (othercars.car{i}.vel(1) - (othercars.car{idx_observedcar(j)}.vel(1)*cos((othercars.car{idx_observedcar(j)}.pos(3)-othercars.car{i}.pos(3))*pi/180)))/2/sqrt(idm.a*idm.b))/A3_TTC;
    end
else
    A2 = (idm.s0 + othercars.car{i}.vel(1)*idm.T + othercars.car{i}.vel(1) * othercars.car{i}.vel(1)/2/sqrt(idm.a*idm.b))/A3_TTC;
end

A1 = othercars.car{i}.vel(1)/idm.v0;
accele_IDM = idm.a*(1 - A1^idm.delta - A2^2);

% -----ACC model-----------
aLead = 0;  % this value need to be modified !!
aLeadRestricted = min(aLead,idm.a);

if FLAG_Follow_or_Cross
    if idx_observedcar(j) == 0 % for mycar
        dvp = max(othercars.car{i}.vel(1) - mycar.vel(1)*cos((mycar.pos(3)-othercars.car{i}.pos(3))*pi/180),0);
        vLead = mycar.vel(1)*cos((mycar.pos(3)-othercars.car{i}.pos(3))*pi/180);
    else % for othercar
        dvp = max(othercars.car{i}.vel(1) - othercars.car{idx_observedcar(j)}.vel(1)*cos((othercars.car{idx_observedcar(j)}.pos(3)-othercars.car{i}.pos(3))*pi/180),0);
        vLead = othercars.car{idx_observedcar(j)}.vel(1);
    end
else
    dvp = max(othercars.car{i}.vel(1),0);
    vLead = 0;
end

denomCAH = vLead*vLead - 2*A3_TTC*aLeadRestricted;

if (vLead*dvp < -2*A3_TTC*aLeadRestricted)&&(denomCAH~=0)
    accele_CAH = othercars.car{i}.vel(1)*othercars.car{i}.vel(1)*aLeadRestricted/denomCAH;
else
    accele_CAH = aLeadRestricted - 0.5*dvp*dvp/max(A3_TTC,0.1);
end

if accele_IDM > accele_CAH
    cur_acceleration = accele_IDM;
else
    cur_acceleration = (1-idm.coolness)*accele_IDM + idm.coolness*( accele_CAH + idm.b*tanh((accele_IDM - accele_CAH)/idm.b));
end


end

