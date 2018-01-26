Plant = tf([1 1],[1 2 0]);
Ts = 0.1;
MV = struct('Min',-1,'Max',1);
p = 20;
m = 3;
MPCobj = mpc(Plant,Ts,p,m,[],MV);