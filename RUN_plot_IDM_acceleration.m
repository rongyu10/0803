idm.v0 = 17500; % desired velocity
idm.T = 1.5; % Safe time headway
idm.a = 1400; % maximum acceleration
idm.b = 2000; %desired deceleration
idm.delta = 4; %acceleration exponent
idm.s0 = 2000; % minimum distance
idm.l = 4000; % vehicle length
idm.coolness = 0.99;

x_grid = 0:100:17500; % 自車速度
y_grid = 5000:1000:60000; % 車間距離

[X,Y] = meshgrid(x_grid,y_grid);

acceleration = -idm.a*((idm.s0+idm.T*X)./(Y-idm.l));


itvl= -4000:500:0;
contourf(X,Y,acceleration,'LevelList',itvl);
xlim([0, 17500]);
ylim([5000, 60000]);
caxis([-4000,0])
colorbar
hold on