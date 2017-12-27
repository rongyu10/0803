bd1 = mycar.bd;
L    = 1000;
L2   = 0;

BDmycar = [bd1(1,1),bd1(2,1),bd1(4,1),bd1(5,1),bd1(7,1),bd1(8,1);... 
           bd1(1,2),bd1(2,2),bd1(4,2),bd1(5,2),bd1(7,2),bd1(8,2)];

BD2 = [bd1(1,1)+L,bd1(2,1)+L,bd1(4,1)+L,bd1(5,1)+L,bd1(7,1)+L,bd1(8,1)+L;... 
           bd1(1,2)+L2,bd1(2,2)+L2,bd1(4,2)+L2,bd1(5,2)+L2,bd1(7,2)+L2,bd1(8,2)+L2];

hold on; 
h1 = patch(BDmycar(1,:), BDmycar(2,:), 'c');
h2 = patch(BD2(1,:), BD2(2,:), 'b');

P = InterX(BDmycar,BD2);
