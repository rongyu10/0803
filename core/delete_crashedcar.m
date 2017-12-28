function othercars = delete_crashedcar(mycar,othercars)


idx_nearestCar = get_nearestCar(mycar,othercars);
othercars.car{idx_nearestCar}.pos(1) = 1000*10^3;

end

function idx_nearestCar = get_nearestCar(mycar,othercars)


mycar_pos = mycar.pos(1:2);
nr_cars = othercars.n;

min_DISTANCE = 100*10^3;
for i=1:nr_cars
    pos = othercars.car{i}.pos(1:2);
    diff= pos - mycar_pos;
    if norm(diff) < min_DISTANCE
      min_DISTANCE = norm(diff);
      idx_nearestCar = i;
    end
end

end
