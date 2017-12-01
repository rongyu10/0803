function [squareX, squareY] = make_detecting_rectangle_lengthfix(car, position,laneChangePath, length, width)

% make rectangle of detecting area(4points)-----------------------------------
squareX = zeros(1,5);
squareY = zeros(1,5);

for i = 0:1
    X_est = position(1) + length*i;
    
    if X_est <= 100*10^3
        squareX(i+1) = X_est;
        squareX(4-i) = X_est;
        squareY(i+1) = car.pos(2) - width;
        squareY(4-i) = car.pos(2) + width;
        
        
    elseif X_est <= 275*10^3
        
        if i == 0
            mycar_posEst_det = car.pos;
        else
            for idx = 1:201
                if X_est - 100*10^3 - 175/200*(idx - 1)*10^3 < 0
                    % if squareX(i+1) - laneChangePath{car.goallane, car.save.lane_idx}(idx+1,1) < 0
                    break;
                end
            end
            
            if idx~=201
                vx= laneChangePath{car.goallane, car.save.lane_idx}(idx+1,1)-laneChangePath{car.goallane, car.save.lane_idx}(idx,1);
                vy= laneChangePath{car.goallane, car.save.lane_idx}(idx+1,2)-laneChangePath{car.goallane, car.save.lane_idx}(idx,2);
            else
                vx= laneChangePath{car.goallane, car.save.lane_idx}(idx,1)-laneChangePath{car.goallane, car.save.lane_idx}(idx-1,1);
                vy= laneChangePath{car.goallane, car.save.lane_idx}(idx,2)-laneChangePath{car.goallane, car.save.lane_idx}(idx-1,2);
            end
            
            %calculate the center point of the red rectangle
            mycar_posEst_det(1) = laneChangePath{car.goallane, car.save.lane_idx}(idx,1);
            mycar_posEst_det(2) = laneChangePath{car.goallane, car.save.lane_idx}(idx,2);
            mycar_posEst_det(3) = atan(vy/vx)*180/pi;
            
        end
        %calculate point of the red rectangle from the center point
        left_right_point = get_car_futurepoint(mycar_posEst_det, car.W, width*2);
        squareX(i+1) = left_right_point(1,1);
        squareY(i+1) = left_right_point(1,2);
        squareX(4-i) = left_right_point(2,1);
        squareY(4-i) = left_right_point(2,2);
        
    else
        squareX(i+1) = X_est;
        squareX(4-i) = X_est;
        squareY(i+1) = (77.5-car.goallane*5.0)*10^3 - width;
        squareY(4-i) = (77.5-car.goallane*5.0)*10^3 + width;
        
    end
    
end
squareX(5) = squareX(1);
squareY(5) = squareY(1);
%(end) make rectangle of detecting area(4points)----------------------------------


% make rectangle of detecting area(12points)-----------------------------------
% squareX = zeros(1,13);
% squareY = zeros(1,13);
% 
% for i = 0:5
%     X_est = position(1) + velocity(1)*(rect_length_time/5)*i;
%     
%     if X_est <= 100*10^3
%         squareX(i+1) = X_est;
%         squareX(12-i) = X_est;
%         squareY(i+1) = car.pos(2) - rect_width_side;
%         squareY(12-i) = car.pos(2) + rect_width_side;
%         
%         
%     elseif X_est <= 275*10^3
%         
%         if i == 0
%             mycar_posEst_det = car.pos;
%         else
%             for idx = 1:201
%                 if X_est - 100*10^3 - 175/200*(idx - 1)*10^3 < 0
%                     % if squareX(i+1) - laneChangePath{car.goallane, car.save.lane_idx}(idx+1,1) < 0
%                     break;
%                 end
%             end
%             
%             if idx~=201
%                 vx= laneChangePath{car.goallane, car.save.lane_idx}(idx+1,1)-laneChangePath{car.goallane, car.save.lane_idx}(idx,1);
%                 vy= laneChangePath{car.goallane, car.save.lane_idx}(idx+1,2)-laneChangePath{car.goallane, car.save.lane_idx}(idx,2);
%             else
%                 vx= laneChangePath{car.goallane, car.save.lane_idx}(idx,1)-laneChangePath{car.goallane, car.save.lane_idx}(idx-1,1);
%                 vy= laneChangePath{car.goallane, car.save.lane_idx}(idx,2)-laneChangePath{car.goallane, car.save.lane_idx}(idx-1,2);
%             end
%             
%             %calculate the center point of the red rectangle
%             mycar_posEst_det(1) = laneChangePath{car.goallane, car.save.lane_idx}(idx,1);
%             mycar_posEst_det(2) = laneChangePath{car.goallane, car.save.lane_idx}(idx,2);
%             mycar_posEst_det(3) = atan(vy/vx)*180/pi;
%             
%             
%         end
%         %calculate point of the red rectangle from the center point
%         left_right_point = get_car_futurepoint(mycar_posEst_det, car.W, 5000);
%         squareX(i+1) = left_right_point(1,1);
%         squareY(i+1) = left_right_point(1,2);
%         squareX(12-i) = left_right_point(2,1);
%         squareY(12-i) = left_right_point(2,2);
%         
%     else
%         squareX(i+1) = X_est;
%         squareX(12-i) = X_est;
%         squareY(i+1) = (77.5-car.goallane*5.0)*10^3 - rect_width_side;
%         squareY(12-i) = (77.5-car.goallane*5.0)*10^3 + rect_width_side;
%         
%     end
%     
% end
% squareX(13) = squareX(1);
% squareY(13) = squareY(1);
%(end) make rectangle of detecting area(12points)----------------------------------

end

