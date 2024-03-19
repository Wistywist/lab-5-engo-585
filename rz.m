function rotation = rz(theta)
    rotation = [[ cosd(theta),-sind(theta),0;sind(theta),cosd(theta),0;0,0,1]];

end