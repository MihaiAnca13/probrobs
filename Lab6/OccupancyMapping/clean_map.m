for i=1:length(map)
    for j=1:length(map)  % its square right?
        if map(i,j)>0.5
            map(i,j) = 1;
        elseif map(i,j)<0.5
            map(i,j) = 0;
        end
    end
end

figure(2)
image([1 dimx], [1 dimy], repmat((1-map)', [1 1 3]));
    set(gca, 'ydir', 'normal')
    xlabel('x-axis');
    ylabel('y-axis');