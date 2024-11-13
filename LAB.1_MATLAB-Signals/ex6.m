function [] = ex6()

    x = zeros(200,200,3);
    x(:, :, 2) = 255;
    imshow(x);
    pause(10);
    
    x(:, :, 2) = 0;    
    x(:, :, 3) = 255;  
    imshow(x);
    pause(10);

    x(:, :, 3) = 0;
    x(:, :, 1) = 255;
    imshow(x);
    pause(10);

    x(:, :, 2) = 255;  
    imshow(x);

end