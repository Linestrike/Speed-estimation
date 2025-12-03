function Reto() %#codegen
    r = raspi('10.235.18.39','nogada','123456');
    cam = webcam(r, 1, "320x240");
    
    numFrames = 50;
    positions = nan(numFrames, 2); 
    scaleFactor = 0.046;
    seClose = strel('disk', 22); 
    y_inicio = 180;
    y_fin = 220;
    
    fprintf('Capturando imagen de referencia \n');
    for i = 1:5
        referenceFrame = snapshot(cam);
        pause(0.05);
    end
    referenceFrameROI = referenceFrame(y_inicio:y_fin-1, :, :);
    referenceGray = rgb2gray(referenceFrameROI);
    referenceGray = imgaussfilt(referenceGray, 2.2);


    fprintf('Esperando movimiento para iniciar\n');
    
    while true
        frame = snapshot(cam);
        frameROI = frame(y_inicio:y_fin-1, :, :);
        grayFrame = rgb2gray(frameROI);
        grayFrame = imgaussfilt(grayFrame, 2.2);
        diffFrame = imabsdiff(grayFrame, referenceGray);
        bwFrame = imbinarize(diffFrame, 0.22);
        bwFrame = imclose(bwFrame, seClose); 
        bwFrame = imfill(bwFrame, 'holes');
        
        stats = regionprops(bwFrame, 'Area');
        
        % Si detectamos algo mayor a 500px, rompemos el bucle
        if ~isempty(stats)
            if max([stats.Area]) > 500
                fprintf('Movimiento detectado\n');
                break;
            end
        end
        pause(0.01);
    end

time=tic;    
frameCount = 1;
noMovementCount = 0;
max_nomove = 10;

while frameCount <= numFrames
    frame = snapshot(cam);
    frameROI = frame(y_inicio:y_fin-1, :, :);
    grayFrame = rgb2gray(frameROI);
    grayFrame = imgaussfilt(grayFrame, 2.2);    
    diffFrame = imabsdiff(grayFrame, referenceGray);
    bwFrame = imbinarize(diffFrame, 0.22);
    bwFrame = imclose(bwFrame, seClose); 
    bwFrame = imfill(bwFrame, 'holes');
    stats = regionprops(bwFrame, 'BoundingBox', 'Area', 'Centroid');

    if ~isempty(stats)
        % Encuentra objeto
        [maxArea, idx] = max([stats.Area]);
        
        if maxArea > 500
            positions(frameCount, :) = stats(idx).Centroid;
            noMovementCount = 0;  % Reinicia contador
        else
            positions(frameCount, :) = [NaN, NaN];
            noMovementCount = noMovementCount + 1;
        end
    else
        positions(frameCount, :) = [NaN, NaN];
        noMovementCount = noMovementCount + 1;
    end

    if noMovementCount >= max_nomove
        fprintf("Se detuvo por falta de movimiento");
        break;
    end

    frameCount = frameCount + 1;

end

totalFramesCaptured = frameCount - 1;
totalTime = toc(time);
realFPS = totalFramesCaptured / totalTime;
fprintf('\nFPS durante captura %.2f\n', realFPS);
validPositions = positions(~any(isnan(positions), 2), :);

if size(validPositions, 1) < 5
    fprintf('Pocos datos validos\n');
else
    distances = sqrt(sum(diff(validPositions).^2, 2));
    cleanDistances = distances(distances > 2 & distances < 150);
    avg_disp = median(cleanDistances);
    speed_px_s = avg_disp * realFPS;
    speed_m_s = speed_px_s * scaleFactor;
    speed_kmh = speed_m_s * 3.6;

    fprintf('Velocidad %.2f km/h\n', speed_kmh);
    
end
