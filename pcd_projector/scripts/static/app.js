const ROBOTICA_AVANZADA = false;
const RVIZ = false;

const tolerance = 10;  // Adjust this value based on testing

function colorMatches(r, g, b, a) {
    let targetR = 255, targetG = 0, targetB = 0, targetA = 255;
    return (
        r >= targetR - tolerance && r <= targetR + tolerance &&
        g >= targetG - tolerance && g <= targetG + tolerance &&
        b >= targetB - tolerance && b <= targetB + tolerance &&
        a >= targetA - tolerance && a <= targetA + tolerance
    );
}

document.addEventListener('DOMContentLoaded', function () {
    const overviewCanvas = document.getElementById('overviewCanvas');
    const overviewCtx = overviewCanvas.getContext('2d');
    const mapCanvas = document.getElementById('mapCanvas');
    const mapCtx = mapCanvas.getContext('2d');
    const panButton = document.getElementById('panToggle');
    const panControls = document.getElementById('panControls');
    const confirmZoomButton = document.getElementById('confirmZoomButton');
    // const setZeroButton = document.getElementById('setZeroButton');
    const undoButton = document.getElementById('undoButton');
    const clearButton = document.getElementById('clearButton');

    let img = new Image();
    let boundingBox;
    let x_ratio, y_ratio;
    let robotImg = new Image();

    const isLocalhost = window.location.hostname === 'localhost';
    const isIP = /^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$/.test(window.location.hostname);
    // const protocol = (isLocalhost || isIP) ? 'http' : 'https';
    const protocol = (isLocalhost || isIP) ? 'http' : 'http';

    // const host = isLocalhost ? 'localhost:5000' : (isIP ? window.location.hostname + ':5000' : 'server-agv-pucmm-2024.loca.lt');
    // const host = isLocalhost ? 'localhost:80' : (isIP ? window.location.hostname + ':80' : 'server-agv-pucmm-2024.serveo.net');
    const host = isLocalhost ? 'localhost:80' : (isIP ? window.location.hostname + ':80' : 'agv-pucmm.com');

    const url = `${protocol}://${host}/data`;
    const urlim = `${protocol}://${host}/image`;
    const urthumb = `${protocol}://${host}/robotthumbail`;
    const urlRobotPosition = `${protocol}://${host}/robot_position`;

    if (ROBOTICA_AVANZADA){
        document.body.style.backgroundColor = "#0000AA";
    }

    const refreshRate = 200; // Refresh rate in milliseconds (5 Hz)

    fetch(url)
        .then(response => response.json())
        .then(data => {
            boundingBox = data.bounding_box;
            x_ratio = data.x_ratio;
            y_ratio = data.y_ratio;
            // console.log(`Bounding Box: ${JSON.stringify(boundingBox)}`);
            // console.log(`x_ratio: ${x_ratio}, y_ratio: ${y_ratio}`);
        })
        .catch(error => console.error('Error fetching data:', error));

        fetch('/get_processes')
           .then(response => response.json())
           .then(data => {
               processes = data;
               updateProcessList();
   });

    let canvasOffsetX = 0;
    let canvasOffsetY = 0;
    let scaleX = 1.0;
    let scaleY = 1.0;
    const scaleMultiplier = 1.1;
    let isPanning = false;
    let startCoords;
    let initialX = null, initialY = null;
    let zoomConfirmed = false;
    let isSettingZero = false;
    let isPlacingCircles = false;
    let crosshairX = null, crosshairY = null;
    let circles = [];
    let robotX = 0, robotY = 0, robotOrientation = 0;

    img.onload = function () {
        overviewCanvas.width = overviewCanvas.clientWidth;
        overviewCanvas.height = overviewCanvas.clientHeight;
        mapCanvas.width = mapCanvas.clientWidth;
        mapCanvas.height = mapCanvas.clientHeight;
        performInitialCoordinateDetection();  // Perform initial coordinate detection after image load
        draw();
    };

    robotImg.onload = function () {
        // If you need to do anything after loading the robot image
    };

    img.src = urlim;
    robotImg.src = urthumb;

    function performInitialCoordinateDetection() {
        const offScreenCanvas = document.createElement('canvas');
        const offScreenCtx = offScreenCanvas.getContext('2d');
        offScreenCanvas.width = img.width;
        offScreenCanvas.height = img.height;
        offScreenCtx.drawImage(img, 0, 0);
        const imageData = offScreenCtx.getImageData(0, 0, img.width, img.height);
        const data = imageData.data;

        let redPixels = [];  // Store the coordinates of red pixels

        for (let i = 0; i < data.length; i += 4) {
            const r = data[i], g = data[i + 1], b = data[i + 2], a = data[i + 3];

            if (colorMatches(r, g, b, a)) {
                const x = (i / 4) % img.width;
                const y = Math.floor((i / 4) / img.width);
                redPixels.push({ x, y });
            }
        }

        if (redPixels.length > 0) {
            let sumX = 0, sumY = 0;
            redPixels.forEach(pixel => {
                sumX += pixel.x;
                sumY += pixel.y;
            });
            initialX = sumX / redPixels.length;
            initialY = sumY / redPixels.length;
            console.log(`Initial red square position: x=${initialX}, y=${initialY}`);
        } else {
            console.error('Red square not found in the image.');
        }

        draw();
    }

    function draw() {
        mapCtx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);  // Clear the canvas
        mapCtx.save();
        mapCtx.translate(mapCanvas.width / 2 + canvasOffsetX, mapCanvas.height / 2 + canvasOffsetY);
        mapCtx.scale(scaleX, scaleY);
        mapCtx.translate(-img.width / 2, -img.height / 2);
        mapCtx.drawImage(img, 0, 0);  // Draw the image
        mapCtx.restore();

        // Draw the overview image with proper aspect ratio
        overviewCtx.clearRect(0, 0, overviewCanvas.width, overviewCanvas.height);  // Clear the canvas
        const overviewScale = Math.min(overviewCanvas.width / img.width, overviewCanvas.height / img.height);
        const overviewWidth = img.width * overviewScale;
        const overviewHeight = img.height * overviewScale;
        const overviewX = (overviewCanvas.width - overviewWidth) / 2;
        const overviewY = (overviewCanvas.height - overviewHeight) / 2;
        overviewCtx.drawImage(img, overviewX, overviewY, overviewWidth, overviewHeight);

        // Draw the red rectangle on the overview canvas representing the current view on the map canvas
        if (zoomConfirmed) {
          updateOverviewRectangle();
        }

        // Draw a green circle with a black outline around the (0,0) point
        if (initialX !== null && initialY !== null) {
            mapCtx.save();
            mapCtx.translate(mapCanvas.width / 2 + canvasOffsetX, mapCanvas.height / 2 + canvasOffsetY);
            mapCtx.scale(scaleX, scaleY);
            mapCtx.translate(-img.width / 2, -img.height / 2);
            mapCtx.beginPath();
            mapCtx.arc(initialX, initialY, 5 / Math.max(scaleX, scaleY), 0, 2 * Math.PI); // Adjusted for independent scaling
            mapCtx.fillStyle = 'green';
            mapCtx.fill();
            mapCtx.strokeStyle = 'black';
            mapCtx.lineWidth = 1 / Math.max(scaleX, scaleY); // Adjusted for independent scaling
            mapCtx.stroke();
            mapCtx.restore();
        }

        // Draw the robot's position and orientation
        if (zoomConfirmed) {
            mapCtx.save();
            mapCtx.translate(mapCanvas.width / 2 + canvasOffsetX, mapCanvas.height / 2 + canvasOffsetY);
            mapCtx.scale(scaleX, scaleY);
            mapCtx.translate(-img.width / 2, -img.height / 2);
            drawRobot();
            mapCtx.restore();
            drawRobotOverview();
        }

        // Draw the lines first
        if (circles.length > 0) {
            mapCtx.save();
            mapCtx.translate(mapCanvas.width / 2 + canvasOffsetX, mapCanvas.height / 2 + canvasOffsetY);
            mapCtx.scale(scaleX, scaleY);
            mapCtx.translate(-img.width / 2, -img.height / 2);
            mapCtx.strokeStyle = 'orange';
            mapCtx.lineWidth = 1 / Math.max(scaleX, scaleY); // Adjusted for independent scaling
            for (let i = 0; i < circles.length; i++) {
                const circle = circles[i];
                if (i > 0) {
                    const prevCircle = circles[i - 1];
                    mapCtx.beginPath();
                    mapCtx.moveTo(prevCircle.x, prevCircle.y);
                    mapCtx.lineTo(circle.x, circle.y);
                    mapCtx.stroke();
                }
            }
            mapCtx.restore();
        }

        // Draw the circles on top of the lines
        if (circles.length > 0) {
            mapCtx.save();
            mapCtx.translate(mapCanvas.width / 2 + canvasOffsetX, mapCanvas.height / 2 + canvasOffsetY);
            mapCtx.scale(scaleX, scaleY);
            mapCtx.translate(-img.width / 2, -img.height / 2);
            mapCtx.strokeStyle = 'black';
            mapCtx.fillStyle = 'orange';
            mapCtx.lineWidth = 1 / Math.max(scaleX, scaleY); // Adjusted for independent scaling
            for (let i = 0; i < circles.length; i++) {
                const circle = circles[i];
                if(circle.r == 1)
                {
                    mapCtx.fillStyle = 'cyan';
                    mapCtx.beginPath();
                    mapCtx.arc(circle.x, circle.y, 5 / Math.max(scaleX, scaleY), 0, 2 * Math.PI);
                    mapCtx.fill();
                    mapCtx.stroke();
                }
                else
                {
                    if (ROBOTICA_AVANZADA) {
                        mapCtx.fillStyle = 'yellow';
                        mapCtx.beginPath();
                        mapCtx.arc(circle.x, circle.y, 5 / Math.max(scaleX, scaleY), 0, 2 * Math.PI);
                        mapCtx.fill();
                        mapCtx.stroke();
                    }
                    else {
                        if(i == 0)
                        {
                        mapCtx.fillStyle = 'green';
                        mapCtx.beginPath();
                        mapCtx.arc(circle.x, circle.y, 5 / Math.max(scaleX, scaleY), 0, 2 * Math.PI);
                        mapCtx.fill();
                        mapCtx.stroke();
                        } else if(i == circles.length - 1)
                        {
                        mapCtx.fillStyle = 'red';
                        mapCtx.beginPath();
                        mapCtx.arc(circle.x, circle.y, 5 / Math.max(scaleX, scaleY), 0, 2 * Math.PI);
                        mapCtx.fill();
                        mapCtx.stroke();
                        }
                        else {
                        mapCtx.fillStyle = 'orange';
                        mapCtx.beginPath();
                        mapCtx.arc(circle.x, circle.y, 5 / Math.max(scaleX, scaleY), 0, 2 * Math.PI);
                        mapCtx.fill();
                        mapCtx.stroke();
                        }
                    }
                }
            }
            mapCtx.restore();
        }

        // Draw the crosshair if it exists
        if (crosshairX !== null && crosshairY !== null) {
            mapCtx.save();
            mapCtx.translate(mapCanvas.width / 2 + canvasOffsetX, mapCanvas.height / 2 + canvasOffsetY);
            mapCtx.scale(scaleX, scaleY);
            mapCtx.translate(-img.width / 2, -img.height / 2);
            mapCtx.strokeStyle = 'red';
            mapCtx.lineWidth = 1 / Math.max(scaleX, scaleY); // Adjusted for independent scaling
            mapCtx.beginPath();
            mapCtx.moveTo(crosshairX - 10 / Math.max(scaleX, scaleY), crosshairY);
            mapCtx.lineTo(crosshairX + 10 / Math.max(scaleX, scaleY), crosshairY);
            mapCtx.moveTo(crosshairX, crosshairY - 10 / Math.max(scaleX, scaleY));
            mapCtx.lineTo(crosshairX, crosshairY + 10 / Math.max(scaleX, scaleY));
            mapCtx.stroke();
            mapCtx.restore();
        }
    }

    function drawRobot() {
      // Calculate robot's logical coordinates
      const robotLogicalX = img.width - ((robotX - boundingBox.x_min) / (boundingBox.x_max - boundingBox.x_min)) * img.width;
      const robotLogicalY = ((robotY - boundingBox.y_min) / (boundingBox.y_max - boundingBox.y_min)) * img.height;

      // Calculate the size of the robot image
      const robotImgWidth = 0.6 * (img.width / (boundingBox.x_max - boundingBox.x_min));
      const robotImgHeight = 0.6 * (img.height / (boundingBox.y_max - boundingBox.y_min));

      // Log the coordinates and size for debugging
      // console.log(`Robot Logical Coordinates: x=${robotLogicalX}, y=${robotLogicalY}`);
      // console.log(`Robot Image Size: width=${robotImgWidth}, height=${robotImgHeight}`);
      // console.log(`Robot Orientation (radians): ${robotOrientation}`);

      // Draw the robot image
      mapCtx.save();
      mapCtx.translate(robotLogicalX, robotLogicalY);
      mapCtx.rotate(-robotOrientation + Math.PI / 2); // Invert the orientation and adjust by 90 degrees
      mapCtx.translate(-robotImgWidth / 2, -robotImgHeight / 2);
      mapCtx.drawImage(robotImg, 0, 0, robotImgWidth, robotImgHeight);
      mapCtx.restore();
    }

    function drawRobotOverview() {
        const overviewScale = Math.min(overviewCanvas.width / img.width, overviewCanvas.height / img.height);
        const overviewWidth = img.width * overviewScale;
        const overviewHeight = img.height * overviewScale;
        const overviewX = (overviewCanvas.width - overviewWidth) / 2;
        const overviewY = (overviewCanvas.height - overviewHeight) / 2;

        const robotLogicalX = overviewX + ((boundingBox.x_max - robotX) / (boundingBox.x_max - boundingBox.x_min)) * overviewWidth;
        const robotLogicalY = overviewY + ((robotY - boundingBox.y_min) / (boundingBox.y_max - boundingBox.y_min)) * overviewHeight;
        const robotImgWidth = 0.6 * (overviewWidth / (boundingBox.x_max - boundingBox.x_min));
        const robotImgHeight = 0.6 * (overviewHeight / (boundingBox.y_max - boundingBox.y_min));

        overviewCtx.save();
        overviewCtx.translate(robotLogicalX, robotLogicalY);
        overviewCtx.rotate(-robotOrientation + Math.PI / 2); // Invert the orientation and adjust by 90 degrees
        overviewCtx.translate(-robotImgWidth / 2, -robotImgHeight / 2);

        overviewCtx.fillStyle = 'red';
        overviewCtx.fillRect(0, 0, robotImgWidth, robotImgHeight);

        overviewCtx.drawImage(robotImg, 0, 0, robotImgWidth, robotImgHeight);
        overviewCtx.restore();
    }


    function getCanvasCoordinates(logicalX, logicalY) {
        const canvasX = (logicalX * img.width / (boundingBox.x_max - boundingBox.x_min)) + initialX;
        const canvasY = (logicalY * img.height / (boundingBox.y_max - boundingBox.y_min)) + initialY;
        return { canvasX, canvasY };
    }

    // Function to load circles from the server
    function loadCircles() {
        fetch('/get_goals')
            .then(response => response.json())
            .then(data => {
                circles = data.map(goal => {
                    const canvasCoords = getCanvasCoordinates(goal.x, goal.y);
                    return { x: canvasCoords.canvasX, y: canvasCoords.canvasY, r: goal.r};
                });
                draw(); // Function to draw circles on the canvas
                // console.log(circles);
            })
            .catch(error => console.error('Error loading circles:', error));
    }
    window.onload = loadCircles;
    setInterval(loadCircles, 400);


    function adjustOffset(factor, cx, cy) {
        const dx = (cx - mapCanvas.width / 2 - canvasOffsetX) * (1 - factor);
        const dy = (cy - mapCanvas.height / 2 - canvasOffsetY) * (1 - factor);
        canvasOffsetX += dx;
        canvasOffsetY += dy;
    }

    window.zoomIn = function() {
        adjustOffset(scaleMultiplier, mapCanvas.width / 2, mapCanvas.height / 2);  // Adjust zoom to center
        scaleX *= scaleMultiplier;
        scaleY *= scaleMultiplier;
        draw();
    }

    window.zoomOut = function() {
        const factor = 1 / scaleMultiplier;
        adjustOffset(factor, mapCanvas.width / 2, mapCanvas.height / 2);  // Adjust zoom to center
        scaleX *= factor;
        scaleY *= factor;
        draw();
    }

    window.confirmZoom = function() {
        console.log("Confirm Zoom button clicked");
        document.getElementById('zoomControls').style.display = 'none';
        panControls.style.display = 'block';
        // setZeroButton.style.display = 'inline-block';
        undoButton.style.display = 'inline-block';
        clearButton.style.display = 'inline-block';

        addPlaceCircleButton(); // Add the place circle button
        console.log("Pan Controls should be visible now");
        // Confirm zoom level
        zoomConfirmed = true;

        // Center the red square on the canvas
        centerRedSquare();

        // Start fetching the robot's position
        setInterval(fetchRobotPosition, refreshRate);
    }

    window.togglePanning = function() {
        isPanning = !isPanning;
        panButton.textContent = isPanning ? 'Pan (On)' : 'Pan (Off)';
        console.log(`Panning is now ${isPanning ? 'enabled' : 'disabled'}`);
    }

    window.setZeroMode = function() {
        isSettingZero = true;
        // setZeroButton.textContent = 'Set (0,0) (Active)';
        console.log('Set (0,0) mode activated');
    }

    window.undoLastCircle = function() {
        if (circles.length > 0) {
            circles.pop();
            draw();
            console.log('Last circle removed');
            exportCircles();  // Export circles when undoing
        }
    }

    function setZero(event) {
        if (!isSettingZero) return;

        const rect = mapCanvas.getBoundingClientRect();
        const mouseX = event.clientX - rect.left;
        const mouseY = event.clientY - rect.top;

        // Convert screen coordinates to canvas coordinates
        const canvasX = (mouseX - mapCanvas.width / 2 - canvasOffsetX) / scaleX + img.width / 2;
        const canvasY = (mouseY - mapCanvas.height / 2 - canvasOffsetY) / scaleY + img.height / 2;

        initialX = canvasX;
        initialY = canvasY;

        console.log(`New (0,0) set: x=${initialX}, y=${initialY}`);

        isSettingZero = false;
        // setZeroButton.textContent = 'Set (0,0)';
        draw();
    }

    function startPan(event) {
        if (!isPanning) return;
        startCoords = [event.clientX, event.clientY];
        console.log('Start Panning', startCoords);
        document.addEventListener('mousemove', doPan);
        document.addEventListener('mouseup', endPan);
        document.addEventListener('mouseleave', endPan);
        event.preventDefault();
    }

    function doPan(event) {
        const dx = (event.clientX - startCoords[0]);
        const dy = (event.clientY - startCoords[1]);
        // console.log('Panning', {dx, dy});
        canvasOffsetX += dx;
        canvasOffsetY += dy;
        startCoords = [event.clientX, event.clientY];
        draw();
        if (zoomConfirmed) {
            updateOverviewRectangle();
        }
    }

    function endPan() {
        console.log('End Panning');
        document.removeEventListener('mousemove', doPan);
        document.removeEventListener('mouseup', endPan);
        document.removeEventListener('mouseleave', endPan);
    }

    function handleWheel(event) {
        if (zoomConfirmed) return;  // Disable zoom after confirmation
        event.preventDefault();
        const factor = event.deltaY > 0 ? 1 / scaleMultiplier : scaleMultiplier;
        adjustOffset(factor, mapCanvas.width / 2, mapCanvas.height / 2);
        scaleX *= factor;
        scaleY *= factor;
        draw();
        if (zoomConfirmed) {
            updateOverviewRectangle();
        }
    }

    function centerRedSquare() {
        if (initialX != null && initialY != null) {
            // Center the red square on the canvas
            const canvasCenterX = mapCanvas.width / 2;
            const canvasCenterY = mapCanvas.height / 2;
            const imgCenterX = img.width / 2;
            const imgCenterY = img.height / 2;

            // Calculate the offsets to center the red square
            canvasOffsetX = (canvasCenterX - (initialX - imgCenterX) * scaleX) - mapCanvas.width / 2;
            canvasOffsetY = (canvasCenterY - (initialY - imgCenterY) * scaleY) - mapCanvas.height / 2;

            console.log(`Centering red square: canvasOffsetX=${canvasOffsetX}, canvasOffsetY=${canvasOffsetY}`);

            draw();

            // Draw the red rectangle on the overview canvas based on the red square's position
            updateOverviewRectangle();
        } else {
            console.error('Cannot center red square because it was not found.');
        }
    }

    let dib = false;
    let offpasx = 0;
    let offpasy = 0;
    function updateOverviewRectangle() {
        if (dib == false)
          {
            offpasx = canvasOffsetX / scaleX;
            offpasy = canvasOffsetY / scaleY;
            dib = true;
          }
        const overviewScale = Math.min(overviewCanvas.width / img.width, overviewCanvas.height / img.height);
        const overviewWidth = img.width * overviewScale;
        const overviewHeight = img.height * overviewScale;
        const overviewX = (overviewCanvas.width - overviewWidth) / 2;
        const overviewY = (overviewCanvas.height - overviewHeight) / 2;

        const rectWidth = mapCanvas.width / scaleX;
        const rectHeight = mapCanvas.height / scaleY;
        const rectX = initialX - canvasOffsetX / scaleX  + offpasx - rectWidth/2;
        const rectY = initialY - canvasOffsetY / scaleY + offpasy - rectHeight/2;

        // const overviewRectX = overviewX + (rectX * overviewScale);
        // const overviewRectY = overviewY + (rectY * overviewScale);
        const overviewRectX = overviewX + (rectX * overviewScale);
        const overviewRectY = overviewY + (rectY * overviewScale);
        const overviewRectWidth = rectWidth * overviewScale;
        const overviewRectHeight = rectHeight * overviewScale;

        overviewCtx.save();
        overviewCtx.clearRect(0, 0, overviewCanvas.width, overviewCanvas.height);  // Clear the overview canvas
        overviewCtx.drawImage(img, overviewX, overviewY, overviewWidth, overviewHeight);
        overviewCtx.strokeStyle = 'blue';
        overviewCtx.lineWidth = 2;
        overviewCtx.strokeRect(overviewRectX, overviewRectY, overviewRectWidth, overviewRectHeight);
        overviewCtx.restore();
    }

    function getLogicalCoordinates(canvasX, canvasY) {
        // console.log(`Canvas coordinates: x=${canvasX}, y=${canvasY}`);
        // console.log(`Initial coordinates: x=${initialX}, y=${initialY}`);

        const logicalX = (canvasX - initialX) * (boundingBox.x_max - boundingBox.x_min) / img.width;
        const logicalY = (canvasY - initialY) * (boundingBox.y_max - boundingBox.y_min) / img.height;

        return { logicalX, logicalY };
    }

    function addPlaceCircleButton() {
        const button = document.createElement('button');
        button.id = 'placeCircleButton';
        button.textContent = 'Place Circle (Off)';
        button.className = 'zoomButton';
        button.onclick = togglePlaceCircleMode;
        document.getElementById('panControls').appendChild(button);
    }

    function exportCircles() {
        const logicalCircles = circles.map(circle => {
            const logicalCoords = getLogicalCoordinates(circle.x, circle.y);
            return { x: logicalCoords.logicalX, y: logicalCoords.logicalY };
        });

        fetch('/update_goals', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify(logicalCircles)
        })
        .then(response => response.json())
        .then(data => console.log('Exported circles:', data))
        .catch(error => console.error('Error exporting circles:', error));
    }

    window.clearCircles = function() {
      circles = [];
      draw;
      exportCircles();
    }

    function togglePlaceCircleMode() {
        isPlacingCircles = !isPlacingCircles;
        const button = document.getElementById('placeCircleButton');
        button.textContent = isPlacingCircles ? 'Place Circle (On)' : 'Place Circle (Off)';
        console.log(`Place Circle Mode is now ${isPlacingCircles ? 'enabled' : 'disabled'}`);
        exportCircles();  // Export circles when toggling mode
    }

    mapCanvas.addEventListener('mousemove', function (event) {
        const rect = mapCanvas.getBoundingClientRect();
        const mouseX = event.clientX - rect.left;
        const mouseY = event.clientY - rect.top;

        // Convert screen coordinates to canvas coordinates
        const canvasX = (mouseX - mapCanvas.width / 2 - canvasOffsetX) / scaleX + img.width / 2;
        const canvasY = (mouseY - mapCanvas.height / 2 - canvasOffsetY) / scaleY + img.height / 2;

        crosshairX = canvasX;
        crosshairY = canvasY;

        // Adjust for independent x and y scales
        const { logicalX, logicalY } = getLogicalCoordinates(crosshairX, crosshairY);

        // if (!isNaN(logicalX) && !isNaN(logicalY)) {
        //     console.log(`Logical coordinates: x=${logicalX.toFixed(2)} meters, y=${logicalY.toFixed(2)} meters`);
        // }

        draw();
        if (zoomConfirmed) {
            updateOverviewRectangle();
        }
    });

    mapCanvas.addEventListener('mouseleave', function () {
        crosshairX = null;
        crosshairY = null;
        draw();
        if (zoomConfirmed) {
            updateOverviewRectangle();
        }
    });

    mapCanvas.addEventListener('mousedown', function (event) {
        if (event.button === 2) { // Right-click
            if (isPlacingCircles) {
                const rect = mapCanvas.getBoundingClientRect();
                const mouseX = event.clientX - rect.left;
                const mouseY = event.clientY - rect.top;

                // Convert screen coordinates to canvas coordinates
                const canvasX = (mouseX - mapCanvas.width / 2 - canvasOffsetX) / scaleX + img.width / 2;
                const canvasY = (mouseY - mapCanvas.height / 2 - canvasOffsetY) / scaleY + img.height / 2;


                circles.push({ x: canvasX, y: canvasY });
                console.log(`Placed circle at: x=${canvasX}, y=${canvasY}`);
                draw();
                exportCircles();  // Export circles after placing a new one
            }
        } else {
            if (isSettingZero) {
                setZero(event);
            }
            startPan(event);
        }
    });

    mapCanvas.addEventListener('contextmenu', function (event) {
        event.preventDefault();  // Prevent the default context menu from appearing
    });

    mapCanvas.addEventListener('wheel', handleWheel);

    function fetchRobotPosition() {
    fetch(urlRobotPosition)
        .then(response => response.json())
        .then(data => {
            robotX = data.x;
            robotY = data.y;

            // Assuming data.orientation is a quaternion with w, x, y, z
            const qw = data.orientation.w;
            const qx = data.orientation.x;
            const qy = data.orientation.y;
            const qz = data.orientation.z;

            // Calculate yaw (orientation around the z-axis)
            robotOrientation = Math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

            draw();
        })
        .catch(error => console.error('Error fetching robot position:', error));
}

});

let processId = null;
let imagePollingInterval = null;

// Function to send a request to reprocess the point cloud
function reprocessPointCloud() {
    // Clear the content of both canvases and display the message
    const overviewCanvas = document.getElementById('overviewCanvas');
    const overviewCtx = overviewCanvas.getContext('2d');
    const mapCanvas = document.getElementById('mapCanvas');
    const mapCtx = mapCanvas.getContext('2d');
    overviewCtx.clearRect(0, 0, overviewCanvas.width, overviewCanvas.height);
    mapCtx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);

    overviewCtx.font = "30px Arial";
    overviewCtx.fillStyle = "red";
    overviewCtx.textAlign = "center";
    overviewCtx.fillText("No hay mapa disponible", overviewCanvas.width / 2, overviewCanvas.height / 2);

    mapCtx.font = "30px Arial";
    mapCtx.fillStyle = "red";
    mapCtx.textAlign = "center";
    mapCtx.fillText("No hay mapa disponible", mapCanvas.width / 2, mapCanvas.height / 2);

    fetch('/reprocess_pointcloud', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        }
    })
    .then(response => response.json())
    .then(data => {
        document.getElementById('commandOutput').innerText = data.message || data.error;
        if (data.message) {
            // Start polling the server to check if the new image is ready
            imagePollingInterval = setInterval(checkImageReady, 1000); // Poll every 2 seconds
        }
    })
    .catch(error => {
        console.error('Error:', error);
        document.getElementById('commandOutput').innerText = 'An error occurred: ' + error.message;
    });
}

// Function to check if the new image is ready
function checkImageReady() {
    fetch('/image_ready')
    .then(response => response.json())
    .then(data => {
        if (data.ready) {
            clearInterval(imagePollingInterval); // Stop polling
            loadImage(); // Load the new image
        }
    })
    .catch(error => {
        console.error('Error:', error);
    });
}

// Function to load the new image onto the canvases
function loadImage() {
    const overviewCanvas = document.getElementById('overviewCanvas');
    const overviewCtx = overviewCanvas.getContext('2d');
    const mapCanvas = document.getElementById('mapCanvas');
    const mapCtx = mapCanvas.getContext('2d');
    const img = new Image();
    img.onload = function() {
        overviewCtx.clearRect(0, 0, overviewCanvas.width, overviewCanvas.height);
        mapCtx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
        mapCtx.drawImage(img, 0, 0, mapCanvas.width, mapCanvas.height);
        overviewCtx.drawImage(img, 0, 0, overviewCanvas.width, overviewCanvas.height);
    };
    img.src = '/image'; // Update the source to trigger the image load
    location.reload(true);
}


// Chart.js setup
const ctxCpu = document.getElementById('cpuChart').getContext('2d');
const cpuChart = new Chart(ctxCpu, {
    type: 'line',
    data: {
        labels: [], // Your labels here
        datasets: [{
            label: 'CPU Usage (%)',
            data: [], // Your data here
            backgroundColor: 'rgba(54, 162, 235, 0.2)',
            borderColor: 'rgba(54, 162, 235, 1)',
            fill: true, // Fill the area below the line
            borderWidth: 1
        }]
    },
    options: {
        scales: {
            y: {
                beginAtZero: true,
                ticks: {
                    stepSize: 50, // Set the step size for ticks
                    callback: function(value) {
                        return value + '%'; // Add percentage sign to ticks
                    }
                }
            },
            x: {
                display: false // Remove x-axis labels
            }
        },
        plugins: {
            legend: {
                display: true
            }
        }
    }
});

// Similar configurations for memoryChart and swapChart
const ctxMemory = document.getElementById('memoryChart').getContext('2d');
const memoryChart = new Chart(ctxMemory, {
    type: 'line',
    data: {
        labels: [], // Your labels here
        datasets: [{
            label: 'Memory Usage (%)',
            data: [], // Your data here
            backgroundColor: 'rgba(153, 102, 255, 0.2)',
            borderColor: 'rgba(153, 102, 255, 1)',
            fill: true, // Fill the area below the line
            borderWidth: 1
        }]
    },
    options: {
        scales: {
            y: {
                beginAtZero: true,
                ticks: {
                    stepSize: 50, // Set the step size for ticks
                    callback: function(value) {
                        return value + '%'; // Add percentage sign to ticks
                    }
                }
            },
            x: {
                display: false // Remove x-axis labels
            }
        },
        plugins: {
            legend: {
                display: true
            }
        }
    }
});

const ctxSwap = document.getElementById('swapChart').getContext('2d');
const swapChart = new Chart(ctxSwap, {
    type: 'line',
    data: {
        labels: [], // Your labels here
        datasets: [{
            label: 'Swap Usage (%)',
            data: [], // Your data here
            backgroundColor: 'rgba(255, 159, 64, 0.2)',
            borderColor: 'rgba(255, 159, 64, 1)',
            fill: true, // Fill the area below the line
            borderWidth: 1
        }]
    },
    options: {
        scales: {
            y: {
                beginAtZero: true,
                ticks: {
                    stepSize: 50, // Set the step size for ticks
                    callback: function(value) {
                        return value + '%'; // Add percentage sign to ticks
                    }
                }
            },
            x: {
                display: false // Remove x-axis labels
            }
        },
        plugins: {
            legend: {
                display: true
            }
        }
    }
});



// Function to update charts
function updateCharts() {
    fetch('/system_usage')
        .then(response => response.json())
        .then(data => {
            const currentTime = new Date().toLocaleTimeString();
            cpuChart.data.labels.push(currentTime);
            cpuChart.data.datasets[0].data.push(data.cpu);
            memoryChart.data.labels.push(currentTime);
            memoryChart.data.datasets[0].data.push(data.memory);
            swapChart.data.labels.push(currentTime);
            swapChart.data.datasets[0].data.push(data.swap);

            if (cpuChart.data.labels.length > 10) {
                cpuChart.data.labels.shift();
                cpuChart.data.datasets[0].data.shift();
                memoryChart.data.labels.shift();
                memoryChart.data.datasets[0].data.shift();
                swapChart.data.labels.shift();
                swapChart.data.datasets[0].data.shift();
            }



            cpuChart.update();
            memoryChart.update();
            swapChart.update();
        })
        .catch(error => console.error('Error fetching system usage data:', error));
}

// Update charts every 5 seconds
function loadImageToCanvas(imgPath, canvasId) {
    const img = new Image();
    img.src = imgPath + "?t=" + new Date().getTime(); // Append timestamp to prevent caching
    img.onload = function() {
        const canvas = document.getElementById(canvasId);
        const ctx = canvas.getContext('2d');
        ctx.clearRect(0, 0, canvas.width, canvas.height); // Clear the canvas before drawing
        ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
    }
}

// Function to update images at 2Hz
function updateImages() {
    loadImageToCanvas('/img_map', 'imgMapCanvas');
    loadImageToCanvas('/img_heatmap', 'imgHeatmapCanvas');
    loadImageToCanvas('/img_camera', 'imgCamera');
}

// Set interval to update images every 500 milliseconds (2Hz)
setInterval(updateImages, 200);
setInterval(updateCharts, 1000);

let processes = [];

function runTerminalCommand() {
    if (RVIZ)
        runCommand("cd ~ && cd FastLio/ && source devel/setup.bash && roslaunch fast_lio mapping_velodyne.launch", "Fast Lio", "fast_lio");
    else
        runCommand("cd ~ && cd FastLio/ && source devel/setup.bash && roslaunch fast_lio mapping_velodyne_norviz.launch", "Fast Lio", "fast_lio");
}

function stopTerminalCommand() {
    runCommand('rostopic pub /save_topic std_msgs/Bool data:= true', 'Save Map', 'save_map');
}

function startHeatMapping() {
    runCommand("cd ~ && cd pcd_ws/ && source devel/setup.bash && rosrun pcd_projector tray_parse.py", "Heat Mapping", "heat_mapping");

    if (ROBOTICA_AVANZADA){
        runCommand("cd ~ && cd pcd_ws/ && source devel/setup.bash && rosrun pcd_projector Gripper.py", "G R I P P Y", "gri_ppy");
    }
}

function startVelodyneNode() {
    runCommand("cd ~ && cd VLP_Vis/ && source devel/setup.bash && roslaunch velodyne_pointcloud VLP16_points.launch", "Velodyne Node", "velodyne_node");
}

function startWitmotionNode() {
    runCommand("cd ~ && cd WitmotionDriver/ && source devel/setup.bash && roslaunch witmotion_ros witmotion.launch", "Witmotion Node", "witmotion_node");
}

function startCameraNode() {
    if (ROBOTICA_AVANZADA){
        runCommand("cd ~ && cd pcd_ws/ && source devel/setup.bash && rosrun pcd_projector Flag_Tracker.py", "Flag Cam Node", "flag_camera_node");
        runCommand("cd ~ && cd pcd_ws/ && source devel/setup.bash && rosrun pcd_projector Flag_Publish.py", "Flag Publish", "flag_node");
    }else
        runCommand("cd ~ && cd pcd_ws/ && source devel/setup.bash && rosrun pcd_projector Person_Tracker.py", "Camera Node", "camera_node");
}

function startPersonNav() {
    runCommand("cd ~ && cd pcd_ws/ && source devel/setup.bash && rosrun pcd_projector Person_Follower.py", "Person Follower", "person_foll");
}

function startAStarNav() {
    runCommand("cd ~ && cd pcd_ws/ && source devel/setup.bash && rosrun pcd_projector Aut_Nav.py", "A* Navigation", "astar_nav");
}

function startGoalNav() {
    runCommand("cd ~ && cd pcd_ws/ && source devel/setup.bash && rosrun pcd_projector Goal_Nav_P1.py", "Goal Navigation", "goal_nav");
}

function startControllerNav() {
    runCommand("cd ~ && cd pcd_ws/ && source devel/setup.bash && rosrun pcd_projector Cont_Nav.py", "Controller Navigation", "controller_nav");
}

var lidar_on = 0;
function toggleLidar() {
  if(lidar_on == 0)
    {runCommand("curl -X POST \"http://192.168.1.201/cgi/setting\" -d \"rpm=1200\"", "Velodyne Toggle", "velo_tog");
    lidar_on = 1;
  }
  else {
    runCommand("curl -X POST \"http://192.168.1.201/cgi/setting\" -d \"rpm=0\"", "Velodyne Toggle", "velo_tog");
    lidar_on = 0;
  }
}

function runCommand(command, name, tag) {
    fetch('/run_command', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ command: command, tag: tag, name: name }),  // Send the name and tag in the request
    })
    .then(response => response.json())
    .then(data => {
        if (data.pid) {
            processes.push({ name: data.name, pid: data.pid, tag: data.tag });  // Use the name and tag from the response
            updateProcessList();
        }
        document.getElementById('commandOutput').textContent += data.output + '\n';
    })
    .catch((error) => {
        console.error('Error:', error);
        document.getElementById('commandOutput').textContent += 'Error: ' + error + '\n';
    });
}

function killSelectedProcess() {
    const processList = document.getElementById('processList');
    const selectedIndex = processList.selectedIndex;
    if (selectedIndex !== -1) {
        const selectedProcess = processes[selectedIndex];
        fetch('/stop_command', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ tag: selectedProcess.tag }),
        })
        .then(response => response.json())
        .then(data => {
            document.getElementById('commandOutput').textContent += data.message + '\n';
            processes.splice(selectedIndex, 1);
            updateProcessList();
        })
        .catch((error) => {
            console.error('Error:', error);
            document.getElementById('commandOutput').textContent += 'Error: ' + error + '\n';
        });
    }
}

function updateProcessList() {
    const processList = document.getElementById('processList');
    processList.innerHTML = '';
    processes.forEach((process, index) => {
        const option = document.createElement('option');
        option.value = index;
        option.textContent = `${process.name} (PID: ${process.pid})`;
        processList.appendChild(option);
    });
}

function reboot(){
  runCommand("reboot", "Reboot", "reb");

}

function shutdown(){
  runCommand("poweroff", "Shutdown", "shut");
}

function onHeatMapDoubleClick(){
    runCommand('rm -f /home/alvaro/Desktop/AGV_Data/trajectories.csv', 'Reset Trajectory', 'reset_tray');
}
