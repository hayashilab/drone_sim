let map;
function initMap() {
    map = new google.maps.Map(document.getElementById('map'), {
        center: { lat: 33.85316, lng: 130.50157 },
        zoom: 50,
        maxZoom: 150,
        mapTypeId: 'satellite',
        streetViewControl: false,
        fullscreenControl: false,
        mapTypeControl: false,
        rotateControl: true,
        scaleControl: true,
        zoomControl: true
    });
}

let marker;
let rectangle;
let targetmarker;
let targetMarkers = [];


function updateMap(latitude, longitude, heading, height, width, targetPositions) {
    const location = new google.maps.LatLng(latitude, longitude);
  
    if (marker) {
      marker.setMap(null);
    }
  
    if (rectangle) {
      rectangle.setMap(null);
    }
  
    if (targetMarkers) {
      targetMarkers.forEach(marker => marker.setMap(null));
    }
  
    targetMarkers = [];
  
    // ----------------------------------------------------------------------------------------- drone
    const symbol = {
      path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
      scale: 4,
      rotation: heading,
      fillColor: '#FF0000',
      fillOpacity: 1,
      strokeWeight: 2,
      strokeColor: '#FFFFFF',
      anchor: new google.maps.Point(0, 2.5)
    };
  
    marker = new google.maps.Marker({
      position: location,
      map: map,
      icon: symbol
    });
  
    // ----------------------------------------------------------------------------------------- camera frame
    const heightDegrees = height / 111111; // 1 degree of latitude is approximately 111,111 meters
    const widthDegrees = width / (111111 * Math.cos(latitude * Math.PI / 180)); // Adjust for longitude based on latitude
  
    const halfHeight = heightDegrees / 2;
    const halfWidth = widthDegrees / 2;
  
    const radians = heading * Math.PI / 180;
    const sinHeading = Math.sin(radians);
    const cosHeading = Math.cos(radians);
  
    const dx1 = -halfWidth * cosHeading - halfHeight * sinHeading;
    const dy1 = halfWidth * sinHeading - halfHeight * cosHeading;
    const dx2 = halfWidth * cosHeading - halfHeight * sinHeading;
    const dy2 = -halfWidth * sinHeading - halfHeight * cosHeading;
    const dx3 = halfWidth * cosHeading + halfHeight * sinHeading;
    const dy3 = -halfWidth * sinHeading + halfHeight * cosHeading;
    const dx4 = -halfWidth * cosHeading + halfHeight * sinHeading;
    const dy4 = halfWidth * sinHeading + halfHeight * cosHeading;
  
    const rectangleCoords = [
      { lat: latitude + dy1, lng: longitude + dx1 },
      { lat: latitude + dy2, lng: longitude + dx2 },
      { lat: latitude + dy3, lng: longitude + dx3 },
      { lat: latitude + dy4, lng: longitude + dx4 }
    ];

    //console.log(rectangleCoords)
  
    rectangle = new google.maps.Polygon({
      paths: rectangleCoords,
      map: map,
      fillColor: 'rgba(0, 255, 0, 0.3)',
      strokeColor: 'green',
      strokeWeight: 2
    });
  


    // -----------------------------------------------------------------------------------------  garbage
    //targetmarker = new google.maps.Marker({
    //    position: targetlocation,
    //    map: map,
    //    title: "garbage"
    //})
    
    targetPositions.forEach(position => {
        const [targetLatitude, targetLongitude] = position.split(',').map(Number);
        console.log( [targetLatitude, targetLongitude])
        if (!isNaN(targetLatitude) && !isNaN(targetLongitude)) {
            const targetLocation = new google.maps.LatLng(targetLatitude, targetLongitude);
        
            // Check if a marker for this position already exists
            const existingMarker = targetMarkers.find(marker => marker.getCenter().equals(targetLocation));
        
            if (!existingMarker) {
              // If no existing marker, create a new one
              const targetPoint = new google.maps.Circle({
                center: targetLocation,
                radius: 0.5,
                map: map,
                fillColor: 'red',
                fillOpacity: 1,
                strokeWeight: 0
              });
        
              targetMarkers.push(targetPoint);
            }
          } else {
            console.error(`Invalid coordinates: ${position}`);
          }
    });
}

document.getElementById('center-button').addEventListener('click', function() {
    if (marker) {
        map.setCenter(marker.getPosition());
    }
});

function fetchGUIData() {
    fetch("/gui_data")
      .then((response) => response.json())
      .then((data) => {
        document.getElementById("latitude").textContent = data.latitude;
        document.getElementById("longitude").textContent = data.longitude;
        document.getElementById("time_passed").textContent = data.time_passed + " second"; 
        document.getElementById("height_meter").textContent = data.height_meter + " m";
        document.getElementById("width_meter").textContent = data.width_meter + " m";
        document.getElementById("gps_altitude").textContent = data.gps_altitude + " m";
        document.getElementById("laser_altitude").textContent =  data.laser_altitude + " m";
        document.getElementById("heading").textContent = data.heading + " degree";
        
        const targetPositions = data.target_position
            .trim()
            .split(";")
            
    
        targetCount = targetPositions.length;
     
        document.getElementById("target-count").textContent = targetCount;
                
        const latitude = parseFloat(data.latitude);
        const longitude = parseFloat(data.longitude);
        const heading = parseFloat(data.heading);
        const height = parseFloat(data.height_meter);
        const width = parseFloat(data.width_meter);
  
        updateMap(
          latitude,
          longitude,
          heading,
          height,
          width,
          targetPositions
        );
        
      });
  }

setInterval(fetchGUIData, 500);
window.onload = fetchGUIData;

google.maps.event.addDomListener(window, 'load', initMap);


let intervalId = null;

function startMovement(direction) {
    if (!intervalId) {
        sendMovementCommand(direction);
        intervalId = setInterval(() => {
            sendMovementCommand(direction);
        }, 100); // Adjust the interval as needed
    }
}

function stopMovement() {
    if (intervalId) {
        clearInterval(intervalId);
        intervalId = null;
        sendStopCommand();
    }
}

document.getElementById('forward-button').addEventListener('mousedown', function() {
    startMovement('forward');
});

document.getElementById('left-button').addEventListener('mousedown', function() {
    startMovement('left');
});

document.getElementById('right-button').addEventListener('mousedown', function() {
    startMovement('right');
});

document.getElementById('backward-button').addEventListener('mousedown', function() {
    startMovement('backward');
});

document.getElementById('cw-button').addEventListener('mousedown', function() {
    startMovement('cw');
});

document.getElementById('ccw-button').addEventListener('mousedown', function() {
    startMovement('ccw');
});

document.getElementById('up-button').addEventListener('mousedown', function() {
    startMovement('up');
});

document.getElementById('down-button').addEventListener('mousedown', function() {
    startMovement('down');
});

['forward-button', 'left-button', 'right-button', 'backward-button','cw-button', 'ccw-button','up-button','down-button'].forEach(buttonId => {
    document.getElementById(buttonId).addEventListener('mouseup', stopMovement);
    document.getElementById(buttonId).addEventListener('mouseleave', stopMovement);
});

function sendMovementCommand(direction) {
    fetch('/move_drone', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ direction: direction })
    });
}

function sendStopCommand() {
    fetch('/stop_drone', {
        method: 'POST'
    });
}

document.getElementById('toggle-lines-button').addEventListener('click', function() {
    // Toggle the button state
    this.classList.toggle('active');
    
    // Get the current state of the button
    const showLines = this.classList.contains('active');
    
    // Send the button state to the Python backend
    fetch('/toggle_lines', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ show_lines: showLines })
    });
});


document.getElementById('auto-mode-button').addEventListener('click', function() {
    setVehicleMode('AUTO');
});

document.getElementById('guided-mode-button').addEventListener('click', function() {
    setVehicleMode('GUIDED');
});

document.getElementById('rtl-mode-button').addEventListener('click', function() {
    setVehicleMode('RTL');
});

function setVehicleMode(mode) {
    fetch('/set_vehicle_mode', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ mode: mode })
    })
    .then(response => {
        if (response.ok) {
            console.log(`${mode} mode activated`);
        } else {
            console.error(`Failed to activate ${mode} mode`);
        }
    })
    .catch(error => {
        console.error('Error:', error);
    });
}

document.getElementById('reset-checkpoint-button').addEventListener('click', function() {
    fetch('/reset_checkpoint', {
        method: 'POST'
    });
});

document.getElementById('spawn-objects-button').addEventListener('click', function() {
    fetch('/spawn_objects', {
        method: 'POST'
    });
});

document.getElementById('delete-objects-button').addEventListener('click', function() {
    fetch('/delete_objects', {
        method: 'POST'
    });
});

function refreshMap() {
    fetch('/show_map_result', {
      method: 'POST'
    })
      .catch(error => {
        console.log('Error updating map result:', error);
      })
      .then(response => response.blob())
      .then(blob => {
        const url = URL.createObjectURL(blob);
        document.getElementById('plotImage').src = url;
      });
    
  }
  
  setInterval(refreshMap, 5000);