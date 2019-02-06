// Shorthand for $( document ).ready()
 var topic_cmd = "GARUDA_01/cmd"
 var topic_mav = "GARUDA_01/mav"
 var topic_move = "GARUDA_01/move"
 var count_notif = 0;
$(function() {
    console.log( "ready!" );
    // Create a client instance

    client = new Paho.MQTT.Client(location.hostname, 8083, "clientId");

    // set callback handlers
    client.onConnectionLost = onConnectionLost;
    client.onMessageArrived = onMessageArrived;

    // connect the client
    client.connect({onSuccess:onConnect});


    // called when the client connects
    function onConnect() {
      // Once a connection has been made, make a subscription and send a message.
      console.log("onConnect");
      client.subscribe(topic_cmd);
      client.subscribe(topic_mav);
      message = new Paho.MQTT.Message("Hello");
      message.destinationName = topic_move;
      client.send(message);
    }

    // called when the client loses its connection
    function onConnectionLost(responseObject) {
      if (responseObject.errorCode !== 0) {
        console.log("onConnectionLost:"+responseObject.errorMessage);
      }
    }

    // called when a message arrives
    var json_res = {}
    var maps_status = false;
    var polyline
    function onMessageArrived(message) {
      // topic filter

      if(message.topic == topic_mav) {
        console.log("MAV")
        //console.log(message.payloadString)
        json_res = JSON.parse(message.payloadString)
        console.log(json_res)
        // draw html dom
        $("#gps").html("<i class='fas fa-location-arrow'></i> " + gps_status(json_res.gps.gps_fix))
        $("#battery").html( battery_icon(json_res.battery) + json_res.battery + " %")
        $("#mode").html("<i class='fas fa-code-branch'></i> " + json_res.flight_mode)
        $("#gnd_speed").html("<i class='fas fa-tachometer-alt'></i> " +  json_res.velocity.toFixed(2) + " m/s" )
        $("#altitude").html( alt_icon(json_res.altitude,json_res.max_altitude) + json_res.altitude + " m")
        $("#flight_time").html('<i class="fas fa-clock"></i> ' + json_res.flight_time)

        if(!maps_status){
            console.log("Draw Maps")
            draw_maps(json_res.gps.latitude, json_res.gps.longitude)
        } else {
            polyline.addLatLng([json_res.gps.latitude, json_res.gps.longitude]);
            map.fitBounds(polyline.getBounds());
        }

      }else if(message.topic == topic_cmd) {
        command = message.payloadString;

        console.log(message.payloadString)

        if (count_notif > 8) {
            $('#move li').first().remove();
        }
        if (command == '0') {
            $('#move').append('<li> Move forward</li>')
        } else if (command == '1') {
            $('#move').append('<li> Move backward</li>')
        } else if (command == '2'){
            $('#move').append('<li> Move Left</li>')
        } else if (command == '3') {
            $('#move').append('<li> Yaw Right</li>')
        } else if (command == '4') {
            $('#move').append('<li> Yaw Left</li>')
        } else if (coommand == '5') {
            $('#move').append('<li> Move Right</li>')
        }
        count_notif += 1;
      }
    }

    function gps_status(type){
        if(type == 1)
            return 'Not Fix'
        else if (type == 2)
            return '2D fix'
        else if (type == 3)
            return '3D fix'
        else if(type == 4)
            return '3D dgps'
    }

    function battery_icon(battery_level){
        if(battery_level >= 100){
            return "<i class='fas fa-battery-full'></i> "
        }else if (battery_level >= 75) {
            return "<i class='fas fa-battery-three-quarters'></i> "
        } else if (battery_level >= 50) {
            return "<i class='fas fa-battery-half'></i> "
        } else if (battery_level >= 25) {
            return "<i class='fas fa-battery-quarter'></i> "
        } else  {
            return "<i class='fas fa-battery-empty></i> "
        }
    }

    function alt_icon(alt,max_alt){
        if(alt >= max_alt * 1) {
            return '<i class="fas fa-thermometer-full"></i> '
        } else if (alt >= max_alt * 3/4) {
            return '<i class="fas fa-thermometer-three-quarters"></i> '
        } else if (alt >= max_alt * 1/2) {
            return '<i class="fas fa-thermometer-half"></i> '
        } else if (alt >= max_alt * 1/4) {
            return '<i class="fas fa-thermometer-quarter"></i> '
        } else  {
            return '<i class="fas fa-thermometer-empty"></i> '
        }
    }

    // draw maps
    var map = {}
    function draw_maps(lat,long){
        map = L.map('map', {
            center: [lat, long],
            zoom: 50
        });
        L.tileLayer('https://api.tiles.mapbox.com/v4/{id}/{z}/{x}/{y}.png?access_token={accessToken}', {
            attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors, <a href="https://creativecommons.org/licenses/by-sa/2.0/">CC-BY-SA</a>, Imagery © <a href="https://www.mapbox.com/">Mapbox</a>',
            maxZoom: 18,
            id: 'mapbox.streets',
            accessToken: 'pk.eyJ1IjoiemVyb29uZXRtIiwiYSI6ImNqbnhlcG42NDB3bGUzcG5kazdhNG9uYzkifQ.x2Th970t4YOd53O4Eu1J_Q'
        }).addTo(map);

        polyline = L.polyline([]).addTo(map);

        maps_status = true;
    }
});