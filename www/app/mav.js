// Shorthand for $( document ).ready()
$(function() {
    console.log( "ready!" );
    // Create a client instance
    var topic_cmd = "GARUDA_01/cmd"
    var topic_mav = "GARUDA_01/mav"
    var localhost =
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
      message.destinationName = "World";
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
        $("#gps").html("GPS Status : " + gps_status(json_res.gps.gps_fix))
        $("#battery").html("Battery : " + json_res.battery + " %")
        $("#mode").html("Flight Mode : " + json_res.flight_mode)
        $("#gnd_speed").html("Ground Speed : " + json_res.velocity + " m/s" )
        $("#altitude").html("Altitude : " + json_res.altitude + " m")

        if(!maps_status){
            console.log("Draw Maps")
            draw_maps(json_res.gps.latitude, json_res.gps.longitude)
        } else {
            polyline.addLatLng([json_res.gps.latitude, json_res.gps.longitude]);
            map.fitBounds(polyline.getBounds());
        }

      }else if(message.topic == topic_cmd) {
        console.log(message.payloadString)
      }
    }

    function gps_status(type){
        if(type == 1)
            return 'Not Fix'
        else if (type == 2)
            return '2D fix'
        else if (type == 3)
            return '3D fix'
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