$(function() {
    document.addEventListener('keydown', (event) => {
      const keyName = event.key;

      if (keyName === 'w') {
        console.log('Depan');
        message = new Paho.MQTT.Message("w");
        message.destinationName = topic_move;
        client.send(message);
        $('#move').append('<li><i class="fa fa-angle-up"></i> Move forward</li>')
        count_notif++;
      } else if (keyName == 's') {
        console.log('Belakang')
        message = new Paho.MQTT.Message("s");
        message.destinationName = topic_move;
        client.send(message);
        $('#move').append('<li> <i class="fa fa-angle-down"></i> Move backward</li>')
        count_notif++;
      } else if (keyName == 'd') {
        console.log('Kanan')
        message = new Paho.MQTT.Message("d");
        message.destinationName = topic_move;
        client.send(message);
        $('#move').append('<li> <i class="fa fa-angle-right"></i> Move Right</li>')
        count_notif++;
      } else if (keyName == 'a') {
        console.log('Kiri')
        message = new Paho.MQTT.Message("a");
        message.destinationName = topic_move;
        client.send(message);
        $('#move').append('<li> <i class="fa fa-angle-left"></i> Move Left</li>')
        count_notif++;
      } else if (keyName == 'r') {
        console.log("RTL")
        message = new Paho.MQTT.Message("r");
        message.destinationName = topic_move;
        client.send(message);
        $('#move').append('<li> <i class="fas fa-plane-arrival"></i> Return to Launch </li>')
        count_notif++;
      } else if (keyName == 't') {
        console.log("Take Off")
        message = new Paho.MQTT.Message("t");
        message.destinationName = topic_move;
        client.send(message);
        $('#move').append('<li> <i class="fas fa-plane-departure"></i>Taking off </li>')
        count_notif++;
      } else if (keyName == 'q') {
        console.log("Yaw Left")
        message = new Paho.MQTT.Message("q");
        message.destinationName = topic_move;
        client.send(message);
        $('#move').append('<li> <i class="fas fa-plane-departure"></i>Yaw Left </li>')
        count_notif++;
      } else if (keyName == 'e') {
        console.log("Yaw Right")
        message = new Paho.MQTT.Message("e");
        message.destinationName = topic_move;
        client.send(message);
        $('#move').append('<li> <i class="fas fa-plane-departure"></i>Yaw Right </li>')
        count_notif++;
      } else if (keyName == '1') {
        console.log("Move Up")
        message = new Paho.MQTT.Message("1");
        message.destinationName = topic_move;
        client.send(message);
        $('#move').append('<li> <i class="fas fa-plane-departure"></i>Move Up</li>')
        count_notif++;
      } else if (keyName == '1') {
        console.log("Move Down")
        message = new Paho.MQTT.Message("2");
        message.destinationName = topic_move;
        client.send(message);
        $('#move').append('<li> <i class="fas fa-plane-departure"></i>Move Down</li>')
        count_notif++;
      }

       var listKey = ['a','s','d','w','r','t','q','e','1','2']
       if (count_notif > 8 && listKey.includes(keyName) ) {
            $('#move li').first().remove();
        }

        console.log(count_notif)

    }, false);
});