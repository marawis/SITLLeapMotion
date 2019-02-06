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
      }

       var listKey = ['a','s','d','w','r','t']
       if (count_notif > 8 && listKey.includes(keyName) ) {
            $('#move li').first().remove();
        }

        console.log(count_notif)

    }, false);
});