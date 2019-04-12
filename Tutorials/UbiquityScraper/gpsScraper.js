const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;
const sensor_msgs = rosnodejs.require('sensor_msgs').msg;

const fs = require("fs");

let gps1_lat;
let gps1_long;
let gps2_lat;
let gps2_long;

var data_csv_labels = "Time Stamp, GPS1 latitude, GPS1 longitude, GPS2 latitude, GPS2 longitude"

fs.appendFile('gps_log.csv', data_csv_labels, function (err) {
  if (err) throw err;
  console.log(data_csv_labels);
});

function log_gps_data(){
  setTimeout(log_gps_data, 2000);  //repeat function every 2 seconds

    var today = new Date();
    var comp_time = today.getHours() + ":" + today.getMinutes() + ":" + today.getSeconds();
    var data_string = comp_time + ", " + gps1_lat + ", " + gps1_long + ", " + gps2_lat + ", " + gps2_long + "\n"

    fs.appendFile('gps_log.csv', data_string, function (err) {
      if (err) throw err;
      console.log(data_string);
    });
}

log_gps_data();

rosnodejs.initNode('/listener_node')
    .then((rosNode) => {
      // Create ROS subscriber on the 'chatter' topic expecting String messages
      sub = rosNode.subscribe('/mruhGps', sensor_msgs.NavSatFix,
        (data) => { // define callback execution
          gps1_lat = data.latitude
          gps1_long = data.longitude
        }
      );

      sub2 = rosNode.subscribe('/fix', sensor_msgs.NavSatFix,
        (data) => { // define callback execution
          gps2_lat = data.latitude
          gps2_long = data.longitude
        }
      );

});
