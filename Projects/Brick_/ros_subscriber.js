const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;

module.exports = {
  test: function (mainWindow) {
    test(mainWindow);
  },
  main: function (mainWindow) {
    main(mainWindow);
  },

};

let mainWindow;

function ros_subscriber(mainWindow){
	rosnodejs.initNode('/listener_node')
    .then((rosNode) => {
      // Create ROS subscriber on the 'chatter' topic expecting String messages
      sub = rosNode.subscribe('/chatter', std_msgs.String,
        (data) => { // define callback execution
    	  mainWindow.webContents.send('test_ros_topic', data.data);
          // console.log(data.data);
          // console.log(typeof data.data);
        }
      );
    });
}

function main(mainWindow){
  ros_subscriber(mainWindow);
}

// ros_subscriber();

function test(text){
  console.log(text);
  console.log("End");
}