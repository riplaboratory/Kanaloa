const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;
const sensor_msgs = rosnodejs.require('sensor_msgs').msg;

module.exports = {
  main: function (mainWindow) {
    main(mainWindow);
  }
};


function main(mainWindow){
	mainWindow.loadURL(`file://${__dirname}/index.html`);
	rosnodejs.initNode('/gps_node')
	.then((rosNode) => {

		sub = rosNode.subscribe('/wamvGps', sensor_msgs.NavSatFix,
		(data) => { 
		  mainWindow.webContents.send('wamvGps', data);

        }
      );
    });
}


