module.exports = {
  test: function (mainWindow) {
    test(mainWindow);
  },
  main: function (mainWindow, ipcMain) {
    main(mainWindow, ipcMain);
  },

};



let mainWindow;


function main(mainWindow, ipcMain){
  mainWindow.loadURL(`file://${__dirname}/index.html`);

  // var Quaternion = require('quaternion');
  //
  // var q = new Quaternion({x: 0.0931212382,
  // y: 0.9602833958,
  // z: -0.2596249662,
  // w: 0.041961625 });
  //
  // console.log(q)
  //
  // var yaw = Math.atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
  // console.log(yaw);
}
