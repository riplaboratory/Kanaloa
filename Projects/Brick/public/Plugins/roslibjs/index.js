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
}
