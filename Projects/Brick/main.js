const fs = require('fs');
const path = require('path');
const electron = require('electron');

const exec = require('child_process').exec

let child_window_num = 0;
let main_window_indicator = false;
let window_location = [0, 0]
let window_position_change = [0, 0];
let child = [];
let first_time_init = false;


const { app, BrowserWindow, Menu , ipcMain} = electron;

let mainWindow;
let sub;

app.on('ready', () => {

  mainWindow = new BrowserWindow({nodeIntegration: true, icon: path.join(__dirname, `./public/images/Kanaloa_logo4.png`) });
//  mainWindow = new BrowserWindow({nodeIntegration: true, icon: path.join(__dirname, `./public/images/Kanaloa_logo1_filled.png`) });
  mainWindow.loadURL(`file://${__dirname}/index.html`);
  main_window_indicator = true;

  if (first_time_init){
    child.push(new BrowserWindow({ nodeIntegration: true, parent: mainWindow, icon: path.join(__dirname, `./public/images/Kanaloa_logo4.png`)  }));
//    child.push(new BrowserWindow({ nodeIntegration: true, parent: mainWindow, icon: path.join(__dirname, `./public/images/Kanaloa_logo1_filled.png`)  }));
    child[child_window_num].loadURL(`file://${__dirname}/public/Plugins/test_ros_subscriber/index.html`);
    child[child_window_num].setMenuBarVisibility(false)
    child_window_num += 1;

    first_time_init = false;
  }

  check_for_new_plugins('./public/Plugins/', Menu);

});


function function1(){

  mainWindow.on('move', function() {

    position = mainWindow.getPosition();
    console.log("Window Position: " + position);
    x = position[0];
    y = position[1];
    window_position_change = [window_location[0]-x, window_location[1]-y]
    console.log("Position change");
    console.log(window_position_change);
    //then change position of child windows

    window_location = [x, y];

    for (var child_window in child){

      child_pos = child[child_window].getPosition();
      console.log("Child POS")
      console.log(child_pos);
      child[child_window].setPosition(child_pos[0] - window_position_change[0], child_pos[1] - window_position_change[1] );
      console.log("Y: " + child_pos[1] + " - " + window_position_change[1])
      console.log("Child position " + child_window + child[child_window].getPosition());

    }


});
}

ipcMain.on('ipc_init', (event, text) => {;
  console.log(text);
  mainWindow.webContents.send('ipc_init', "IPC Started JS");

});

const menuTemplate = [

  {
    label: 'File',
    submenu: [
      {
        label: "Back To Home Screen",
        click() {
          mainWindow.loadURL(`file://${__dirname}/index.html`);
        }
      },
      {
        label: 'Quit',
        accelerator: process.platform === 'darwin' ? 'Command+Q' : 'Ctrl+Q', //If true return Command+Q if false return Ctrl+Q
        click() {
          app.quit();
        },
      }
    ]
  },
  {
    label: 'Edit',
    submenu: [
      { role: 'undo' },
      { role: 'redo' },
      { type: 'separator' },
      { role: 'cut' },
      { role: 'copy' },
      { role: 'paste' },
    ]
  },
  {
    label: 'View',
    submenu: [
      { role: 'reload' },
      { role: 'forcereload' },
      { role: 'toggledevtools' },
      { type: 'separator' },
      { role: 'resetzoom' },
      { role: 'zoomin' },
      { role: 'zoomout' },
      { type: 'separator' },
      { role: 'togglefullscreen' },
      { type: 'separator' },
      { role: 'minimize' },
    ]
  },
  {
    label: 'Robot',
    submenu: [

      {
        label: 'Start roscore locally',
        click() {
          console.log("roscore funtionality not developed yet");
        }
      },
      {
        label: "Change ROS IP",
        click() {
          console.log("roscore funtionality not developed yet");
        }
      },
      { type: 'separator' },
      {
        label: "Run Launch File",
        click() {
          console.log("roscore funtionality not developed yet");
        }
      },
    ]
  },


  {
    label: 'Plugins',
    submenu: [
      { type: 'separator' },
      {
        label: "Add New Plugin",
        click() {
          console.log("roscore funtionality not developed yet");
        }
      }
    ]
  },
];




function init_plugins_menu(Menu){
  const fs = require('fs');

  let plugindata = fs.readFileSync('plugins.json');
  plugindata = JSON.parse(plugindata);

  for (var plugin in plugindata){

    var plugin_name = plugindata[plugin]["name"];
    plugin_folder = plugindata[plugin]["plugin_folder"];

    var plugin_submenu = {
        "label": plugin_name,
        "plugin_folder": plugin_folder,
        click(menuItem, browserWindow, event){
          plugin_require = require('./public/Plugins/' + menuItem.plugin_folder + '/index.js');
          plugin_require.main(mainWindow, ipcMain);
        }
      } //End of submenu
      menuTemplate[4]["submenu"].unshift(plugin_submenu);
  } //End of for loop
  var mainMenu = Menu.buildFromTemplate(menuTemplate);
  Menu.setApplicationMenu(mainMenu);
}

function check_for_new_plugins(path, Menu){

  var plugin_directories = fs.readdirSync(path);
  plugindata = fs.readFileSync('plugins.json');
  plugindata = JSON.parse(plugindata);
  var plugin_json_directories = [];
  for (var plugin in plugindata){
    plugin_json_directories.push(plugin);
  }

  if (plugin_directories.length != plugin_json_directories.length){
    var all_plugin_info = {};

    console.log("New Plugins Detected");
    for (plugin in plugin_directories){
      plugin = plugin_directories[plugin];

      if (fs.existsSync('./public/Plugins/'+plugin+'/plugin_info.json')) {
        plugindata = fs.readFileSync('./public/Plugins/'+plugin+'/plugin_info.json');
        plugindata = JSON.parse(plugindata);
        all_plugin_info[plugin] = {"name" : plugindata["name"], "repo" : plugindata["repo"], "plugin_folder" : plugindata["plugin_folder"]};
      }

    }

    all_plugin_info = JSON.stringify(all_plugin_info, null, "\t");
    fs.writeFile('plugins.json', all_plugin_info, (err) => {
      if (err) {throw err; console.log("Error"); console.log(err);}
          console.log('plugins.json file updated');
          init_plugins_menu(Menu);

      });
  } else {
    init_plugins_menu(Menu);
  }

}


function rostopic_list(callback){
  exec("rostopic list", (err, stdout, stderr) => {
    var res = stdout.split("\n");
    res.splice(-1,1);
    callback(res);
})
}
