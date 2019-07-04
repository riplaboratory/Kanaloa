const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;
const sensor_msgs = rosnodejs.require('sensor_msgs').msg;

const exec = require('child_process').exec
const glob = require('glob');

module.exports = {
  main: function (mainWindow, ipcMain) {
    main(mainWindow, ipcMain);
  },

};

let mainWindow;

function ros_subscriber(mainWindow){
  console.log(`file://${__dirname}/index.html`);
  mainWindow.loadURL(`file://${__dirname}/index.html`);

}

function main(mainWindow, ipcMain){
  ros_subscriber(mainWindow);

  let subscribers = [];
  // let ros_node = rosnodejs.initNode('/ros_dashboard');

  function sub(topic_name, message_type, rosNode = ros_node){
    rosNode.subscribe(topic_name, message_type,
      (data) => {
        mainWindow.webContents.send(topic_name, data);
      }
    );
  }



    function rostopic_list(callback){
      exec("rostopic list", (err, stdout, stderr) => {
        var res = stdout.split("\n");
        res.splice(-1,1);
        console.log(res);
        callback(res);
    })
    }

  ipcMain.on('rostopic_list', (event, text) => {
      console.log("Rostopic list js triggered");
      rostopic_list(function (result) {
        mainWindow.webContents.send('rostopic_list', result);
      });
});


////////////////////////////////////////////////////////////////////////////////
//Raymonds Shit

const { dialog } = require('electron');

function roslaunch_list(callback){
  exec("rospack list", (err, stdout, stderr) => {
  var package_list = [];
  var launch_files = [];
  var res = stdout.split("\n");
  res.splice(-1,1);
  // console.log(res)
  for (package in res){
    package_list.push(res[package].split(" "));
  }
  for (package in package_list){
    var path = package_list[package][1];
    files = glob.sync(path + '/**/*.launch', {});
      if (files != undefined && files.length != 0) {
        for (launch_file in files){
          split_file = files[launch_file].split("/");
          launch_files.push([package_list[package][0], split_file[split_file.length-1]]);
          // console.log([package_list[package][0], split_file[split_file.length-1]]);
        }
      } // end if not undefined
    // }) // end glob file checking
  } //end for loop
  setTimeout(callback, 1000, launch_files);
  // callback(launch_files);
})
}

  ipcMain.on('roslaunch_list', (event, text) => {
    console.log("List all ROSLAUNCH:");

    roslaunch_list(function (result){
      console.log("Result");
      console.log(result);
      mainWindow.webContents.send('roslaunch_list',result);
    })
    // console.log("Launch Files Completed: ");
    // console.log(launch_files);
  });

  ipcMain.on('launch_file_upload', (event, text) => {

        var launch_file = dialog.showOpenDialog({properties: ['openFile'] })
        console.log(launch_file)

        if (launch_file != undefined){
          mainWindow.webContents.send('launch_file_upload', launch_file[0]);
          var command_str = "roslaunch " + launch_file

          exec(command_str, (err, stdout, stderr) => {
            // console.log(err);
            console.log(stdout);
            console.log(stderr);
          })
        }
      })

    ipcMain.on('launch_file_from_dropdown', (event, text) => {
      console.log(text)

      if (text != undefined){
        var command_str = "roslaunch " + text
        console.log(command_str);

        exec(command_str, (err, stdout, stderr) => {
          // console.log(err);
          console.log(stdout);
          console.log(stderr);
        })
      }
    })

    ipcMain.on('bag_file_save_folder', (event, text) => {

      var launch_folder = dialog.showOpenDialog({properties: ['openDirectory'] })
      console.log(launch_folder)
      if (launch_folder != undefined){
        mainWindow.webContents.send('bag_file_save_folder', launch_folder[0]);
      }
    })

    ipcMain.on('bag_file_play_location', (event, text) => {

      var launch_folder = dialog.showOpenDialog({properties: ['openFile'] })
      console.log(launch_folder)
      if (launch_folder != undefined){
        mainWindow.webContents.send('launch_save_path', launch_folder[0]);
      }
    })

    ipcMain.on('record_bag', (event, info_array) => {

      // console.log(info_array);
      var command_str = "rosbag record --split --size=";
      command_str += info_array[0];
      command_str += ' -o ' + info_array[2] + '/ ';
      topic_list = info_array[1]
      topic_list = topic_list.toString();
      console.log(topic_list);
      topic_list = topic_list.replace(/,/g, ' ');
      console.log(topic_list)
      command_str += topic_list;
      command_str += " __name:=gui_bag_recording";
      console.log(command_str);

      exec(command_str, (err, stdout, stderr) => {
          // console.log(err);
          console.log(stdout);
          // console.log(stderr);
        })
    })

    ipcMain.on('stop_bag_recording', (event, info_array) => {
      exec("rosnode kill /gui_bag_recording", (err, stdout, stderr) => {
          // console.log(err);
          console.log(stdout);
          // console.log(stderr);
        })
    })

    ipcMain.on('ros_node_kill', (event, text) => {
      console.log("Killing all ROS nodes");
      exec("rosnode kill -a", (err, stdout, stderr) => {
    })
    });

    ipcMain.on('map_server_start', (event, text) => {
      console.log("Starting Map Server: ")
      // exec("sudo service docker start", (err, stdout, stderr) => {
      // console.log(err);
      // console.log(stdout);
      // console.log(stderr);
      // console.log("ERR: ");
      // console.log(err);
      // if (err == null){
        exec("cd public/Plugins/ros_dashboard/assets && sudo docker run --rm -v $(pwd):/data -p 8082:80 klokantech/openmaptiles-server", (err, stdout, stderr) => {
        // exec("sudo docker  run  --rm -v  $(pwd):/data  -p  8081:80  klokantech/tileserver-gl .public/Plugins/image_testing/oahu.mbtiles ", (err, stdout, stderr) => {
          console.log("Socket Command Run")
          console.log(err);
          console.log("STDOUT")
          console.log(stdout);
          console.log("STDERR")
          console.log(stderr);
          if (stderr != null){
            console.log("This port is already used. Map Tile server may already be running, or another application is using the same port")
          }
        })
      // }
    // })

    });
    ipcMain.on('map_server_stop', (event, text) => {
      console.log(text);
      exec("service docker stop", (err, stdout, stderr) => {
        // console.log(err);
      console.log(stdout);
      console.log(stderr);
    })
    });

    ipcMain.on('image_topics', (event, text) => {
      console.log(text);

      exec("rostopic find sensor_msgs/CompressedImage && rostopic find sensor_msgs/Image", (err, stdout, stderr) => {
        // console.log(err);
      console.log(stdout);
      console.log(stderr);

      var res = stdout.split("\n");
      res.splice(-1,1);
      mainWindow.webContents.send('image_topics',res);
    });
    });

    ipcMain.on('gps_topics', (event, text) => {
      console.log("Gps topics index js called")
      console.log(text);

      exec("rostopic find sensor_msgs/NavSatFix", (err, stdout, stderr) => {
        // console.log(err);
      console.log(stdout);
      console.log(stderr);

      var res = stdout.split("\n");
      res.splice(-1,1);
      mainWindow.webContents.send('gps_topics',res);
    });
    });

    // ipcMain.on('start_video_server', (event, text) => {
    //   exec("rosrun web_video_server web_video_server", (err, stdout, stderr) => {
    //   //   console.log(err)
    //   console.log(stdout);
    //   // console.log(stderr);
    // })
    // });



    // ipcMain.on('refresh_map', (event, text) => {
    //
    // });

// function rostopic_list_auto(){
//   rostopic_list(function (result) {
//     mainWindow.webContents.send('rostopic_list', result);
//   });
// }

// setInterval(rostopic_list_auto, 3000);

////////////////////////////////////////////////////////////////////////////////

// ros_subscriber();

} //This is the end of main funciton

function test(text){
  console.log(text);
  console.log("End");
}
