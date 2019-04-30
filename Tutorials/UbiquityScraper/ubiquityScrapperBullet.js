function ray_func(){
  var a = $("#sta_list")[0].innerHTML;
  var b = a.split("</td>");
  var indexes = [];
  //var span_index = b[3].indexOf(">")+1;
  //b[3] = b[3].slice(span_index, b[3].lenght);
  indexes.push(b[2].indexOf(">")+1);
  indexes.push(b[2].length);
  indexes.push(b[3].indexOf(">")+1);
  indexes.push(b[3].length);
  indexes.push(b[6].indexOf(">")+1);
  indexes.push(b[6].length);
  indexes.push(b[7].indexOf(">")+1);
  indexes.push(b[7].length);
  //indexes.push(b[7].indexOf(">")+1);
  //indexes.push(b[7].indexOf("<span"));
  indexes.push(b[4].indexOf(">")+1);
  indexes.push(b[4].length);
  indexes.push(b[9].indexOf(">")+1);
  indexes.push(b[9].length);
  var signal = b[2].slice(indexes[0], indexes[1]);
  var remote_signal = b[3].slice(indexes[2], indexes[3]);
  var distance = b[6].slice(indexes[4], indexes[5]);
  var download_upload = b[7].slice(indexes[6], indexes[7]);
  //var upload = b[7].slice(indexes[8], indexes[9]);
  var noise = b[4].slice(indexes[8], indexes[9]);
  var conn_time = b[9].slice(indexes[10], indexes[11]);
  var today = new Date();
  var comp_time = today.getHours() + ":" + today.getMinutes() + ":" + today.getSeconds();
  
  console.log(comp_time , "," , signal, "," , remote_signal, "," , distance, "," ,download_upload, ",", noise, ",", conn_time);
    
  if(global_indicator == 0){
    var refresh = $("#_refresh").click()
    setTimeout(ray_func, 2000);
  }
}