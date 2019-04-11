function ray_func(){
  var a = $("#station_list_table_row_FCECDADCCE8E")[0].innerHTML;
  var b = a.split("</td>");
  var indexes = [];
  var span_index = b[3].indexOf(">")+1;
  b[3] = b[3].slice(span_index, b[3].lenght);
  indexes.push(b[3].indexOf(">")+1);
  indexes.push(b[3].indexOf("dBm"));
  indexes.push(b[4].indexOf(">")+1);
  indexes.push(b[4].indexOf("dBm"));
  indexes.push(b[5].indexOf(">")+1);
  indexes.push(b[5].indexOf("miles"));
  indexes.push(b[6].indexOf(">")+1);
  indexes.push(b[6].indexOf("<span"));
  indexes.push(b[7].indexOf(">")+1);
  indexes.push(b[7].indexOf("<span"));
  indexes.push(b[10].indexOf(">")+1);
  indexes.push(b[10].length);
  var signal = b[3].slice(indexes[0], indexes[1]);
  var remote_signal = b[4].slice(indexes[2], indexes[3]);
  var distance = b[5].slice(indexes[4], indexes[5]);
  var download = b[6].slice(indexes[6], indexes[7]);
  var upload = b[7].slice(indexes[8], indexes[9]);
  var conn_time = b[10].slice(indexes[10], indexes[11]);
  var today = new Date();
  var comp_time = today.getHours() + ":" + today.getMinutes() + ":" + today.getSeconds();

  console.log(comp_time , "," , signal, "," , remote_signal, "," , distance, "," ,download, ",", upload, ",",conn_time);

  if(global_indicator == 0){
    setTimeout(ray_func, 2000);
  }
}
