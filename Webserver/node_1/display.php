<?php
//fetch data and display it into json format
$connect = mysqli_connect("localhost", "root", "", "system");
$query = '
SELECT temp, humi, soil, 
UNIX_TIMESTAMP(CONCAT_WS(" ", date, time)) AS datetime 
FROM node_1 
ORDER BY date DESC, time DESC
';
$result = mysqli_query($connect, $query);
$rows = array();
$table = array();

$table['cols'] = array(
 array(
  'label' => 'Date Time', 
  'type' => 'datetime'
 ),
 array(
  'label' => 'Temperature (°C)', 
  'type' => 'number'
 ),
 array(
  'label' => 'Humidity (%)', 
  'type' => 'number'
 ),
 array(
  'label' => 'Soil Moisture (%)', 
  'type' => 'number'
 )
);

while($row = mysqli_fetch_array($result))
{
 $sub_array = array();
 $datetime = explode(".", $row["datetime"]);
 $sub_array[] =  array(
      "v" => 'Date(' . $datetime[0] . '000)'
     );
 $sub_array[] =  array(
      "v" => $row["temp"]
     );
 $sub_array[] =  array(
      "v" => $row["humi"]
     );
 $sub_array[] =  array(
      "v" => $row["soil"]
     );
 $rows[] =  array(
     "c" => $sub_array
    );
}
$table['rows'] = $rows;
$jsonTable = json_encode($table);

?>

<!DOCTYPE html>
<html>
<head>
	<title>Data From Database</title>
	<style>
		table
		{
			border-collapse: collapse;
			width: 100%;
			color: #A9A9A9;
			font-family: monospace;
			font-size: 25px;
			text-align: center;
		}
		th
		{
			background-color #A9A9A9;
			color: black;
		}
		tr: nth-child(even)
		{
			background-color:#F2F2F2;
		}
	</style>
	<script type="text/javascript" src="https://www.gstatic.com/charts/loader.js"></script>
  <script type="text/javascript" src="//ajax.googleapis.com/ajax/libs/jquery/1.10.2/jquery.min.js"></script>
  <script type="text/javascript">
   google.charts.load('current', {'packages':['corechart']});
   google.charts.setOnLoadCallback(drawChart);
   function drawChart()
   {
    var data = new google.visualization.DataTable(<?php echo $jsonTable; ?>);

    var options = {
     title:'Sensors Data',
     legend:{position:'bottom'},
     chartArea:{width:'95%', height:'65%'},
	 colors:['red','yellow','green'],
    };

    var chart = new google.visualization.LineChart(document.getElementById('line_chart'));

    chart.draw(data, options);
   }
  </script>
  <style>
  .page-wrapper
  {
   width:1000px;
   margin:0 auto;
  }
  </style>
</head>
<body>
	<table>
		<tr>
			<th>Nhiệt độ</th>
			<th>Độ ẩm</th>
			<th>Độ ẩm đất</th>
			<th>Máy bơm</th>
			<th>Đèn</th>
			<th>Thời gian</th>
		</tr>
		<tr>
			<?php
			include_once '../base/constant.php';
			$sql = "SELECT * FROM node_1 ORDER BY id DESC LIMIT 10";
			$result = $dbconn->query($sql);
			if ($result->num_rows > 0) 
			{
				// output data of each row
				while($row = $result->fetch_assoc()) 
				{
					echo "<tr><td>" . $row["temp"]. "</td><td>" . $row["humi"] . "</td><td>"
					. $row["soil"]. "</td><td>" . $row["relay_1"] . "</td><td>" . $row["relay_2"] . "</td><td>" . $row["date"] . " " . $row["time"] . "</td></tr>";
				}
				echo "</table>";
			} 
			else 
			{
				echo "0 results"; 
			}
			$dbconn->close();
			?>
		</th>
	</table>
	<div class="page-wrapper">
   <br />
   <h2 align="center">Sensor Chart</h2>
   <div id="line_chart" style="width: 100%; height: 500px"></div>
  </div>
</body>
</html>