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
</head>
<body>
	<table>
		<tr>
			<th>Nhiệt độ</th>
			<th>Độ ẩm</th>
			<th>Độ ẩm đất</th>
			<th>Động cơ</th>
			<th>Đèn</th>
			<th>Thời gian</th>
		</tr>
		<tr>
			<?php
			include_once '../base/constant.php';
			$sql = "SELECT * FROM node_3";
			$result = $dbconn->query($sql);
			if ($result->num_rows > 0) 
			{
				// output data of each row
				while($row = $result->fetch_assoc()) {
				echo "<tr><td>" . $row["temp"]. "</td><td>" . $row["humi"] . "</td><td>"
				. $row["soil"]. "</td><td>" . $row["relay_1"] . "</td><td>" . $row["relay_2"] . "</td><td>" . $row["date"] . "</td></tr>";
			}
				echo "</table>";
			} 
			else { echo "0 results"; }
			$dbconn->close();
			?>
		</th>
	</table>
</body>
</html>