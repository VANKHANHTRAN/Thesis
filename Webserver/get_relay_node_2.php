<?php
include_once 'base/constant.php';

$node	 = $_GET['node'];
$motor_1 = $_GET['relay_1'];
$motor_2 = $_GET['relay_2'];

$sql = "SELECT node, relay_1, relay_2 FROM status_device 
WHERE node = '$node', relay_1 = '$motor_1', relay_2 = '$motor_2' 
ORDER BY id DESC LIMIT 1";
$result = mysqli_query($dbconn,$sql);

if($result->num_rows > 0)
{
	while($rows = mysqli_fetch_assoc($result))
	{
		echo $rows["ip"];
	}
}
mysqli_close($dbconn);
?>