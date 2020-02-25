<?php
include_once 'base/constant.php';

$node	 = $_GET['node'];

$sql = "SELECT relay_1, relay_2 FROM status_device 
WHERE node = '$node' LIMIT 1";
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