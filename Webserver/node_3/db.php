<?php
include_once('../base/constant.php');
error_reporting(0);
$id = "01";
//define variables and set to empty values
$node_id = $temp = $humi = $soil = $relay_1 = $relay_2 = "";

if ($_SERVER["REQUEST_METHOD"] == "POST")
{
	$temp = test_input($_POST["temp"]);
	$humi = test_input($_POST["humi"]);
	$soil = test_input($_POST["soil"]);
	$relay_1 = test_input($_POST["relay_1"]);
	$relay_2 = test_input($_POST["relay_2"]);
	//Create conenction
	$conn = mysqli_connect(DB_HOST, DB_USER, DB_PASS, DB_NAME);
	//Check connection
	if (!$conn)
	{
		die('Connection failed: ' . mysqli_connect_error());
	}
	$sql = "INSERT INTO node_1(temp, humi, soil, relay_1, relay_2)
	VALUES('" . $temp . "','" . $humi ."', '" . $soil ."','" . $relay_1 . "',
	'" . $relay_2 . "')";
	if($conn->query($sql) === TRUE)
	{
		echo "New record is created successfully";
	}
	else
	{
		echo "Error: " . $sql . "<br>" . $conn->error;
	}
	$conn->close();
}
else
{
	echo "No data posted with HTTP POST";
}

function test_input($data)
{
	$data = trim($data);
	$data = stripslashes($data);
	$data = htmlspecialchars($data);
	return $data;
}
?>