<?php
include_once 'base/constant.php';

$node	 = $_POST['node'];
$relay_1 = $_POST['relay_1'];
$relay_2 = $_POST['relay_2'];

$sql = $dbconn->query("INSERT INTO status_device(node,relay_1, relay_2) VALUES('$node', 'relay_1', 'relay_2')");
mysqli_close($dbconn);
?>