<?php
include_once 'base/constant.php';

$ip = '';

if (isset($_GET['ip'])
{
    $ip = $_GET['ip'];
}

$sql = "INSERT INTO esp_address(ip) VALUES ('$ip')";
$result = mysqli_query($dbconn, $sql);
echo $ip;
mysqli_close($dbconn);
?>