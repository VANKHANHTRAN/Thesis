<?php
include_once '../base/constant.php';
$temp= $humi = $soil = $relay_1 = $relay_2 = '';

if(isset($_GET['temp'])){
    $temp = $_GET['temp'];  
}

if(isset($_GET['humi'])){
    $humi = $_GET['humi'];
}

if(isset($_GET['soil'])){
    $soil = $_GET['soil'];
}

if(isset($_GET['relay_1'])){
    $relay_1 = $_GET['relay_1'];
}

if(isset($_GET['relay_2'])){
    $relay_2 = $_GET['relay_2'];
}

$query = "INSERT INTO node_1(temp, humi, soil, relay_1, relay_2) VALUES ('$temp' , '$humi' , '$soil' , '$relay_1' , '$relay_2')";

$result = mysqli_query($dbconn, $query);  
echo $query;
?>
