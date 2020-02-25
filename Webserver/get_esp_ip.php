<?php
include_once 'base/constant.php';

$sql = "SELECT ip FROM esp_address ORDER BY id DESC LIMIT 1";
$result = mysqli_query($dbconn,$sql);

while($rows = mysqli_fetch_assoc($result))
{
    echo $rows["ip"];
}
mysqli_close($dbconn);
?>