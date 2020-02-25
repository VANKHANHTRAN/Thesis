<?php
include_once 'base/constant.php';

$username = $_POST['username'];
$password = $_POST['password'];

$sql = "SELECT username,password FROM users WHERE username = '$username' AND password = '$password'";
$result = $dbconn->query($sql);
if (mysqli_num_rows($result) > 0)
{
	echo "Log in OK!";
}
else
{
	echo "Log in ERROR!";
}
mysqli_close($dbconn);
?>