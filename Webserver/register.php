<?php
include_once 'base/constant.php';

$username 	= $_POST['username'];
$password 	= $_POST['password'];
$email 		= $_POST['email'];

$sql1 = $dbconn->query("SELECT username,email FROM users WHERE username = '$username' AND email = '$email'");

if (mysqli_num_rows($sql1) > 0)
{
    echo "Account Already Exist!";
}
else
{
    $sql2 = $dbconn->query("INSERT INTO users(username, password, email) VALUES('$username','$password','$email')");

    if ($sql2)
    {
        echo "Register Completed!";
    }
    else
    {
        echo "Register Failed!";
    }
}
mysqli_close($dbconn);
?>