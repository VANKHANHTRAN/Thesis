<?php
$DB_HOST = "localhost";
$DB_USER = "root"; //"tvkhanh";
$DB_PASS = "";  //"24121996";
$DB_NAME = "system";

$dbconn = new MySQLi("$DB_HOST","$DB_USER","$DB_PASS","$DB_NAME");

if (!$dbconn)
{
    echo "Connection Error";
	exit();
}
?>