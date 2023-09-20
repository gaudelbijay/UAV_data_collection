#!/usr/bin/expect -f

set timeout 20
set password "oelinux123"

spawn ssh root@172.16.0.33
# expect "password:"
# send "$password\r"
expect "# "
send "cd /\r"
expect "# "
send "source /opt/ros/melodic/setup.sh\r"

expect "# "
send "roslaunch mavros px4.launch fcu_url:=\"udp://:14540@127.0.0.1:14557\"\r"
interact
