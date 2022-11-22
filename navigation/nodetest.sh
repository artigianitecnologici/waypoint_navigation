rostopic pub -1 /go std_msgs/String "data: 'START;0.70;0.0;0;testcasa'"
sleep 8
rostopic pub -1 /go std_msgs/String "data: 'GO;0.70;-1.0;0;testcasa'"
sleep 8
rostopic pub -1 /go std_msgs/String "data: 'GO;0.70;0.0;1,5708;testcasa'"
sleep 8
rostopic pub -1 /go std_msgs/String "data: 'GO;1.40;-1.0;0;testcasa'"
sleep 8
rostopic pub -1 /go std_msgs/String "data: 'GO;1.40;-1.4;0;testcasa'"
sleep 8
rostopic pub -1 /go std_msgs/String "data: 'GO;2.10;-1.4;0;testcasa'"
sleep 8
rostopic pub -1 /go std_msgs/String "data: 'GO;1.40;-1.0;0;testcasa'"
sleep 8
rostopic pub -1 /go std_msgs/String "data: 'GO;1.40;0.0;0;testcasa'"
sleep 8
rostopic pub -1 /go std_msgs/String "data: 'GO;0.60;0.0;0;testcasa'"
sleep 8
rostopic pub -1 /go std_msgs/String "data: 'GO;1.40;-1.0;0;testcasa'"
sleep 8
rostopic pub -1 /go std_msgs/String "data: 'GO;0.60;-1.0;0;testcasa'"
sleep 8
rostopic pub -1 /go std_msgs/String "data: 'GO;0.60;-0.0;0;testcasa'"