rostopic pub -1 /go std_msgs/String "data: 'START;13;19;0;testcasa'" 
sleep 3
rostopic pub -1 /go std_msgs/String "data: 'GO;13;21.9;0;testcasa'" 
sleep 3
rostopic pub -1 /go std_msgs/String "data: 'GO;14.9;21.9;0;testcasa'" 
sleep 3
rostopic pub -1 /go std_msgs/String "data: 'GO;12.9;21.9;90;testcasa'" 
sleep 3