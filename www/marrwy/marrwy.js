var twist;
var cmdVel;
var publishImmidiately = true;
var robot_IP;
var manager;
var teleop;
var ros;

var battery_status;
var loadfoodClient;

robot_IP = location.hostname;

ros = new ROSLIB.Ros({
     url: "ws://" + robot_IP + ":9090"
 });

ros.on('connection', function () {
  document.getElementById("status").innerHTML = "Connected";
});

ros.on('error', function (error) {
  document.getElementById("status").innerHTML = "Error";
});

ros.on('close', function () {
  document.getElementById("status").innerHTML = "Closed";
});
// 
 //* A topic for messaging.
 var gototableTopic = new ROSLIB.Topic({
    ros: ros,
    name : '/nro_table',
    messageType: 'std_msgs/String'
  });

function gototavolo(tavolo){

    var mymsg = new ROSLIB.Message({
        text: tavolo
    });
    
    console.log(' Vado al tavolo :');
    console.log(tavolo);  
    
    gototableTopic.publish(mymsg);  
}
function moveAction(linear, angular) {
    if (linear !== undefined && angular !== undefined) {
        twist.linear.x = linear;
        twist.angular.z = angular;
    } else {
        twist.linear.x = 0;
        twist.angular.z = 0;
    }
    cmdVel.publish(twist);
}
   //* A topic for messaging.
   var emotionTopic = new ROSLIB.Topic({
    ros: ros,
    name : '/nro_table',
    messageType: 'std_msgs/String'
  });

function FaceExpression( face){
    var mymsg = new ROSLIB.Message({
         data :  face   
    });
    emotionTopic.publish(mymsg); // error here als
  
    console.log(face);   
  }
  
  
function initVelocityPublisher() {
    // Init message with zero values.
    twist = new ROSLIB.Message({
        linear: {
            x: 0,
            y: 0,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: 0
        }
    });
    // Init topic object
    cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel_mux/input/teleop',
        messageType: 'geometry_msgs/Twist'
    });
    // Register publisher within ROS system
    cmdVel.advertise();
}


function initTeleopKeyboard() {
    // Use w, s, a, d keys to drive your robot

    // Check if keyboard controller was aready created
    if (teleop == null) {
        // Initialize the teleop.
        teleop = new KEYBOARDTELEOP.Teleop({
            ros: ros,
            topic: '/cmd_vel_mux/input/teleop'
        });
    }

    // Add event listener for slider moves
    robotSpeedRange = document.getElementById("robot-speed");
    robotSpeedRange.oninput = function () {
        teleop.scale = robotSpeedRange.value / 100
    }
}


function createJoystick() {
    // Check if joystick was aready created
    if (manager == null) {
        joystickContainer = document.getElementById('joystick');
        // joystck configuration, if you want to adjust joystick, refer to:
        // https://yoannmoinet.github.io/nipplejs/
        var options = {
            zone: joystickContainer,
            position: { left: 50 + '%', top: 105 + 'px' },
            mode: 'static',
            size: 200,
            color: '#0066ff',
            restJoystick: true
        };
        manager = nipplejs.create(options);
        // event listener for joystick move
        manager.on('move', function (evt, nipple) {
            // nipplejs returns direction is screen coordiantes
            // we need to rotate it, that dragging towards screen top will move robot forward
            var direction = nipple.angle.degree - 90;
            if (direction > 180) {
                direction = -(450 - nipple.angle.degree);
            }
            // convert angles to radians and scale linear and angular speed
            // adjust if youwant robot to drvie faster or slower
            var lin = Math.cos(direction / 57.29) * nipple.distance * 0.005;
            var ang = Math.sin(direction / 57.29) * nipple.distance * 0.05;
            // nipplejs is triggering events when joystic moves each pixel
            // we need delay between consecutive messege publications to 
            // prevent system from being flooded by messages
            // events triggered earlier than 50ms after last publication will be dropped 
            if (publishImmidiately) {
                publishImmidiately = false;
                moveAction(lin, ang);
                setTimeout(function () {
                    publishImmidiately = true;
                }, 50);
            }
        });
        // event litener for joystick release, always send stop message
        manager.on('end', function () {
            moveAction(0, 0);
        });
    }
}

// Setup battery status
// Subscribing to a Topic
function setBattery(battery_status) {
    console.log('SetBattery Started');
    


    var listener = new ROSLIB.Topic({
        ros : ros,
        name : '/battery_charge',
        messageType : 'std_msgs/Float32'
    });

    listener.subscribe(function(message) {
    //console.log('Received message on ' + listener.name + ': ' + message.data);
    battery_status.style = "width:"+message.data+"%";
    battery_status.innerHTML = message.data + '%';

    var n = parseInt(message.data);

    if (10 < n && n < 20){
        console.log("Warning Battery Low");
    }else if (parseFloat(message.data) < 10){
        console.log("Battery Empty");
    }else{
        console.log("Battery Ok");
    }


    });

    console.log('SetBattery Finished');
}






function setMoveToTableBtn() {
    console.log('Move To Table Initialised');
    
    // 
    
    $(".btn-table").click(function(event2){
        // Holds the product ID of the clicked element
        event2.preventDefault(); // To prevent following the link (optional)
        var tableId = this.id;
        var res = tableId.split("-");
        var table_number = res[1];
        console.log(res[1]);
        var tableinnerHTML = this.innerHTML;
        console.log("Generic Table ID===>"+tableId);
        console.log("Generic Table innerHTML===>"+tableinnerHTML);

        table_btn = document.getElementById(tableId);
        event2.preventDefault(); // To prevent following the link (optional)
        //if (table_btn.innerHTML == "T1"){
        if ($(this).hasClass( 'active' )){
            console.log('MOVING to Table='+table_number);     
            
        }else{
            table_btn.innerHTML = "VADO AL TAVOLO T"+table_number;
            $(this).toggleClass('active');
            $(table_btn).removeClass("btn-success");
            $(table_btn).addClass("btn-warning");
            gototavolo(res[1]);
            //MoveToTable(parseInt(table_number));
            
        }
        
      });

}

// Calling action
function MoveToTable(table_number){
  
    $( "#table-cancel-btn" ).click(function(event3) {
        console.log('Cancel Move Clicked');
        
            
        });
    


}


  


window.onload = function () {
    
    /*
    initVelocityPublisher();
    // get handle for video placeholder
    video = document.getElementById('robot-image');
    // Populate video source 
    //video.src = "http://" + robot_IP + ":8080/stream?topic=/pan_and_tilt/main_cam/image_raw&type=mjpeg&quality=80";
    video.src = "images/barista.png";
    video.onload = function () {
        // joystick and keyboard controls will be available only when video is correctly loaded
        createJoystick();
        initTeleopKeyboard();
    };
    */
    /*
    // get handle for video placeholder
    battery_status = document.getElementById('battery-status');
    battery_status.style = "width:10%";
    battery_status.innerHTML = '10%';
    setBattery(battery_status);
    */
    
    //loadfood = document.getElementById('loadfood-btn');
    //setLoadFoodBtn();
    //setResetFoodBtn();

    // Start the Move to table action client
    setMoveToTableBtn();
    
    
}