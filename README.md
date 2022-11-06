# marrtino_r3d
# MARRTINO implementazioni
# by Ferrarini Fabio - ROBOTICS-3D.com




## 1 - GESTIONE DELLA NAVIGAZIONE TRAMITE WAYPOINT
    
###   Creazione dei waypoint 
    
      PROGRAMMA PER LA CREAZIONE DEI WAYPOINT 

      TASTO A per MEMORIZZARE WAYPOINT
      TASTO Y per SALVARE

      TASTO X PER INDICARE CHE E' ARRIVATO AL TAVOLO 

      python createwaypoint_v1.py /home/ubuntu/src/waypoint_navigation/waypoint/counter_fwd1.csv

       
###   Navigazione tramite waypoint
      

      python movexy_v1.py /home/ubuntu/src/waypoint_navigation/waypoint/waypoint01.csv

      Parametri consigliati:

      VEL_ANGOLARE = 0.3
      VEL_LINEARE = 0.4
      ANGLE_TOLERANCE  = 20
      DISTANCE_TOLERANCE  = 0.35 #
        
      COEFF_VEL_ANGOLARE = 1
      COEFF_VEL_LINEARE = 0.1



##    /ready
      Una volta arrivato al tavolo attende "OK" per proseguire 

      rostopic pub -1 /ready std_msgs/String "OK"

##    
      cd $HOME/src
      git clone https://github.com/tork-a/roswww.git
      cd $HOME/ros/catkin_make/src
      ln -s $HOME/src/roswww .
      cd ..
      catkin_make
 
## www


# sudo ln -s /home/ubuntu/src/waypoint_navigation/www/marrwy

   
/nro_table 

rostopic pub -1 /nro_table std_msgs/String "data: '01'

/ready
 
rostopic pub -1 /ready std_msgs/String "data: 'OK'"
