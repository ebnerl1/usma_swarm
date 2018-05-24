# usma_swarm Data Streaming Between Vehicles & Basestation

1. Much of the code from both the waypoint server and waypoint client files originates from https://pymotw.com/2/socket/tcp.html

2. Install the waypoint server on the base sensor station. IT IS CRITICAL THAT THE BASE SENSOR STATION IP ADDRESS IS 192.168.11.203. Both the waypoint server and waypoint client is configured to connect to this server address. The intent behind this was to separate the base sensor station from the base control station. That being said, the script automatically populates photos from the vehicles and saves them in a directory, so the server could still be on one of the base control stations.

* The following directory must exist: "~/home/odroid_images/" This is where all of the photos are stored. I also suggest keeping the waypoint server python script in that directory. To run the script, simply enter:
`python ./waypoint_client.py`

* After running this command, you should see text appear on the screen that describes what is going on. You should see "waiting for connection," otherwise, you should begin to troubleshoot what is going on.

3. Install the waypoint client on the Odroid. I created a "servers" directory in the home folder to do this. While you have to manually run the script now to establish a connection with the server, the end state is to include a script call within a swarm behavior to automatically connect to the waypoint server once a vehicle arrives at a waypoint.

* To run the script, simply enter the command:
`python ./waypoint_client.py`

4. Now, your server and client should have established a connection. You should be able to visualize this on either the server or client terminal. On the server terminal, you will see "message received from ..." "sending updated waypoint list to..." "no more messages from..." and then the server should go back to waiting for more connections. On the client, you will see "sending..." "received..." "closing socket". 

5. For now, the data that the waypoint_client is sending is hard coded within the "message" variable. Once we include this python script in a behavior, the behavior will generate a similar string variable in the following format:
`"quad_number waypoint_number"`

* This assumes that we are also referring to our waypoints only by integer values since we can then cross-reference the integer-valued waypoint to a LAT/LON/ALT. 

6. For now, the waypoint list is hard coded into the waypoint server under the "waypoints" array. In the future, the basestation will receive a list of all the waypoints that the vehicles need to visit and keep a running list of this while the vehicles are in action. Every time a vehicle sends back that it has completed a waypoint, the waypoint server removes it from the list of waypoints that the vehicles need to travel to AND triggers the vehicle that visited the waypoint to take a picture and send it back. The server saves this picture in the `~/home/odroid_images` directory with the file name: `waypoint_num-Date-Time-Group.jpeg`. 

* I modified the snap-odroid-image.sh script so that you have to provide two arguments in order to run it. The command is as follows:
`./snap-odroid-image.sh quad_num waypoint_num`
