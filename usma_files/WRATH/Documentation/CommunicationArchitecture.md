# **Communications**

In the drone swarming architecture, we have two main types of communication: drone-drone and drone-server. To make it easier to modify and handle messages being sent, we created a new communication infrastructure that combines drone-drone and drone-server messaging into one. When you send a network message, the system will send that message to everyone. If a network object like the drone or the server decides that they care about a message, then they will register it as a message of importance. Then, anytime that network device receives a message, it will be alerted. This alert occurs through a callback. If a drone cares about a message, it will register a callback with the message handler. This system uses a custom messaging protocol that will be discussed below.

## **Description**

### **Objects**

* **Swarm Communications**: This is the top level object that handles all communications for a drone. It will process messages from another drone and the server, as well as send messages to both.
* **Server** / **Client**: These two classes handle messages that get sent between the drone and server. When either of these receive bytes over a network, they will determine if they have a complete message. If they do, they will send all of the bytes to the Message Handler to process. If not, they will store the bytes and wait until they have a complete message. The main difference between them is that the server needs to be able to send messages to all clients, while the client only sends messages to the server. Because of this, the server uses a broadcast function that will send the message to all clients.
* **Message Handler**: This class will process a message, determine if it is valid, and if the network device cares about the message. If it does, then the message handler will notify the network device, and if not, it will drop the message. 
* **Message**: While there is no base message class, every message that is created will need to adhere to three requirements.
    * Unique integer ID so it can be distinguished between other messages. 
    * Pack function that takes no parameters and will convert required data to bytes based on a format.
    * Unpack function that takes bytes and will convert those bytes to data. 

### **Message Protocol**
Our messaging protocol has two layers of encapsulation that allow it to handle a variety of custom messages. When you write your own messages, you will only need to worry about layer 2.
* **Layer 1**: This layer is in charge of determining if a message is complete. It is handled in the server and client. Layer of encapsulation has a 4 byte header which stores the number of bytes in the message. When a server or client sends a message, they create the header and attach it to the message before sending it out. When the server or client receives a message, they check the header to determine if they have a complete message. If they do, they strip the header and send it to the message handler.
* **Layer 2**: This layer handles the actual messages and is processed in the Message Handler. This layer has a 4 byte header which stores the type of message. Each message type has a specified format that the remaining bytes must adhere to. For example, if a message's format contains 2 integers, then the message must contain a 4 byte header that stores the type and 8 more bytes that store the data in the message. 

### **Functionality**

This section will discuss the flow of control for various activities.

* **Sending Drone Messages**: To send a message, the behavior will construct a valid message and send it to the swarm communications which will then send it to other drones, using the built-in drone messaging system, as well as the client, which will then send it to the server. An example pseudo stack trace / pseudo code can be seen below
    ```python
    -- SwarmCommunications.sendMessage(msg = LogMessage("This is a log"))
        # Send message to drones
        -- DronePublisher.publish(droneMsg = DroneMessage(msg.id, msg.pack()))

        # Send message to server
        -- Client.sendMessage(msg)
          -- bytes = msg.pack() # this adds the layer 2 header

          # Construct layer 1 header
          -- bytes = len(bytes) + bytes
          -- socket.sendall(bytes)
    ```

* **Sending Server Messages**: To send a message, the server will construct a message and then broadcast it. The process is the same as above, however it doesn't use swarm communications and it will not use the drone publisher. It will still create the layer 2 header with msg.pack() and will construct the layer 1 header by getting the length of the bytes. The only difference is that it broadcasts to each connection.

* **Receiving a Drone-Drone Message**: When the drone receives a message from another drone, it will automatically process the message and call a method in our SwarmCommunications class. One this processDroneMsg method is called, the drone comms will then send this message to the message handler. The message handler will strip the layer 2 header and check if the behavior cares about a message. If it does, then the message handler will call a method in the behavior. If not, it drops the message.

* **Receiving a Drone-Server Message**: This section will cover bother Drone->Server messages and Server->Drone messages since they are handled the same. When a message is received, the layer 1 header is stripped, and then the remaining bytes are sent to the message handler. The message handler strips the layer 2 header and converts the bytes into a valid message. If the Server/Drone has registered a callback with the MessageHandler, then it will call that method. Otherwise, the message gets dropped. 

### **Usage and Extending**

This system is designed to be used and extended, so this section will teach you how to do that. 

* **Sending a Message from the Drone**: All you need to do is create a message, fill the data, and call the sendMessage method on the DroneCommunications object.
    ```python
    message = msgs.RadiationMessage()
    message.location = (lat, lon)
    message.count = count
    self.comms.sendMessage(message)
    ```
* **Sending a Message from the Server**: Pretty much the same as the drone, but now you need to broadcast the message.
    ```python
    message = msgs.StartLaneGenerationMessage()
    message.contourPoints = self.contourPoints
    self.broadcast(message)
    ``` 
* **Subscribe to a Message**: You need to first create a function that has the following signature:
    ```python
    def myMessageCallback(self, msg):
        # Do stuff with my msg
        pass
    ```
    Then you need to register this callback.
    ```python
    # This is how you do it in the drone
    self.comms.registerMessageCallback(MyMessage.id, self.myMessageCallback)

    # This is how you do it in the server
    self.registerMessageCallback(MyMessage.id, self.myMessageCallback)
    ``` 

* **Creating a Custom Message**: When you create a new message, you need to do two things. First, you need to make the message class, then you need to add it to the MessageHandler in both the Server and Swarm repositories. When making a message, it must have an `id` and a `pack` and `unpack` method. For this project, we used the `struct` module to convert data to bytes, but you could do it yourself, if you're feeling masochistic. As a note, the first two characters in the struct format must be '!l'. The ! means it's going over the network, and the l means that the first 4 bytes are an integer. This is required because the layer 2 header specifies that the first 4 bytes of the message must be the unique id of the message, which is an int
```python
import struct

class MyMessage(object): # this is python 2, so you need object

    id = 137 # unique id
    fmt = "!ll" # look up documentation on formats, but here's what this one means
        # !: this is being stored in network byte order
        # l: 4 byte integer
        # putting ll together, means that the first 8 bytes are 2 integers

    def __init__(self):
        self.myData = 7

    def pack(self):
        return struct.pack(type(self).fmt, type(self).id, self.myData)
    
    def unpack(self, bytes):
        id, self.myData = struct.unpack(type(self).fmt, bytes)
``` 

### **Improvements**
The most pressing improcement relates to how the server handles connections. Right now, the server creates a new thread for each connection, but this isn't great if there are more that 10 drones connected. This should probably be fixed by using select connections.