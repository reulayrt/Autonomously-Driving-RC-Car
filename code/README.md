# Code Readme

## Folder Structure
<pre>
code/
├── README.md <-- You're here
└── fullstack/
    ├── frontend.html
    ├── backend.js
└── crawler/
    └── ...
    └── main/
        └── udp_client.c
    └── ...
</pre>


## fullstack/
The fullstack folder contains all the code that runs the website that includes the ON and OFF toggle for the crawler. It opens up a website that can be accessed through various devices, which can be used to turn the crawler on and off!

### frontend.html
The frontend html uses ajax to communicate to the backend. The fetch changes the content of /api of the server which allows the backend to know what the toggle is switched to. When the toggle is red and says off, api says off, and if the toggle is blue and says on, api says on. This frontend is started with the backend.js.

### backend.js
The backend is the server that runs the frontend and allows the entire toggle to work. First, using Express and frontend.html, the website is created at localhost. then it does a appget to constantly get the value at the at /api. Then, it sets up a UDP connection with the ESP32 that keeps a constant communication between the ESP and the computer. The esp sends a single byte and the server in return sends the value of the toggle back to the esp. 

### Resources Used
- AJAX Package
- Express Package
- dgram Package

## crawler/
The driver code and build for the crawler is found within this folder. The code is all written in the "udp_client.c" file. 

### udp_client.c
In this file, you will find the code that controls the wheel servo, PID cruise control, the steering servo, PID for steering control, udp client code, and several more helper functions that we used to create our solution design.

### Resources Used
- Cluster 5
- Provided links in quest + cluster