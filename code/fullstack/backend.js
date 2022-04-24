var app = require('express')();
var fs = require("fs");
const path = require("path");
var dgram = require("dgram");

// Predefine 
var payload = "";
var data = []; // Array to store payload data from UDP datagram
var buttonToggle;

// Setting up server
var PORT = 3000;
var HOST = '192.168.1.112';
var server = dgram.createSocket('udp4');

app.get('/', (req,res) => {
    res.sendFile(__dirname + '/frontend.html')
})

app.get('/api', async (req, res) => {
    console.log(req.query.q);
    buttonToggle = req.query.q
    res.send("ok")
});

server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

var msgResponse = "OFF"
server.on('message', function (message, remote) {

    if (buttonToggle == "OFF") {
        msgResponse="OFF";
    } else { msgResponse="ON"}

    server.send(msgResponse, 0, msgResponse.length, remote.port, remote.address, function(err, bytes) {
        if (err) throw err;
        console.log('UDP server message sent to ' + remote.address +':'+ remote.port);
    });

});

server.bind(PORT, HOST);

app.listen(42069, () => {
    console.log("go to http://localhost:42069/");
});
