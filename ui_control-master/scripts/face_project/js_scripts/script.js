//var url_ = "ws://192.168.87.18:9090"; 
var url_ = "ws://localhost:9090"
var url_ = "ws://0.0.0.0:9090"

var	ros	= new ROSLIB.Ros({
	url:  url_ });

ros.on("connection", function () {
	console.log("Connected to websocket	server.");
});

ros.on("error",	function (error) {
	console.log("Error connecting to websocket server: ", error);
});

ros.on("close",	function () {
	console.log("Connection	to websocket server	closed.");
});

// Subscribing to a	Topic
// ----------------------

var	listener = new ROSLIB.Topic({
	ros: ros,
	name:	"/ui/look",
	messageType: "std_msgs/String" });


listener.subscribe(function	(message) {
	console.log("Received	message	on " + listener.name + ": "	+ message.data);
});

// Creating a service 
// ----------------------
// The Service object does double duty for both calling and advertising services
var setBoolServer = new ROSLIB.Service({
	ros : ros,
	name : '/ui_control/setmode',
	serviceType : '/ui_control/ui_mode'
});

// Use the advertise() method to indicate that we want to provide this service
setBoolServer.advertise(function(request, response) {
	console.log('Received service request on ' + setBoolServer.name + ': ' + request.mode);
	response['response'] = 'Set successfully';
	setMode(request.mode)
	// var ulid = $(".ulid");	
	// if (request.mode == "worried") {
	// 	ulid.css({
	// 		top: "-180px",
	// 		width: "250px",
	// 		borderRadius: "",
	// 		transform: ""
	// 	});
	// }
	// else if (request.mode == "normal"){
	// 	ulid.css({
	// 		top: "",
	// 		width: "",
	// 		borderRadius: "",
	// 		transform: ""
	// 	});
	// }
	// else if (request.mode == "angry") {
	// 	var llidu = $("#llidu"); 
	// 	var rlidu = $("#rlidu");
	// 	ulid.css({
	// 		top: "-200px",
	// 		width: "250px",
	// 		borderRadius: "40px"
	// 	});
	// 	llidu.css({
	// 		transform: "rotate(10deg)"
	// 	});
	// 	rlidu.css({
	// 		transform: "rotate(-10deg)"
	// 	});
	// }
	return true;
});

//import { look_around,	blink }	from './face_functionality.js';
import * as face_functionality from	'./face_functionality.js';
face_functionality.look_around(true);
face_functionality.blink(true);

console.log("Script.js loaded ok");
