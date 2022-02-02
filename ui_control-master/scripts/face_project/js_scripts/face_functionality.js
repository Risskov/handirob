import * as asyncMutex from "https://cdn.skypack.dev/async-mutex@0.3.2";

var lookX = 0;
var lookY = 0;

const eyeMutex = new asyncMutex.Mutex();	// Mutes for locking eyes for functions such as wink and blink

function look(lookY_ = lookY, lookX_ = lookX) {
	var pupil = $(".pupil");
	pupil.css({
		transform: "translate(" + lookX_ + "px," + lookY_ + "px)" }); // Moves both pupils equally
}

const randn_bm = (min, max) => {		// Normally distributed random
	var u = 0,
	v = 0;
	while (u === 0) u = Math.random(); //Converting [0,1) to (0,1)
	while (v === 0) v = Math.random();
	let num = Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v);

	num = num / 10.0 + 0.5; // Translate to 0 -> 1
	if (num > 1 || num < 0) num = randn_bm(min, max); // resample between 0 and 1 if out of range
	num *= max - min; // Stretch to fill range
	num += min; // offset to min
	return num;
};

async function wink(shutEye = $("#reye"), shutPupil = $("#rpup")) {
	await eyeMutex.runExclusive(async () => {
		var eye = $(".eye");
		var llid = $(".llid");
		var pupil = $(".pupil");
		eye.css({
			height: "200px",
			top: "50px" });

		llid.css({
			top: "200px" });

		await new Promise(r => setTimeout(r, 100));
		eye.css({
			height: "170px",
			top: "60px" });

		shutEye.css({
			height: "60px",
			top: "140px",
			background: "grey" });

		look(0);
		shutPupil.css({
			background: "grey",
			height: "20px" });

		await new Promise(r => setTimeout(r, 300));
		eye.css({
			height: "300px",
			top: "0px",
			background: "white" });

		llid.css({
			top: "250px" });

		shutPupil.css({
			background: "black",
			height: "70px",
			transform: "translate(" + lookX + "px," + lookY + "px)" });

		look();
		await new Promise(r => setTimeout(r, 300));
	});
}

$("body").click(function (event) {
	var face = $(".face");
	blink(false);
	lookX = (event.pageX - $("body").width() / 2) / $("body").width() * 60;
	lookY = (event.pageY - $("body").height() / 2) / $("body").height() * 100;
	face.css({
		transform: "translate(" + event.pageX + "px," + event.pageY + "px)" });

});

$("body").keypress(function (event) {
	if (event.key == "a") {
		wink();
	}
	if (event.key == "d") {
		wink($("#leye"), $("#lpup"));
	}
	if (event.key == "s") {
		alert("Press detected");
	}
	if (event.key == "w") {
		setMode("worried");
	}
	if (event.key == "q") {
		setMode("angry");
	}
	if (event.key == "e") {
		setMode("normal");
	}
	
});

async function look_around(consistent = false) {
	var face = $(".face");
	do {
		var randX = randn_bm(0, $("body").width());
		var randY = randn_bm(0, $("body").height());
		blink(false);
		lookX = (randX - $("body").width() / 2) / $("body").width() * 60;
		lookY = (randY - $("body").height() / 2) / $("body").height() * 100;
		face.css({
		transform: "translate(" + randX + "px," + randY + "px)" });

		var rando = 1000 + Math.floor(Math.random() * 8000);
		await new Promise(r => setTimeout(r, rando));
	} while (consistent);
}

async function blink(consistent = false) {
	var eye = $(".eye");
	var pupil = $(".pupil");
	do {
		if (consistent) {
			var rando = 600 + Math.floor(Math.random() * 6000);
			await new Promise(r => setTimeout(r, rando));
		}
		await eyeMutex.runExclusive(async () => {
			eye.css({
				top: "150px",
				height: "50px",
				background: "grey" });

			pupil.css({
				transform: "translate(" + lookX + "px," + 10 + "px)",
				height: "30px",
				background: "gray" });

			await new Promise(r => setTimeout(r, 100));
			eye.css({
				top: "0px",
				height: "300px",
				background: "white" });

			pupil.css({
				transform: "translate(" + lookX + "px," + lookY + "px)",
				height: "70px",
				background: "black" });

		});
	} while (consistent);
}

function setMode(mode) {
	var ulid = $(".ulid");	
	if (mode == "worried") {
		ulid.css({
			top: "-180px",
			width: "250px",
			borderRadius: "",
			transform: ""
		});
	}
	else if (mode == "normal"){
		ulid.css({
			top: "",
			width: "",
			borderRadius: "",
			transform: ""
		});
	}
	else if (mode == "angry") {
		var llidu = $("#llidu"); 
		var rlidu = $("#rlidu");
		ulid.css({
			top: "-200px",
			width: "250px",
			borderRadius: "40px"
		});
		llidu.css({
			transform: "rotate(10deg)"
		});
		rlidu.css({
			transform: "rotate(-10deg)"
		});
	}
	else {
		return false;
	}
	return true;
}

export { look_around, blink, setMode };
console.log("Face_functionality.js loaded ok");