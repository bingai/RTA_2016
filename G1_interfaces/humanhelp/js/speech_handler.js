var lang = 'en-GB';
var final_transcript = '';
var recognizing = false;
var ignore_onend;
var start_timestamp;

var json_parse = '';
var action, food, duration1, duration2, unit1, unit2;
var utensil, utensil_specifier, location_id;
var understand = "";

if (!('webkitSpeechRecognition' in window)) {
	console.log("UPDATE YOUR BROWSER");
} else {
	var recognition = new webkitSpeechRecognition();
	recognition.continuous = true;
	// recognition.interimResults = true;

	recognition.onstart = function() {
		recognizing = true;
	};

	recognition.onerror = function(event) {
		if (event.error == 'no-speech') {
			ignore_onend = true;
		}
		if (event.error == 'audio-capture') {
			ignore_onend = true;
		}
		if (event.error == 'not-allowed') {
			if (event.timeStamp - start_timestamp < 100) {
				console.log("BLOCKED");
			} else {
				console.log("DENIED");
			}
			ignore_onend = true;
		}
	};

	recognition.onend = function() {
		recognizing = false;
		if (ignore_onend) {
			return;
		}
		if (!final_transcript) {
			console.log("STARTED");
			return;
		}
	};

	recognition.onresult = function(event) {
		var interim_transcript = '';
		for (var i = event.resultIndex; i < event.results.length; ++i) {
			if (event.results[i].isFinal) {
				final_transcript += event.results[i][0].transcript;
			} else {
				interim_transcript += event.results[i][0].transcript;
			}
		}
		speech_results.innerHTML = '\"' + final_transcript + '\"';
	};
}

function startSpeech(event) {
	console.log("STARTED");
	$("#start_speak").html("<em class=\"glyphicon glyphicon-record\"></em> Recording...")
	if (recognizing) {
		$("#start_speak").html("<em class=\"glyphicon glyphicon-record\"></em> Speak")
		recognition.stop();
		$("#alert_1").text("Accept Command Below...");
		return;
	}
	final_transcript = '';
	recognition.lang = lang;
	recognition.start();
	ignore_onend = false;
	speech_results.innerHTML = '';
	start_timestamp = event.timeStamp;
}

function nlp(text){
	$.ajax({
		url: 'https://api.wit.ai/message',
		data: {
			'q': text,
			'access_token' : MY_WIT_TOKEN
		},
		dataType: 'jsonp',
		method: 'GET',
		success: function(response) {
			console.log(response);
			action=food=duration1=duration2=unit1=unit2="";
			utensil = utensil_specifier = location_id = "";
			if(response.entities.action){
				action = response.entities.action[0].value;
				if (response.entities.action[0].metadata == "reset_robot"){
					understand = "Resetting robot now";
					send_cmd(action,"","","");			
					return
				}
			}else{
				action = "";
			}

			if(response.entities.location_id){
				location_id = response.entities.location_id[0].value;
			}else{
				location_id = "";
			}

			if(response.entities.utensil){
				if(response.entities.utensil){
					utensil = response.entities.utensil[0].value;
				}else{
					utensil = "";
				}
				if(response.entities.utensil_specifier){
					utensil_specifier = response.entities.utensil_specifier[0].value;
				}else{
					utensil_specifier = "";
				}

				if(action!="" && utensil!=""){
					if(utensil_specifier!=""){
						understand = "Now " + action + "ing your " + utensil_specifier + " " + utensil;
					}else if(utensil_specifier==""){
						understand = "Now " + action + "ing your " + utensil;
					}else{
						understand = "Sorry, I did not follow. Please repeat.";	
					}

					if(location_id != ""){
						understand = understand + " at " + location_id;
					}
				}
			}else{
				if(response.entities.food){
					food = response.entities.food[0].value;
				}else{
					food = "";
				}
				if(response.entities.duration){
					if(response.entities.duration.length == 1){
						duration1 = response.entities.duration[0].value;
						unit1 = response.entities.duration[0].unit;
					}else if(response.entities.duration.length == 2){
						duration1 = response.entities.duration[0].value;
						unit1 = response.entities.duration[0].unit;
						duration2 = response.entities.duration[1].value;
						unit2 = response.entities.duration[1].unit;
					}
				}else{
					duration1, duration2 = "";
					unit1, unit2 = "";
				}
				if(action!="" && food!="" && duration1!=""){
					understand = "Now " + action + "ing your " + food + " for " + duration1 + " " + unit1;
					if(duration2!="" && unit2!=""){
						understand = "Now " + action + "ing your " + food + " for " + duration1 + " " + unit1 + " and " + duration2 + " " + unit2;
					}
				}else{
					understand = "Sorry, I did not follow. Please repeat.";
				}
			}
			send_cmd(action,utensil_specifier,utensil,location_id);			
		}
	});
}

function to_speak(this_thing){
	if (!('speechSynthesis' in window)) {
		console.log("CANNOT SPEAK");
	} else{
		var to_say = new SpeechSynthesisUtterance(this_thing);
		to_say.lang = 'en-US';
		window.speechSynthesis.speak(to_say);
	}
}