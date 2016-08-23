function speak_state_type(state){
	var say_state = '';
	switch(state){
		case 0: say_state = "I am awaiting your command.";
		break;
		case 2: say_state = "I am now going to pick the object. Please wait.";
		break;
		case 3: say_state = "I am now going to place the object. Please wait.";
		break;
		case 44: say_state = "I'm done. What next?";
		break;
		case 5: say_state = "Please wait while I reset.";
		break;
		case 6: say_state = "I am going to put the object into the microwave.";
		break;
		case 7: say_state = "The microwave is too far. I will return the object to the original location.";
		break;
		case 10: 
		say_state = "I am now going to open the microwave.";
		break;
		case 11: 
		say_state = "I will now close the microwave.";
		break;
		case 21: say_state = "Let me look for the microwave. Please wait.";
		break;
		case 34: say_state = "Looking for your specified object.";
		break;
		case 40: say_state = "Hold on while I try to find all the objects on the table.";
		break;
		case -10: 
		err_type = 2;
		say_state = "Sorry! I don't know which " + utensil_specifier + " " + utensil + " you are referring to. Please specify.";
		$("#alert_1").text(say_state);
		$("#intervene").trigger('click');
		setTimeout(function(){
			$("#fetch_img").trigger('click');
		}, 5000);
		break;
		case -2: 
		say_state = "Oops! I failed to open the microwave.";
		break;
		case -21: 
		say_state = "Sorry! I failed to pick up object";
		break;
		case -22: 
		say_state = "What a pity! I failed to place the object";
		break;
		default: say_state = "Unknown Error. Contact Human.";
		break;
	}
	to_speak(say_state);
}