// Instance the tour
var tour = new Tour({
  steps: [
  {
    element: "#header",
    title: "Let us begin!",
    content: "This is the Human Interaction Module. It lets you view and manage all aspects of your new robot."
  },
  {
    element: "#visualize",
    title: "The Visualizer Module",
    backdrop: true,
    content: "This allows you to visualize the robot doing it's awesome thing."
  },
  {
    element: "#intervene",
    title: "The Intervention Module",
    backdrop: true,
    content: "This allows you to help out your robot when it needs assistance."
  },
  {
    element: "#command",
    title: "The Command Module",
    backdrop: true,
    content: "The part you have always wanted. Give voice commands to your robot here."
  },
  {
    element: "#main_content",
    title: "The Interaction Window",
    backdrop: true,
    content: "This part shows you images from the robot camera or the live model of the robot. It also changes to allow speech input."
  },
  {
    element: "#conn_panel",
    title: "The Connection Status Bar",
    backdrop: true,
    placement: "bottom",
    content: "Displays the current connection status, allows you to enable/disable connection to robot."
  },
  {
    element: "#instructions_panel",
    title: "The Instructions Panel",
    backdrop: true,
    placement: "bottom",
    content: "Displays the necessary instructions to help out your robot."
  },
  {
    element: "#alert_panel",
    title: "The Alert Panel",
    backdrop: true,
    placement: "bottom",
    content: "Alerts you visually and aurally when your robot needs your help."
  }
  ]});

function begin_tour(){  
// Initialize the tour
tour.init();

// Start the tour
tour.start(true);

// Set to first
tour.setCurrentStep(0);

// Restart
tour.restart()
}