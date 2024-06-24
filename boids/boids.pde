import controlP5.*; //<>// //<>//
import g4p_controls.*;

boolean simulationFinished = false;


Camera cam;
PGraphics camBuffer;

Flock flock;
boolean debug = false; // Global debug flag
boolean influences = false; // Global debug influences flag

boolean leftClicked = false;
boolean rightClicked = false;

// Elliptic curve
PVector circleCenter;
float circleRadius;

float angle = 0; // Angle to determine the position on the circle

GButton playPauseButton;
boolean isPlaying = true;

int frameCounter = 1;

// Static constants
public static final int NUM_BOIDS = 10;
public static final boolean OVERRIDE_LIMITS_FOR_LEADER_INFLUENCE = false;

//// These will be adjustable via sliders
//// Define your configuration parameters with initial values
float maxForce = 0.03f; // Maximum steering force
float maxSpeed = 2;     // Maximum speed
float fov = 270;
//float desiredSeparation = 45.0f; // Desired separation between boids //<>//
//float neighborDist = 105.0f; // Distance to consider boids as neighbors
//float separationWeight = 1.5f; // Weight for separation force
//float alignmentWeight = 1.0f; // Weight for alignment force
//float cohesionWeight = 1.0f; // Weight for cohesion force
//float boidSize = 1.5f; // Size of the boid
//float leaderInfluenceWeightSeparate = 15.0f; // Weight for leader's influence
//float leaderInfluenceWeightAlign = 10.0f; // Weight for leader's influence
//float leaderInfluenceWeightCohere = 10.0f; // Weight for leader's influence
//float leaderInfluenceWeightChase = 0.0f; // Weight on how much the leader is chased

// These will be adjustable via sliders
// Define your configuration parameters with initial values

float desiredSeparation = 0.0f; // Desired separation between boids
float neighborDist = 0.0f; // Distance to consider boids as neighbors
float separationWeight = 0.0f; // Weight for separation force
float alignmentWeight = 0.0f; // Weight for alignment force
float cohesionWeight = 0.0f; // Weight for cohesion force
float boidSize = 0.0f; // Size of the boid
float leaderInfluenceWeightSeparate = 0.0f; // Weight for leader's influence
float leaderInfluenceWeightAlign = 0.0f; // Weight for leader's influence
float leaderInfluenceWeightCohere = 0.0f; // Weight for leader's influence
float leaderInfluenceWeightChase = 0.0f; // Weight on how much the leader is chased

// Define hard-coded maximum values for sliders
float maxMaxForce = 0.2f;
float maxMaxSpeed = 5.0f;
float maxDesiredSeparation = 100.0f;
float maxNeighborDist = 400.0f;
float maxSeparationWeight = 2.0f;
float maxAlignmentWeight = 2.0f;
float maxCohesionWeight = 2.0f;
float maxBoidSize = 10.0f;
float maxLeaderInfluenceWeightSeparate = 4.0f;
float maxLeaderInfluenceWeightAlign = 4.0f;
float maxLeaderInfluenceWeightCohere = 4.0f;
float maxLeaderInfluenceWeightChase = 4.0f;



// Define the number of slices for each parameter
int numSlices = 5;

float stepDesiredSeparation = (maxDesiredSeparation - 0.0f) / numSlices;
float stepNeighborDist = (maxNeighborDist - 0.0f) / numSlices;
float stepSeparationWeight = (maxSeparationWeight - 0.0f) / numSlices;
float stepAlignmentWeight = (maxAlignmentWeight - 0.0f) / numSlices;
float stepCohesionWeight = (maxCohesionWeight - 0.0f) / numSlices;


// Define the current permutation index
int permutationCounter = 1;
int runCounter = 0;
int runAmmount = 10;
int runTime = 1000;

Boid controlledLeader = new Boid(0, 0);
boolean isControlled = false;

ControlP5 cp5;

//Play Pause Button
int buttonWidth = 80;
int buttonHeight = 30;
int xPosition = 0; // Will be set in setup() after window size is determined
int yPosition = 10; // 10 pixels from the top edge

boolean showMenu = false;


//Obstacles
boolean obstacleMode = false; // Flag for obstacle mode
PVector obstacleStart; // Starting point of the obstacle
boolean drawingObstacle = false; // Flag to indicate if an obstacle is being drawn

void setup() {
  frameRate(1000);
  size(1000, 1000);  // Set the size of the window
  int xPosition = width - buttonWidth - 10; // 10 pixels from the right edge
  frameCounter = 0;

  controlledLeader.isLeader = true;
  controlledLeader.isControlled = isControlled;
  controlledLeader.velocity = new PVector(0.0, 0.0);
  controlledLeader.acceleration = new PVector(0, 0);

  startNewRun();

  // Initialize ControlP5
  cp5 = new ControlP5(this);

  // Create sliders for each parameter
  createSlider("maxForce", maxForce, 0.0f, maxMaxForce, 60);
  createSlider("maxSpeed", maxSpeed, 0.0f, maxMaxSpeed, 110);
  createSlider("desiredSeparation", desiredSeparation, 0.0f, maxDesiredSeparation, 160);
  createSlider("viewRange", neighborDist, 0.0f, maxNeighborDist, 210);
  createSlider("FOV", fov, 0, 360, 260);
  createSlider("boidSize", boidSize, 1.5, maxBoidSize, 310);

  createSlider("separationWeight", separationWeight, 0.0f, maxSeparationWeight, 380);
  createSlider("alignmentWeight", alignmentWeight, 0.0f, maxAlignmentWeight, 430);
  createSlider("cohesionWeight", cohesionWeight, 0.0f, maxCohesionWeight, 480);

  createSlider("leaderInfluenceWeightSeparate", leaderInfluenceWeightSeparate, 0.0f, maxLeaderInfluenceWeightSeparate, 550);
  createSlider("leaderInfluenceWeightAlign", leaderInfluenceWeightAlign, 0.0f, maxLeaderInfluenceWeightAlign, 600);
  createSlider("leaderInfluenceWeightCohere", leaderInfluenceWeightCohere, 0.0f, maxLeaderInfluenceWeightCohere, 650);
  createSlider("leaderInfluenceWeightChase", leaderInfluenceWeightChase, 0.0f, maxLeaderInfluenceWeightChase, 700);


  hideMenu(); // Start with sliders hidden

  camBuffer = createGraphics(400, 400);  // Create the off-screen buffer
  cam = new Camera(camBuffer, flock, controlledLeader);


  playPauseButton = new GButton(this, xPosition, yPosition, buttonWidth, buttonHeight, "Pause");
  playPauseButton.addEventHandler(this, "handleButtonEvents");
  while(!simulationFinished)
  drawSimulation();
}

void startNewRun() {
  // Reset the frame counter
  frameCounter = 1;

  // Increment the run counter
  runCounter++;

  flock = new Flock();

  for (int i = 0; i < NUM_BOIDS; i++) {
    float positionX = random(0, width);
    float positionY = random(0, height);
    Boid boid = new Boid(positionX, positionY);
    flock.addBoid(boid);
  }
}

void createSlider(String name, float value, float min, float max, int yOffset) {
  cp5.addSlider(name)
    .setPosition(width / 2 - 150, yOffset)
    .setSize(200, 20)
    .setRange(min, max)
    .setValue(value)
    .hide()
    .setColorForeground(color(100))
    .setColorBackground(color(50))
    .setColorActive(color(255, 0, 0));
}

void createCheckBox(String name, boolean value, int yOffset) {
  cp5.addToggle(name)
    .setPosition(width / 2 - 20, yOffset)
    .setSize(20, 20)
    .setValue(value ? 1 : 0)
    .hide()
    .setColorForeground(color(100))
    .setColorBackground(color(50))
    .setColorActive(color(255, 0, 0));
}

void updateParameters(int permutationIndex) {
  int[] indices = calculatePermutationIndices(permutationIndex);

  // Update parameters based on permutation indices
  desiredSeparation = indices[0] * stepDesiredSeparation;
  neighborDist = indices[1] * stepNeighborDist;
  separationWeight = indices[2] * stepSeparationWeight;
  alignmentWeight = indices[3] * stepAlignmentWeight;
  cohesionWeight = indices[4] * stepCohesionWeight;
}

int[] calculatePermutationIndices(int permutationIndex) {
  int[] indices = new int[5]; // Reduced size to exclude some values
  int remainder = permutationIndex;

  for (int i = 4; i >= 0; i--) { // Adjusted loop range to 10 (indices.length - 1)
    indices[i] = remainder % numSlices;
    remainder /= numSlices;
  }
 //<>//
  return indices;
}

void draw() {
  
}

void drawSimulation() {
  // Check if the maximum number of runs has been reached
  if (runCounter <= runAmmount) {

    // Check if it's time to start a new run based on frame count
    if (frameCounter % (runTime+1) == 0) {
      // New run
      startNewRun();
      return;
    }
  } else {
    
    // Adjust parameters for the next iteration
    updateParameters(permutationCounter);
    
    // Reset permutation counter and increment runCounter when all permutations are done
    if (permutationCounter == pow(numSlices, 5)) {
      simulationFinished = true;
      exit();
    }
    permutationCounter++;
    runCounter = 1;
    return;
  }


  controlledLeader.isControlled = isControlled;
  //background(20);  // Clear the background

  //if (circleCenter != null && circleRadius > 0f) {
  //  drawCircle(this.g);
  //}

  if (isPlaying) {
  //  flock.run();     // Run the flock simulation
  //  if (controlledLeader.isControlled)
  //    controlledLeader.position.set(mouseX, mouseY);

  //  if (leftClicked && circleCenter != null&& !showMenu && !obstacleMode) {
  //    circleRadius = dist(circleCenter.x, circleCenter.y, mouseX, mouseY);
  //  }

  //  if (circleCenter != null && circleRadius > 0f) {
  //    processCircle(controlledLeader);
  //  }

  //  // Check for mouse hover over the dropdown background
  //  if (mouseX >= width / 2 - 20 && mouseX <= width / 2 + 20 && mouseY <= 40) {
  //    showMenu = true;
  //  } else if (!mouseOverDropdownArea()) {
  //    showMenu = false;
  //  }

  //  if (showMenu) {

  //    showMenu();
  //  } else {
  //    // Draw the dropdown arrow
  //    fill(255);
  //    noStroke();
  //    triangle(width / 2 - 10, 10, width / 2 + 10, 10, width / 2, 30);

  //    hideMenu();
  //  }


    //// Update Boid properties with slider values
    //maxForce = cp5.getController("maxForce").getValue();
    //maxSpeed = cp5.getController("maxSpeed").getValue();
    //desiredSeparation = cp5.getController("desiredSeparation").getValue();
    //neighborDist = cp5.getController("viewRange").getValue(); //<>//
    //fov = cp5.getController("FOV").getValue();
    //boidSize = cp5.getController("boidSize").getValue();

    //separationWeight = cp5.getController("separationWeight").getValue();
    //alignmentWeight = cp5.getController("alignmentWeight").getValue();
    //cohesionWeight = cp5.getController("cohesionWeight").getValue();

    //leaderInfluenceWeightSeparate = cp5.getController("leaderInfluenceWeightSeparate").getValue();
    //leaderInfluenceWeightAlign = cp5.getController("leaderInfluenceWeightAlign").getValue();
    //leaderInfluenceWeightCohere = cp5.getController("leaderInfluenceWeightCohere").getValue();
    //leaderInfluenceWeightChase = cp5.getController("leaderInfluenceWeightChase").getValue();

    String filename = "config_" + permutationCounter +"run_" + runCounter + ".csv";  // Example filename
    flock.saveToCSV(filename);
  }
  //flock.display(this.g);

  // Update camBuffer for the camera view
  //camBuffer.beginDraw();
  //camBuffer.background(20);  // Clear the buffer background
  //camBuffer.pushMatrix();
  //camBuffer.translate(camBuffer.width / 2 - controlledLeader.position.x, camBuffer.height / 2 - controlledLeader.position.y);
  //flock.display(camBuffer);
  //if (circleCenter != null && circleRadius > 0f) {
//drawCircle(camBuffer);
  //}
  //camBuffer.popMatrix();
  //camBuffer.endDraw();

  // Display obstacles
  //drawObstacles(this.g);
  //drawObstacles(camBuffer);

  // Display obstacle being drawn if in obstacle mode
  //if (drawingObstacle && obstacleStart != null) {
    //drawTempObstacle(this.g);
  //}


  //displayInfo();


  //fill(255);  // Set text color to white
  //textSize(16);  // Set text size
  //text("Time (t): " + frameCounter, 10, 30); //<>//
  if (isPlaying) {
     frameCounter++; 
  }
}

boolean mouseOverDropdownArea() {
  return (mouseX >= width / 2 - 200 && mouseX <= width / 2 + 200 && mouseY <= 800);
}

// Handle button events
public void handleButtonEvents(GButton button, GEvent event) {
  if (button == playPauseButton && event == GEvent.CLICKED) {
    isPlaying = !isPlaying;
    playPauseButton.setText(isPlaying ? "Pause" : "Play");
  }
}

void showMenu() {
  fill(0, 0, 0, 100);
  noStroke();
  rect(width / 2 - 200, 0, 450, 800 );
  cp5.getController("maxForce").show();
  cp5.getController("maxSpeed").show();
  cp5.getController("desiredSeparation").show();
  cp5.getController("viewRange").show();
  cp5.getController("FOV").show();
  cp5.getController("separationWeight").show();
  cp5.getController("alignmentWeight").show();
  cp5.getController("cohesionWeight").show();
  cp5.getController("boidSize").show();
  cp5.getController("leaderInfluenceWeightSeparate").show();
  cp5.getController("leaderInfluenceWeightAlign").show();
  cp5.getController("leaderInfluenceWeightCohere").show();
  cp5.getController("leaderInfluenceWeightChase").show();
}

void hideMenu() {
  cp5.getController("maxForce").hide();
  cp5.getController("maxSpeed").hide();
  cp5.getController("desiredSeparation").hide();
  cp5.getController("viewRange").hide();
  cp5.getController("FOV").hide();
  cp5.getController("separationWeight").hide();
  cp5.getController("alignmentWeight").hide();
  cp5.getController("cohesionWeight").hide();
  cp5.getController("boidSize").hide();
  cp5.getController("leaderInfluenceWeightSeparate").hide();
  cp5.getController("leaderInfluenceWeightAlign").hide();
  cp5.getController("leaderInfluenceWeightCohere").hide();
  cp5.getController("leaderInfluenceWeightChase").hide();
}

void keyPressed() {
  if (key == 'd') {
    debug = !debug;
    for (Boid boid : flock.boids) {
      boid.debug = debug;
    }
  }

  if (key == 'i') {
    influences = !influences;
    for (Boid boid : flock.boids) {
      boid.influences = influences;
    }
  }

  if (key == 'r') {
    circleCenter = null;
  }

  if (key == 'c') {
    isControlled = !isControlled;
  }

  if (key == ' ') {
    handleButtonEvents(playPauseButton, GEvent.CLICKED);
  }

  if (key == 'o') {
    obstacleMode = !obstacleMode;
    if (obstacleMode) {
      controlledLeader.isControlled = false; // Disable leader control in obstacle mode
    }
  }

  if (key == 'k') {
    flock.obstacles.clear(); // Clear all obstacles
  }
}

void displayInfo() {
  String info = "Press 'd' to see distances\n";
  info += "Press 'i' to see influences\n";
  info += "Press 'r' to reset circle\n";
  info += "Press 'c' to toggle mouse control of the leader\n";
  info += "Press 'SPACEBAR' to Play/Pause the simulation\n";
  info += "Press 'MOUSE_LEFT' to draw a circle\n";
  info += "Press 'MOUSE_RIGHT' to remove leader\n";
  info += "Press 'o' to toggle obstacle mode\n";
  info += "Press 'k' to clear all obstacles\n";

  textAlign(LEFT, BOTTOM);
  fill(255);
  textSize(14);
  text(info, 10, height - 10);
}

boolean mouseOverPlayButton() {
  return mouseX > xPosition && mouseX < xPosition + buttonWidth && mouseY > yPosition && mouseY < yPosition + buttonHeight;
}
void mousePressed() {
  if (obstacleMode) {
    if (mouseButton == LEFT || mouseButton == RIGHT) {
      obstacleStart = new PVector(mouseX, mouseY);
      drawingObstacle = true;
    }
  } else {
    if (mouseButton == LEFT && !showMenu && !mouseOverPlayButton()) {
      leftClicked = !leftClicked;
      if (leftClicked) {
        circleCenter = new PVector(mouseX, mouseY);
      }
    } else if (mouseButton == RIGHT) {
      rightClicked = !rightClicked;
      if (rightClicked) {
        println("right" + controlledLeader.isLeader);
        flock.addControlledBoid(controlledLeader);
      } else {
        println("nright" + controlledLeader.isLeader);
        flock.removeBoid(controlledLeader);
      }
    }
  }
}

void mouseReleased() {
  if (obstacleMode && drawingObstacle) {
    PVector endPoint = new PVector(mouseX, mouseY);
    float radiusX = abs(endPoint.x - obstacleStart.x) / 2; // Adjusted for ellipse
    float radiusY = abs(endPoint.y - obstacleStart.y) / 2; // Adjusted for ellipse
    float centerX = (obstacleStart.x + endPoint.x) / 2; // Center of ellipse
    float centerY = (obstacleStart.y + endPoint.y) / 2; // Center of ellipse
    flock.addObstacle(new Obstacle(centerX, centerY, radiusX  * 2, radiusY  * 2));

    drawingObstacle = false;
    obstacleStart = null;
  }
}

void drawObstacles(PGraphics buffer) {
  for (Obstacle obstacle : flock.obstacles) {
    obstacle.display(buffer);
  }
}

void drawTempObstacle(PGraphics buffer) {
  if (obstacleStart != null) {
    buffer.stroke(255);
    buffer.noFill();
    PVector currentPos = new PVector(mouseX, mouseY);

    float radiusX = abs(currentPos.x - obstacleStart.x) / 2; // Adjusted for ellipse
    float radiusY = abs(currentPos.y - obstacleStart.y) / 2; // Adjusted for ellipse
    float centerX = (obstacleStart.x + currentPos.x) / 2; // Center of ellipse
    float centerY = (obstacleStart.y + currentPos.y) / 2; // Center of ellipse
    buffer.ellipse(centerX, centerY, radiusX * 2, radiusY * 2);
  }
}

void drawCircle( PGraphics buffer) {
  // Draw the circle
  buffer.stroke(255);
  buffer.noFill();
  buffer.ellipse(circleCenter.x, circleCenter.y, circleRadius * 2, circleRadius * 2);
}


void processCircle(Boid controlledBoid) {
  // Calculate the angular velocity to maintain constant speed
  float angularVelocity = maxSpeed / circleRadius;

  // Update the angle to move the boid around the circle
  angle += angularVelocity;

  // Calculate the new position based on the angle
  float x = circleCenter.x + circleRadius * cos(angle);
  float y = circleCenter.y + circleRadius * sin(angle);

  // Update the controlled boid's position
  controlledBoid.position.set(x, y);

  // Calculate the new velocity to be tangent to the circle
  float vx = -circleRadius * sin(angle) * angularVelocity;
  float vy = circleRadius * cos(angle) * angularVelocity;
  PVector velocity = new PVector(vx, vy);
  velocity.setMag(maxSpeed); // Ensure the velocity has the maximum speed

  // Update the controlled boid's velocity
  controlledBoid.velocity.set(velocity);
}
