import controlP5.*;

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

// Static constants
public static final int NUM_BOIDS = 25;
public static final boolean OVERRIDE_LIMITS_FOR_LEADER_INFLUENCE = true;

// These will be adjustable via sliders
float maxForce = 0.03f; // Maximum steering force
float maxSpeed = 2;     // Maximum speed
float fov = 270;
float desiredSeparation = 55.0f; // Desired separation between boids
float neighborDist = 150.0f; // Distance to consider boids as neighbors
float separationWeight = 1.5f; // Weight for separation force
float alignmentWeight = 1.0f; // Weight for alignment force
float cohesionWeight = 1.0f; // Weight for cohesion force
float boidSize = 3; // Size of the boid

//TODO ADD FOV SLIDER
//TODO ADD PLAY/PAUSE
//TODO ADD FRAMECOUNTER to bottom of view



float leaderInfluenceWeightSeparate = 3.0f; // Weight for leader's influence
float leaderInfluenceWeightAlign = 2.0f; // Weight for leader's influence
float leaderInfluenceWeightCohere = 2.0f; // Weight for leader's influence
float leaderInfluenceWeightChase = 0.0f; // Weight on how much the leader is chased

Boid controlledLeader = new Boid(0, 0);
boolean isControlled = false;

ControlP5 cp5;

boolean showMenu = false;

void setup() {
  flock = new Flock();
  size(1000, 1000);  // Set the size of the window
  controlledLeader.isLeader = true;
  controlledLeader.isControlled = isControlled;
  controlledLeader.velocity = new PVector(0.0, 0.0);
  controlledLeader.acceleration = new PVector(0, 0);
  flock = new Flock();  // Create a new flock

  circleCenter = new PVector(width / 2, height / 2);
  circleRadius = 200;
  for (int i = 0; i < NUM_BOIDS; i++) {
    float angle = map(i, 0, NUM_BOIDS, 0, TWO_PI);
    float x = circleCenter.x + circleRadius * cos(angle);
    float y = circleCenter.y + circleRadius * sin(angle);
    PVector position = new PVector(x, y);

    // Berechne die tangentiale Geschwindigkeit
    float vx = -circleRadius * sin(angle) * (maxSpeed / circleRadius);
    float vy = circleRadius * cos(angle) * (maxSpeed / circleRadius);
    PVector velocity = new PVector(vx, vy);
    velocity.setMag(maxSpeed);

    Boid boid = new Boid(position.x, position.y);
    boid.velocity.set(velocity);
    flock.addBoid(boid);
  }
  // Initialize ControlP5
  cp5 = new ControlP5(this);

  // Create sliders for each parameter
  createSlider("maxForce", maxForce, 0.0f, 0.2f, 110);
  createSlider("maxSpeed", maxSpeed, 0.0f, 5, 160);
  createSlider("desiredSeparation", desiredSeparation, 0.0f, 100, 210);
  createSlider("neighborDist", neighborDist, 0.0f, 500, 260);
  createSlider("separationWeight", separationWeight, 0.0f, 5.0f, 310);
  createSlider("alignmentWeight", alignmentWeight, 0.0f, 5.0f, 360);
  createSlider("cohesionWeight", cohesionWeight, 0.0f, 5.0f, 410);
  createSlider("boidSize", boidSize, 1, 10, 460);
  createSlider("leaderInfluenceWeightSeparate", leaderInfluenceWeightSeparate, 0.0f, 20.0f, 510);
  createSlider("leaderInfluenceWeightAlign", leaderInfluenceWeightAlign, 0.0f, 20.0f, 560);
  createSlider("leaderInfluenceWeightCohere", leaderInfluenceWeightCohere, 0.0f, 20.0f, 610);
  createSlider("leaderInfluenceWeightChase", leaderInfluenceWeightChase, 0.0f, 20.0f, 660);

  // Create checkbox for isControlled
  createCheckBox("isControlled", isControlled, 710);


  hideMenu(); // Start with sliders hidden

  camBuffer = createGraphics(400, 400);  // Create the off-screen buffer
  cam = new Camera(camBuffer, flock, controlledLeader);
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

void draw() {
  controlledLeader.isControlled = isControlled;
  background(51);  // Clear the background
  flock.run();     // Run the flock simulation
  flock.display(this.g);

  if (controlledLeader.isControlled)
    controlledLeader.position.set(mouseX, mouseY);

  if (leftClicked && circleCenter != null&& !showMenu) {
    circleRadius = dist(circleCenter.x, circleCenter.y, mouseX, mouseY);
  }

  if (circleCenter != null && circleRadius > 0f) {
    processCircle(controlledLeader);
    drawCircle(this.g);
  }

  // Check for mouse hover over the dropdown background
  if (mouseX >= width / 2 - 20 && mouseX <= width / 2 + 20 && mouseY <= 40) {
    showMenu = true;
  } else if (!mouseOverDropdownArea()) {
    showMenu = false;
  }

  if (showMenu) {

    showMenu();
  } else {
    // Draw the dropdown arrow
    fill(255);
    noStroke();
    triangle(width / 2 - 10, 10, width / 2 + 10, 10, width / 2, 30);

    hideMenu();
  }


  // Update Boid properties with slider values
  maxForce = cp5.getController("maxForce").getValue();
  maxSpeed = cp5.getController("maxSpeed").getValue();
  desiredSeparation = cp5.getController("desiredSeparation").getValue();
  neighborDist = cp5.getController("neighborDist").getValue();
  separationWeight = cp5.getController("separationWeight").getValue();
  alignmentWeight = cp5.getController("alignmentWeight").getValue();
  cohesionWeight = cp5.getController("cohesionWeight").getValue();
  boidSize = cp5.getController("boidSize").getValue();
  leaderInfluenceWeightSeparate = cp5.getController("leaderInfluenceWeightSeparate").getValue();
  leaderInfluenceWeightAlign = cp5.getController("leaderInfluenceWeightAlign").getValue();
  leaderInfluenceWeightCohere = cp5.getController("leaderInfluenceWeightCohere").getValue();
  leaderInfluenceWeightChase = cp5.getController("leaderInfluenceWeightChase").getValue();

  // Update camBuffer for the camera view
  camBuffer.beginDraw();
  camBuffer.background(51);  // Clear the buffer background
  camBuffer.pushMatrix();
  camBuffer.translate(camBuffer.width / 2 - controlledLeader.position.x, camBuffer.height / 2 - controlledLeader.position.y);
  flock.display(camBuffer);
  if (circleCenter != null && circleRadius > 0f) {
    drawCircle(camBuffer);
  }
  camBuffer.popMatrix();
  camBuffer.endDraw();

  displayInfo();
}

boolean mouseOverDropdownArea() {
  return (mouseX >= width / 2 - 200 && mouseX <= width / 2 + 200 && mouseY <= 800);
}

void showMenu() {
  fill(0, 0, 0, 100);
  noStroke();
  rect(width / 2 - 200, 0, 450, 800 );
  cp5.getController("maxForce").show();
  cp5.getController("maxSpeed").show();
  cp5.getController("desiredSeparation").show();
  cp5.getController("neighborDist").show();
  cp5.getController("separationWeight").show();
  cp5.getController("alignmentWeight").show();
  cp5.getController("cohesionWeight").show();
  cp5.getController("boidSize").show();
  cp5.getController("leaderInfluenceWeightSeparate").show();
  cp5.getController("leaderInfluenceWeightAlign").show();
  cp5.getController("leaderInfluenceWeightCohere").show();
  cp5.getController("leaderInfluenceWeightChase").show();
  cp5.getController("isControlled").show();
}

void hideMenu() {
  cp5.getController("maxForce").hide();
  cp5.getController("maxSpeed").hide();
  cp5.getController("desiredSeparation").hide();
  cp5.getController("neighborDist").hide();
  cp5.getController("separationWeight").hide();
  cp5.getController("alignmentWeight").hide();
  cp5.getController("cohesionWeight").hide();
  cp5.getController("boidSize").hide();
  cp5.getController("leaderInfluenceWeightSeparate").hide();
  cp5.getController("leaderInfluenceWeightAlign").hide();
  cp5.getController("leaderInfluenceWeightCohere").hide();
  cp5.getController("leaderInfluenceWeightChase").hide();
  cp5.getController("isControlled").hide();
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
    cp5.getController("isControlled").changeValue(isControlled ? 1 : 0);
  }
}

void displayInfo() {
  String info = "Press 'd' to see distances\n";
  info += "Press 'i' to see influences\n";
  info += "Press 'r' to reset circle\n";
  info += "Press 'c' to toggle mouse control of the leader\n";
  info += "Press 'MOUSE_LEFT' to draw a circle\n";
  info += "Press 'MOUSE_RIGHT' to remove leader\n";


  textAlign(LEFT, BOTTOM);
  fill(255);
  textSize(14);
  text(info, 10, height - 10);
}


void mousePressed() {
  if (mouseButton == LEFT && !showMenu) {
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
