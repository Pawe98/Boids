import controlP5.*;

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
public static final int NUM_BOIDS = 100;
public static final float ANGLE_INCREMENT = 0.02;

// These will be adjustable via sliders
float maxForce = 0.03f; // Maximum steering force
float maxSpeed = 2;     // Maximum speed
float desiredSeparation = 35.0f; // Desired separation between boids
float neighborDist = 250.0f; // Distance to consider boids as neighbors
float separationWeight = 1.5f; // Weight for separation force
float alignmentWeight = 1.0f; // Weight for alignment force
float cohesionWeight = 1.0f; // Weight for cohesion force
float boidSize = 3; // Size of the boid

public static final boolean OVERRIDE_LIMITS_FOR_LEADER_INFLUENCE = true;

float leaderInfluenceWeightSeparate = 1.5f; // Weight for leader's influence
float leaderInfluenceWeightAlign = 1.0f; // Weight for leader's influence
float leaderInfluenceWeightCohere = 1.0f; // Weight for leader's influence
float leaderInfluenceWeightChase = 1.0f; // Weight on how much the leader is chased

Boid controlledLeader = new Boid(0, 0);

ControlP5 cp5;

boolean showMenu = false;

void setup() {
  flock = new Flock();
  size(1000, 1000);  // Set the size of the window
  controlledLeader.isLeader = true;
  controlledLeader.isControlled = true;
  controlledLeader.velocity = new PVector(0.0, 0.0);
  controlledLeader.acceleration = new PVector(0, 0);
  flock = new Flock();  // Create a new flock
  for (int i = 0; i < NUM_BOIDS; i++) {
    flock.addBoid(new Boid(random(1000), random(1000))); // Add 100 boids at random positions
  }

  // Initialize ControlP5
  cp5 = new ControlP5(this);

  // Create sliders for each parameter
  createSlider("maxForce", maxForce, 0.01f, 0.1f, 110);
  createSlider("maxSpeed", maxSpeed, 0.5f, 5, 160);
  createSlider("desiredSeparation", desiredSeparation, 10, 100, 210);
  createSlider("neighborDist", neighborDist, 50, 500, 260);
  createSlider("separationWeight", separationWeight, 0.5f, 2.5f, 310);
  createSlider("alignmentWeight", alignmentWeight, 0.5f, 2.5f, 360);
  createSlider("cohesionWeight", cohesionWeight, 0.5f, 2.5f, 410);
  createSlider("boidSize", boidSize, 1, 10, 460);
  createSlider("leaderInfluenceWeightSeparate", leaderInfluenceWeightSeparate, 0.5f, 2.5f, 510);
  createSlider("leaderInfluenceWeightAlign", leaderInfluenceWeightAlign, 0.5f, 2.5f, 560);
  createSlider("leaderInfluenceWeightCohere", leaderInfluenceWeightCohere, 0.5f, 2.5f, 610);
  createSlider("leaderInfluenceWeightChase", leaderInfluenceWeightChase, 0.5f, 2.5f, 660);


  hideSliders(); // Start with sliders hidden
}

void createSlider(String name, float value, float min, float max, int yOffset) {
  cp5.addSlider(name)
    .setPosition(width / 2 - 200, yOffset)
    .setSize(200, 20)
    .setRange(min, max)
    .setValue(value)
    .hide()
    .setColorForeground(color(100))
    .setColorBackground(color(50))
    .setColorActive(color(255, 0, 0));
}

void draw() {
  background(51);  // Clear the background
  flock.run();     // Run the flock simulation

  controlledLeader.position.set(mouseX, mouseY);

  if (leftClicked && circleCenter != null&& !showMenu) {
    circleRadius = dist(circleCenter.x, circleCenter.y, mouseX, mouseY);
  }

  if (circleCenter != null && circleRadius > 0f) {
    drawCircle(controlledLeader);
  }

  if (circleCenter != null) {
    stroke(255);
    noFill();
    ellipse(circleCenter.x, circleCenter.y, circleRadius * 2, circleRadius * 2);
  }

  // Draw the background for dropdown
  if (showMenu) {
    fill(0,0,0,100);
    noStroke();
    rect(width / 2 - 200, 0, 450, 700 );
  }
  // Draw the dropdown arrow
  fill(255);
  noStroke();
  triangle(width / 2 - 10, 10, width / 2 + 10, 10, width / 2, 30);

  // Check for mouse hover over the dropdown background
  if (mouseX >= width / 2 - 20 && mouseX <= width / 2 + 20 && mouseY <= 40) {
    showMenu = true;
    showSliders();
  } else if (!mouseOverDropdownArea()) {
    showMenu = false;
    hideSliders();
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
}

boolean mouseOverDropdownArea() {
  return (mouseX >= width / 2 - 200 && mouseX <= width / 2 + 200 && mouseY <= 700);
}

void showSliders() {
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
}

void hideSliders() {
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
}


void mousePressed() {
  if (mouseButton == LEFT) {
    leftClicked = !leftClicked;
    if (leftClicked && !showMenu) {
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
void drawCircle(Boid controlledBoid) {
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

  // Draw the circle
  stroke(255);
  noFill();
  ellipse(circleCenter.x, circleCenter.y, circleRadius * 2, circleRadius * 2);
}
