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

public static final float MAX_FORCE = 0.03f; // Maximum steering force
public static final float MAX_SPEED = 2;     // Maximum speed

public static final float DESIRED_SEPARATION = 35.0f; // Desired separation between boids
public static final float NEIGHBOR_DIST = 250.0f; // Distance to consider boids as neighbors
public static final float SEPARATION_WEIGHT = 1.5f; // Weight for separation force
public static final float ALIGNMENT_WEIGHT = 1.0f; // Weight for alignment force
public static final float COHESION_WEIGHT = 1.0f; // Weight for cohesion force
public static final float BOID_SIZE = 3; // Size of the boid

public static final boolean OVERRIDE_LIMITS_FOR_LEADER_INFLUENCE = true;

public static final float LEADER_INFLUENCE_WEIGHT_SEPARATE = 1.5f; // Weight for leader's influence
public static final float LEADER_INFLUENCE_WEIGHT_ALIGN = 1.0f; // Weight for leader's influence
public static final float LEADER_INFLUENCE_WEIGHT_COHERE = 1.0f; // Weight for leader's influence

public static final float LEADER_INFLUENCE_WEIGHT_CHASE = 1.0f; // Weight on how much the leader is chased

Boid controlledLeader = new Boid(0, 0);

void setup() {
  size(1000, 1000);  // Set the size of the window
  controlledLeader.isLeader = true;
  controlledLeader.isControlled = true;
  controlledLeader.velocity = new PVector(0.0, 0.0);
  controlledLeader.acceleration = new PVector(0, 0);
  flock = new Flock();  // Create a new flock
  for (int i = 0; i < NUM_BOIDS; i++) {
    flock.addBoid(new Boid(random(1000), random(1000))); // Add 100 boids at random positions
  }
}

//TODO BUG: when the circle is removed, the mouse cursor deletes boids. Just check again when the leader is controlled by mouse or by the circle. Rework this. Dont know exactly why, but there is definetly some race condition stuff where a boolean flips but then is only counted after the next cycle and creates issues.

void draw() {
  background(51);  // Clear the background
  flock.run();     // Run the flock simulation

  controlledLeader.position.set(mouseX, mouseY);


  if (leftClicked) {
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
  //println(controlledLeader.position);
}

void drawCircle(Boid controlledBoid) {
  // Calculate the angular velocity to maintain constant speed
  float angularVelocity = MAX_SPEED / circleRadius;

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
  velocity.setMag(MAX_SPEED); // Ensure the velocity has the maximum speed

  // Update the controlled boid's velocity
  controlledBoid.velocity.set(velocity);

  // Draw the circle
  stroke(255);
  noFill();
  ellipse(circleCenter.x, circleCenter.y, circleRadius * 2, circleRadius * 2);
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
    if(leftClicked) {
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
