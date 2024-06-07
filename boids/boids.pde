Flock flock;
boolean debug = false; // Global debug flag
boolean influences = false; // Global debug influences flag

boolean leftClicked = false;
boolean rightClicked = false;

Boid controlledLeader = new Boid(random(width), random(height));
void setup() {
  size(1000, 1000);  // Set the size of the window
  controlledLeader.isLeader = true;
  controlledLeader.isControlled = true;
  controlledLeader.velocity = new PVector(0.0,0.0);
  controlledLeader.acceleration = new PVector(0,0);
  flock = new Flock();  // Create a new flock
  for (int i = 0; i < 10; i++) {
    flock.addBoid(new Boid(random(width), random(height))); // Add 150 boids at random positions
  }
}

void draw() {
  background(51);  // Clear the background
  flock.run();     // Run the flock simulation
  controlledLeader.position.set(mouseX, mouseY);
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
}

void mousePressed() {
  if (mouseButton == LEFT) {
    leftClicked = !leftClicked;
  } else if (mouseButton == RIGHT) {
    rightClicked = !rightClicked;
    if (rightClicked) {
      print("right" + controlledLeader.isLeader);
      flock.addControlledBoid(controlledLeader);
    } else {
      flock.removeBoid(controlledLeader);
    }
  }
}
