Flock flock;
boolean debug = false; // Global debug flag
boolean influences = false; // Global debug influences flag


void setup() {
  size(800, 600);  // Set the size of the window
  flock = new Flock();  // Create a new flock
  for (int i = 0; i < 50; i++) {
    flock.addBoid(new Boid(random(width), random(height))); // Add 150 boids at random positions
  }
}

void draw() {
  background(51);  // Clear the background
  flock.run();     // Run the flock simulation
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
