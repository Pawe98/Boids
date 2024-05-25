import java.util.ArrayList;

class Flock {
  ArrayList<Boid> boids; // List of all boids in the flock

  // Constructor initializes the list of boids
  Flock() {
    boids = new ArrayList<Boid>();
  }

  // Add a new boid to the flock
  void addBoid(Boid b) {
    boids.add(b);
  }

  // Run the simulation for all boids in the flock
  void run() {
    for (Boid b : boids) {
      b.checkClicked();  // Check if the boid is clicked
      b.flock(boids);  // Calculate and apply flocking behaviors
      b.update();      // Update the boid's position based on velocity and acceleration
      b.edges();       // Handle edge wrapping
      b.display(boids);     // Draw the boid
    }
  }
}
