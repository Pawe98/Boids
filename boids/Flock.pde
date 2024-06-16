import java.util.ArrayList;

class Flock {
  ArrayList<Boid> boids; // List of all boids in the flock
  ArrayList<Obstacle> obstacles; // List of all obstacles
  Boid controlledBoid;
  // Constructor initializes the list of boids
  Flock() {
    boids = new ArrayList<Boid>();
    obstacles = new ArrayList<Obstacle>();
  }

  void addObstacle(Obstacle obstacle) {
     obstacles.add(obstacle); 
  }

  // Add a new boid to the flock
  void addBoid(Boid b) {
    boids.add(b);
  }
  
  // Add a new boid list to the flock
  void addAllBoids(ArrayList<Boid> b) {
    boids.addAll(b);
  }

  // Add a new boid to the flock
  void addControlledBoid(Boid b) {
    boids.add(b);
    controlledBoid = b;
  }

  void removeBoid(Boid b) {
    boids.remove(b);
  }

  // Run the simulation for all boids in the flock
  void run() {
    for (Boid b : boids) {
      b.checkClicked();  // Check if the boid is clicked
      b.flock(boids, obstacles);  // Calculate and apply flocking behaviors
      b.update();      // Update the boid's position based on velocity and acceleration
      b.edges();       // Handle edge wrapping
    }
  }
  
  void display(PGraphics g) {
    for (Boid b : boids) {
      if(b != null)
      b.display(g, boids);     // Draw the boid
    }
  }
}
