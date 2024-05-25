import java.util.List;
import java.util.Arrays;


class Boid {
  PVector position;   // Position of the boid
  PVector velocity;   // Velocity of the boid
  PVector acceleration; // Acceleration of the boid

  boolean isLeader = false; // Indicates if the boid is a leader
  boolean isClicked = false; // Indicates if the boid is clicked
  boolean prevMousePressed = false; // Previous mouse pressed state

  // Constants for the simulation
  static final float MAX_FORCE = 0.05f; // Maximum steering force
  static final float MAX_SPEED = 2;     // Maximum speed
  static final float MAX_CHASE_SPEED = 1.5;     // Maximum speed
  static final float DESIRED_SEPARATION = 25.0f; // Desired separation between boids
  static final float NEIGHBOR_DIST = 50.0f; // Distance to consider boids as neighbors
  static final float SEPARATION_WEIGHT = 1.5f; // Weight for separation force
  static final float ALIGNMENT_WEIGHT = 1.0f; // Weight for alignment force
  static final float COHESION_WEIGHT = 1.0f; // Weight for cohesion force
  static final float BOID_SIZE = 6; // Size of the boid

  boolean debug = false; // Toggle for debug mode
  boolean influences = false; //Toggle for debug influences

  // Constructor initializes the boid's position and gives it a random velocity
  Boid(float x, float y) {
    position = new PVector(x, y);     // Set initial position
    velocity = PVector.random2D();    // Set random initial velocity
    velocity.setMag(random(2, 4));    // Set magnitude of velocity to a random value between 2 and 4
    acceleration = new PVector();     // Initialize acceleration to zero
    isLeader = false; // Initially not a leader
  }

  // Check if the boid is clicked
  void checkClicked() {
    if (mousePressed && !prevMousePressed && dist(position.x, position.y, mouseX, mouseY) < BOID_SIZE) {
      isLeader = !isLeader; // Toggle leader status
      isClicked = true; // Set clicked status
    }

    if (!mousePressed) {
      isClicked = false; // Reset clicked status
    }

    prevMousePressed = mousePressed; // Update previous mouse state
  }

  // Apply a force to the boid by adding it to the acceleration
  void applyForce(PVector force) {
    acceleration.add(force);
  }

  // Calculate and apply the three flocking behaviors: separation, alignment, and cohesion
  void flock(ArrayList<Boid> boids) {
    if (!isLeader) {
        // If this boid is a follower, chase the leader
        Boid leader = findClosestLeader(boids);
        if (leader != null) {
            //TODO CHASE LOGIC
        }

        PVector separation = separate(boids); // Avoid crowding neighbors
        PVector alignment = align(boids);     // Align with neighbors
        PVector cohesion = cohere(boids);     // Move towards the average position of neighbors

        // Weight these forces
        separation.mult(SEPARATION_WEIGHT);
        alignment.mult(ALIGNMENT_WEIGHT);
        cohesion.mult(COHESION_WEIGHT);

        // Apply the calculated forces
        applyForce(separation);
        applyForce(alignment);
        applyForce(cohesion);
    }
  }

  Boid findClosestLeader(ArrayList<Boid> boids) {
    float closestDist = Float.MAX_VALUE;
    Boid closestLeader = null;

    for (Boid other : boids) {
      if (other.isLeader) {
        float d = PVector.dist(position, other.position);
        if (d < closestDist) {
          closestDist = d;
          closestLeader = other;
        }
      }
    }

    return closestLeader;
  }

  // Separation: Steer to avoid crowding local flockmates
  PVector separate(ArrayList<Boid> boids) {
    PVector steer = new PVector(0, 0); // Initialize the steering vector
    int count = 0;  // Number of boids that are too close

    // Loop through all boids
    for (Boid other : boids) {
      // Calculate toroidal distance between this boid and another boid
      float d = position.dist(other.position);

      // If the other boid is too close
      if ((d > 0) && (d < DESIRED_SEPARATION)) {
        // Calculate toroidal vector pointing away from the other boid
        PVector diff = PVector.sub(position, other.position);
        diff.normalize(); // Normalize to get direction
        diff.div(d);      // Weight by distance
        steer.add(diff);  // Add to steering vector
        count++; // Increment count of close boids
      }
    }

    // Average the steering vector by the number of close boids
    if (count > 0) {
      steer.div((float) count);
    }

    // As long as the steering vector is non-zero
    if (steer.mag() > 0) {
      steer.setMag(MAX_SPEED);  // Set magnitude to maximum speed
      steer.sub(velocity);     // Subtract current velocity to get the steering force
      steer.limit(MAX_FORCE);   // Limit to maximum steering force
    }
    return steer; // Return the calculated steering force
  }

  // Alignment: Steer towards the average heading of local flockmates
  PVector align(ArrayList<Boid> boids) {
    PVector sum = new PVector(0, 0); // Sum of all the velocities
    int count = 0;  // Number of boids that are neighbors

    // Loop through all boids
    for (Boid other : boids) {
      // Calculate toroidal distance between this boid and another boid
      float d = position.dist(other.position);

      // If the other boid is a neighbor
      if ((d > 0) && (d < NEIGHBOR_DIST)) {
        sum.add(other.velocity); // Add the velocity
        count++; // Increment count of neighboring boids
      }
    }

    if (count > 0) {
      sum.div((float) count);   // Average the velocity
      sum.setMag(MAX_SPEED);     // Set magnitude to maximum speed
      PVector steer = PVector.sub(sum, velocity); // Calculate steering force
      steer.limit(MAX_FORCE);    // Limit to maximum steering force
      return steer; // Return the calculated steering force
    } else {
      return new PVector(0, 0); // If no neighbors, return zero steering force
    }
  }

  // Cohesion: Steer to move towards the average position of local flockmates
  PVector cohere(ArrayList<Boid> boids) {
    PVector sum = new PVector(0, 0); // Sum of all positions
    int count = 0;  // Number of boids that are neighbors

    // Loop through all boids
    for (Boid other : boids) {
      // Calculate toroidal distance between this boid and another boid
      float d = position.dist(other.position);

      // If the other boid is a neighbor
      if ((d > 0) && (d < NEIGHBOR_DIST)) {
        sum.add(other.position); // Add the position
        count++; // Increment count of neighboring boids
      }
    }

    if (count > 0) {
      sum.div((float) count); // Average position
      return seek(sum);       // Steer towards the average position
    } else {
      return new PVector(0, 0); // If no neighbors, return zero steering force
    }
  }

  // Seek: Calculate the steering force towards a target
  PVector seek(PVector target) {
    PVector desired = PVector.sub(target, position); // A vector pointing from the position to the target
    desired.setMag(MAX_SPEED); // Set magnitude to maximum speed
    PVector steer = PVector.sub(desired, velocity); // Calculate steering force
    steer.limit(MAX_FORCE);    // Limit to maximum steering force
    return steer;
  }

  // Update the boid's position based on its velocity and acceleration
  void update() {
    velocity.add(acceleration); // Update velocity
    velocity.limit(MAX_SPEED);  // Limit speed
    position.add(velocity);     // Update position
    acceleration.mult(0);       // Reset acceleration to zero
  }

  // Display the boid as a triangle and show different behaviors with colors in debug mode
  void display(ArrayList<Boid> boids) {
    float theta = velocity.heading() + PI / 2; // Calculate heading angle
    if (isLeader) {
      // Display leader differently
      fill(255, 0, 0); // Red color for leader
    } else {
      fill(127); // Normal color for other boids
    }

    stroke(200);

    pushMatrix();
    translate(position.x, position.y);
    rotate(theta);
    beginShape();
    vertex(0, -BOID_SIZE * 2);
    vertex(-BOID_SIZE, BOID_SIZE * 2);
    vertex(BOID_SIZE, BOID_SIZE * 2);
    endShape(CLOSE);
    popMatrix();

    if (debug) {
      // Display detection radii
      noFill();
      stroke(255, 0, 0);
      ellipse(position.x, position.y, DESIRED_SEPARATION * 2, DESIRED_SEPARATION * 2); // Separation radius
      stroke(0, 255, 0);
      ellipse(position.x, position.y, NEIGHBOR_DIST * 2, NEIGHBOR_DIST * 2); // Neighbor radius
    }

    if (influences) {
      // Show influence lines
      showInfluences(boids);
    }
  }
  
  void drawVector(PVector v, float scayl) {
    pushMatrix();
    float arrowsize = 4;
    // Translate to boid location
    translate(position.x, position.y);
    stroke(255, 0, 0);
    // Line from origin
    line(0, 0, v.x * scayl, v.y * scayl);
    popMatrix();
}

  void showInfluences(ArrayList<Boid> boids) {
    for (Boid other : boids) {
      float d = PVector.dist(position, other.position);

      // Separation influence
      if ((d > 0) && (d < DESIRED_SEPARATION)) {
        stroke(255, 0, 0, map(d, 0, DESIRED_SEPARATION, 255, 0)); // Red with intensity based on distance
        line(position.x, position.y, other.position.x, other.position.y);
      }

      //// Alignment influence
      if ((d > 0) && (d < NEIGHBOR_DIST)) {
        stroke(0, 255, 0, map(d, 0, NEIGHBOR_DIST, 255, 0)); // Green with intensity based on distance
        line(position.x, position.y, other.position.x, other.position.y);
      }

      //// Cohesion influence
      if ((d > 0) && (d < NEIGHBOR_DIST)) {
        stroke(0, 0, 255, map(d, 0, NEIGHBOR_DIST, 255, 0)); // Blue with intensity based on distance
        line(position.x, position.y, other.position.x, other.position.y);
      }
    }
  }




  // Wrap around the edges of the window
  void edges() {
    if (position.x > width) position.x = 0;
    if (position.x < 0) position.x = width;
    if (position.y > height) position.y = 0;
    if (position.y < 0) position.y = height;
  }
}