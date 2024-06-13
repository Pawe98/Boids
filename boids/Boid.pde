import java.util.List;
import java.util.Arrays;


class Boid {
  PVector position;   // Position of the boid
  PVector velocity;   // Velocity of the boid
  PVector acceleration; // Acceleration of the boid

  boolean isLeader = false; // Indicates if the boid is a leader
  boolean isClicked = false; // Indicates if the boid is clicked
  boolean isControlled = false; //Indicates if the boid is influenced by the flock
  boolean prevMousePressed = false; // Previous mouse pressed state




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
    if (mousePressed && !prevMousePressed && dist(position.x, position.y, mouseX, mouseY) < BOID_SIZE && mouseButton == LEFT) {
      if (!isControlled) {
        isLeader = !isLeader; // Toggle leader status
      }
      isClicked = true; // Set clicked status
    }

    if (!mousePressed && mouseButton == LEFT) {
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
    Boid leader = null;
    if (!isLeader) {
      leader = findClosestLeader(boids);
    }

    ArrayList<Boid> neighbors = getNeighbors(boids, NEIGHBOR_DIST);

    PVector separation = separate(neighbors, leader);
    PVector alignment = align(neighbors, leader);
    PVector cohesion = cohere(neighbors, leader);

    PVector chaseLeader = chase(neighbors, leader);

    separation.mult(SEPARATION_WEIGHT);
    alignment.mult(ALIGNMENT_WEIGHT);
    cohesion.mult(COHESION_WEIGHT);
    chaseLeader.mult(LEADER_INFLUENCE_WEIGHT_CHASE);

    if (!isControlled) {
      applyForce(separation);
      applyForce(alignment);
      applyForce(cohesion);
      applyForce(chaseLeader);
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

  ArrayList<Boid> getNeighbors(ArrayList<Boid> boids, float radius) {
    ArrayList<Boid> neighbors = new ArrayList<Boid>();
    for (Boid other : boids) {
      float d = PVector.dist(position, other.position);
      if ((d > 0) && (d < radius)) {
        neighbors.add(other);
      }
    }
    return neighbors;
  }

  PVector chase(ArrayList<Boid> boids, Boid leader) {
    if (leader != null) {
      float d = position.dist(leader.position);
      if ((d > 0) && (d < NEIGHBOR_DIST) && (d > DESIRED_SEPARATION)) {
        // Add a component to steer towards the leader's position
        PVector desired = PVector.sub(leader.position, position);

        // Calculate the scaling factor inversely proportional to the distance
        float scale = map(d, DESIRED_SEPARATION, NEIGHBOR_DIST, 0, 1);

        desired.normalize();
        desired.mult(MAX_SPEED * scale); // Scale the desired velocity
        PVector steerTowardsLeader = PVector.sub(desired, velocity);
        steerTowardsLeader.limit(MAX_FORCE * scale); // Scale the steering force

        //fill(255, 0, 0);
        //ellipse(position.x + steerTowardsLeader.x * 1000, position.y + steerTowardsLeader.y * 1000, 20, 20);

        PVector separationForce = separate(boids, leader);
        steerTowardsLeader.add(separationForce);
        return steerTowardsLeader;
      }
    }
    return new PVector(0, 0);
  }

  // Separation: Steer to avoid crowding local flockmates
  PVector separate(ArrayList<Boid> neighbors, Boid leader) {
    PVector steer = new PVector(0, 0); // Initialize the steering vector
    PVector leaderInfluence = new PVector(0, 0); // Influence of the leader
    int count = 0;  // Number of boids that are too close

    // Loop through all neighbors
    for (Boid other : neighbors) {
        if (other != leader) {
            // Calculate distance between this boid and another boid
            float d = position.dist(other.position);

            // If the other boid is too close
            if ((d > 0) && (d < DESIRED_SEPARATION)) {
                // Calculate vector pointing away from the other boid
                PVector diff = PVector.sub(position, other.position);
                diff.normalize(); // Normalize to get direction
                diff.div(d);      // Weight by distance
                steer.add(diff);  // Add to steering vector
                count++; // Increment count of close boids
            }
        }
    }

    // If a leader is found and within separation range, add its influence
    if (leader != null) {
        float d = position.dist(leader.position);
        if ((d > 0) && d < DESIRED_SEPARATION) {
            PVector diff = PVector.sub(position, leader.position);
            diff.normalize(); // Normalize to get direction
            diff.div(d);      // Weight by distance
            leaderInfluence = PVector.mult(diff, LEADER_INFLUENCE_WEIGHT_SEPARATE);  // Apply leader influence weight
        }
    }

    // Average the steering vector by the number of close boids
    if (count > 0) {
        steer.div((float) count);
    }

    // As long as the steering vector is non-zero
    if (steer.mag() > 0) {
        steer.normalize();
        steer.mult(MAX_SPEED);  // Set magnitude to maximum speed
        steer.sub(velocity);     // Subtract current velocity to get the steering force
        steer.limit(MAX_FORCE);   // Limit to maximum steering force
    }

    // Combine leader influence if present
    if (leader != null && leaderInfluence.mag() > 0) {
        if (OVERRIDE_LIMITS_FOR_LEADER_INFLUENCE) {
          leaderInfluence.limit(MAX_FORCE*(LEADER_INFLUENCE_WEIGHT_SEPARATE / SEPARATION_WEIGHT));
            steer.add(leaderInfluence); // Add leader's influence without limiting
        } else {
            leaderInfluence.limit(MAX_FORCE); // Limit leader steering force
            steer.add(leaderInfluence); // Combine with neighbors' steering
        }
    }

    return steer; // Return the calculated steering force
}



  // Alignment: Steer towards the average heading of local flockmates
  PVector align(ArrayList<Boid> neighbors, Boid leader) {
    PVector sum = new PVector(0, 0); // Sum of all the velocities
    PVector leaderInfluence = new PVector(0, 0); // Influence of the leader's velocity
    int count = 0;  // Number of boids that are neighbors

    // Loop through all neighbors
    for (Boid other : neighbors) {
      if (other != leader) {
        sum.add(other.velocity); // Add the velocity
        count++; // Increment count of neighboring boids
      }
    }

    // If a leader is found and within neighbor range, add its influence
    if (leader != null) {
      float d = position.dist(leader.position);
      if ((d > 0) && (d < NEIGHBOR_DIST)) {
        leaderInfluence = PVector.mult(leader.velocity, LEADER_INFLUENCE_WEIGHT_ALIGN); // Apply leader influence weight
        count += LEADER_INFLUENCE_WEIGHT_ALIGN;
      }
    }

    if (count > 0) {
      sum.div((float) count);   // Average the velocity of neighbors
      sum.setMag(MAX_SPEED);     // Set magnitude to maximum speed for neighbors' influence
      PVector steer = PVector.sub(sum, velocity); // Calculate steering force from neighbors
      steer.limit(MAX_FORCE);    // Limit to maximum steering force for neighbors

      if (leader != null && leaderInfluence.mag() > 0) {
        if (OVERRIDE_LIMITS_FOR_LEADER_INFLUENCE) {
          leaderInfluence.setMag(MAX_SPEED); // Set leader influence magnitude to max speed
          PVector leaderSteer = PVector.sub(leaderInfluence, velocity); // Calculate leader steering force
          leaderSteer.limit(MAX_FORCE*(LEADER_INFLUENCE_WEIGHT_ALIGN / ALIGNMENT_WEIGHT)); // Limit leader steering force
          PVector combined = PVector.add(steer, leaderSteer); // Combine with neighbors' steering
          return combined;
        } else {
          leaderInfluence.setMag(MAX_SPEED); // Set leader influence magnitude to max speed
          PVector leaderSteer = PVector.sub(leaderInfluence, velocity); // Calculate leader steering force
          leaderSteer.limit(MAX_FORCE); // Limit leader steering force
          PVector combined = PVector.add(steer, leaderSteer); // Combine with neighbors' steering
          return combined;
        }
      }

      return steer; // Return the calculated steering force
    } else {
      return new PVector(0, 0); // If no neighbors, return zero steering force
    }
  }

  // Cohesion: Steer to move towards the average position of local flockmates
  PVector cohere(ArrayList<Boid> neighbors, Boid leader) {
    PVector sum = new PVector(0, 0); // Sum of all positions
    PVector leaderInfluence = new PVector(0, 0); // Influence of the leader's position
    int count = 0;  // Number of boids that are neighbors

    // Loop through all neighbors
    for (Boid other : neighbors) {
      if (other != leader) {
        sum.add(other.position); // Add the position
        count++; // Increment count of neighboring boids
      }
    }

    // If a leader is found and within neighbor range, add its influence
    if (leader != null) {
      float d = position.dist(leader.position);
      if ((d > 0) && (d < NEIGHBOR_DIST)) {
        leaderInfluence = PVector.mult(leader.position, LEADER_INFLUENCE_WEIGHT_COHERE); // Apply leader influence weight
        count += LEADER_INFLUENCE_WEIGHT_COHERE;
      }
    }

    if (count > 0) {
      sum.div((float) count); // Average position of neighbors
      PVector steer = seek(sum); // Steering force towards average position
      steer.limit(MAX_FORCE);

      if (leader != null && leaderInfluence.mag() > 0) {
        if (OVERRIDE_LIMITS_FOR_LEADER_INFLUENCE) {
          PVector leaderSteer = seek(leaderInfluence); // Steering force towards leader's influenced position
           leaderSteer.limit(MAX_FORCE*(LEADER_INFLUENCE_WEIGHT_COHERE / COHESION_WEIGHT));
          return PVector.add(steer, leaderSteer); // Combine without limiting leader's influence
        } else {
          PVector leaderSteer = seek(leaderInfluence); // Steering force towards leader's influenced position
          leaderSteer.limit(MAX_FORCE); // Limit leader steering force
          return PVector.add(steer, leaderSteer); // Combine with neighbors' steering
        }
      }

      return steer; // Return the calculated steering force
    } else {
      return new PVector(0, 0); // If no neighbors, return zero steering force
    }
  }

  // A method that calculates and applies a steering force towards a target
  // STEER = DESIRED MINUS VELOCITY
  PVector seek(PVector target) {
    PVector desired = PVector.sub(target, position);  // A vector pointing from the position to the target
    // Scale to maximum speed
    desired.normalize();
    desired.mult(MAX_SPEED);

    // Steering = Desired minus Velocity
    PVector steer = PVector.sub(desired, velocity);
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

    if (influences && isLeader) {
      // Show influence lines
      showInfluences(boids);
    }
  }

  void drawVector(PVector v, float scayl) {
    pushMatrix();
    // Translate to boid location
    translate(position.x, position.y);
    // Line from origin
    line(0, 0, v.x * scayl, v.y * scayl);
    popMatrix();
  }

  void showInfluences(ArrayList<Boid> boids) {
    for (Boid other : boids) {
      float d = PVector.dist(position, other.position);

      //// Alignment & Cohecion influence
      if ((d > 0) && (d < NEIGHBOR_DIST)) {
        stroke(0, 255, 0, map(d, 0, NEIGHBOR_DIST, 255, 0)); // Green with intensity based on distance
        strokeWeight(map(d, 0, NEIGHBOR_DIST, 5, 0));
        drawVector(PVector.sub(other.position, position), 1.0f);
      }
    }

    for (Boid other : boids) {
      float d = PVector.dist(position, other.position);

      // Separation influence
      if ((d > 0) && (d < DESIRED_SEPARATION)) {
        stroke(255, 0, 0, map(d, 0, DESIRED_SEPARATION, 255, 0)); // Red with intensity based on distance
        strokeWeight(map(d, 0, DESIRED_SEPARATION, 15, 0));
        drawVector(PVector.sub(other.position, position), 1.0f);
      }
    }
    strokeWeight(1);
  }

  // Wrap around the edges of the window
  void edges() {
    if (position.x > width) {
      velocity.x = -velocity.x; // flip x velocity
      position.x = width; // reposition inside the boundary
    }
    if (position.x < 0) {
      velocity.x = -velocity.x; // flip x velocity
      position.x = 0; // reposition inside the boundary
    }
    if (position.y > height) {
      velocity.y = -velocity.y; // flip y velocity
      position.y = height; // reposition inside the boundary
    }
    if (position.y < 0) {
      velocity.y = -velocity.y; // flip y velocity
      position.y = 0; // reposition inside the boundary
    }
  }
}
