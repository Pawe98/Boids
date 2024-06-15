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

  //for edge wrap, knowing if it is duplicated for beeing near screen edge / in a buffer zone
  boolean duplicated = false;

  // Constructor initializes the boid's position and gives it a random velocity
  Boid(float x, float y) {
    position = new PVector(x, y);     // Set initial position
    velocity = PVector.random2D();    // Set random initial velocity
    velocity.setMag(random(2, 4));    // Set magnitude of velocity to a random value between 2 and 4
    acceleration = new PVector();     // Initialize acceleration to zero
    isLeader = false; // Initially not a leader
  }

  // Check if the boid is clickedneighborDist
  void checkClicked() {
    if (mousePressed && !prevMousePressed && dist(position.x, position.y, mouseX, mouseY) < boidSize && mouseButton == LEFT) {
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

    ArrayList<Boid> neighbors = getNeighbors(boids);

    PVector separation = separate(neighbors, leader);
    PVector alignment = align(neighbors, leader);
    PVector cohesion = cohere(neighbors, leader);

    PVector chaseLeader = chase(neighbors, leader);

    separation.mult(separationWeight);
    alignment.mult(alignmentWeight);
    cohesion.mult(cohesionWeight);
    chaseLeader.mult(leaderInfluenceWeightChase);

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
        if (inSight(other) && d < closestDist) {
          closestDist = d;
          closestLeader = other;
        }
      }
    }

    return closestLeader;
  }

  boolean inSight(Boid boid) {
    float d = PVector.dist(position, boid.position);
    if (d > 0 && d < neighborDist) {
      PVector toOther = PVector.sub(boid.position, position);
      float angle = PVector.angleBetween(velocity, toOther);
      if (degrees(angle) < fov / 2) {
        return true;
      }
    }
    return false;
  }

  ArrayList<Boid> getNeighbors(ArrayList<Boid> boids) {
    ArrayList<Boid> neighbors = new ArrayList<Boid>();
    for (Boid other : boids) {
      float d = PVector.dist(position, other.position);
      if (d > 0 && d < neighborDist) {
        neighbors.add(other);
      }
    }
    return neighbors;
  }

  PVector chase(ArrayList<Boid> boids, Boid leader) {
    if (leader != null) {
      float d = position.dist(leader.position);
      if (inSight(leader)) {
        // Add a component to steer towards the leader's position
        PVector desired = PVector.sub(leader.position, position);

        // Calculate the scaling factor inversely proportional to the distance
        float scale = map(d, desiredSeparation, neighborDist, 0, 1);

        desired.normalize();
        desired.mult(maxSpeed * scale); // Scale the desired velocity
        PVector steerTowardsLeader = PVector.sub(desired, velocity);
        steerTowardsLeader.limit(maxForce * scale); // Scale the steering force

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
        if ((d > 0) && (d < desiredSeparation)) {
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
      if ((d > 0) && d < desiredSeparation) {
        PVector diff = PVector.sub(position, leader.position);
        diff.normalize(); // Normalize to get direction
        diff.div(d);      // Weight by distance
        leaderInfluence = PVector.mult(diff, leaderInfluenceWeightSeparate);  // Apply leader influence weight
      }
    }

    // Average the steering vector by the number of close boids
    if (count > 0) {
      steer.div((float) count);
    }

    // As long as the steering vector is non-zero
    if (steer.mag() > 0) {
      steer.normalize();
      steer.mult(maxSpeed);  // Set magnitude to maximum speed
      steer.sub(velocity);     // Subtract current velocity to get the steering force
      steer.limit(maxForce);   // Limit to maximum steering force
    }

    // Combine leader influence if present
    if (leader != null && leaderInfluence.mag() > 0) {
      if (OVERRIDE_LIMITS_FOR_LEADER_INFLUENCE) {
        leaderInfluence.sub(velocity);
        steer.add(leaderInfluence); // Add leader's influence without limiting
        steer.limit(maxForce*(leaderInfluenceWeightSeparate / separationWeight));
      } else {
        leaderInfluence.sub(velocity);
        steer.add(leaderInfluence); // Combine with neighbors' steering
        steer.limit(maxForce); // Limit leader steering force
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
      if (other != leader && inSight(other)) {
        sum.add(other.velocity); // Add the velocity
        count++; // Increment count of neighboring boids
      }
    }

    // If a leader is found and within neighbor range, add its influence
    if (leader != null) {
      if (inSight(leader)) {
        leaderInfluence = PVector.mult(leader.velocity, leaderInfluenceWeightAlign); // Apply leader influence weight
        count += leaderInfluenceWeightAlign;
      }
    }

    if (count > 0) {
      sum.div((float) count);   // Average the velocity of neighbors
      sum.setMag(maxSpeed);     // Set magnitude to maximum speed for neighbors' influence
      PVector steer = PVector.sub(sum, velocity); // Calculate steering force from neighbors
      steer.limit(maxForce);

      if (leader != null && leaderInfluence.mag() > 0) {
        if (OVERRIDE_LIMITS_FOR_LEADER_INFLUENCE) {
          leaderInfluence.setMag(maxSpeed); // Set leader influence magnitude to max speed
          PVector leaderSteer = PVector.sub(leaderInfluence, velocity); // Calculate leader steering force
          PVector combined = PVector.add(steer, leaderSteer); // Combine with neighbors' steering
          combined.limit(maxForce*(leaderInfluenceWeightAlign / alignmentWeight));    // Limit to maximum steering force for neighbors
          return combined;
        } else {
          leaderInfluence.setMag(maxSpeed); // Set leader influence magnitude to max speed
          PVector leaderSteer = PVector.sub(leaderInfluence, velocity); // Calculate leader steering force
          PVector combined = PVector.add(steer, leaderSteer); // Combine with neighbors' steering
          combined.limit(maxForce);
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
      if (other != leader && inSight(other)) {
        sum.add(other.position); // Add the position
        count++; // Increment count of neighboring boids
      }
    }

    // If a leader is found and within neighbor range, add its influence
    if (leader != null) {
      if (inSight(leader)) {
        for (int i = 0; i <= leaderInfluenceWeightCohere; i++) {
          leaderInfluence = leader.position; // Apply leader influence weight
          count ++;
        }
      }
    }

    if (count > 0) {
      sum.div((float) count); // Average position of neighbors
      PVector steer = seek(sum); // Steering force towards average position
      steer.limit(maxForce);

      if (leader != null && leaderInfluence.mag() > 0) {
        if (OVERRIDE_LIMITS_FOR_LEADER_INFLUENCE) {
          PVector leaderSteer = seek(leaderInfluence); // Steering force towards leader's influenced position
          PVector combined = PVector.add(steer, leaderSteer); // Combine without limiting leader's influence
          combined.limit(maxForce*(leaderInfluenceWeightCohere / cohesionWeight));
          return combined;
        } else {
          PVector leaderSteer = seek(leaderInfluence); // Steering force towards leader's influenced position
          PVector combined = PVector.add(steer, leaderSteer); // Combine with neighbors' steering
          combined.limit(maxForce);
          return combined;
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
    desired.mult(maxSpeed);

    // Steering = Desired minus Velocity
    PVector steer = PVector.sub(desired, velocity);
    return steer;
  }

  // Update the boid's position based on its velocity and acceleration
  void update() {
    velocity.add(acceleration); // Update velocity
    velocity.limit(maxSpeed);  // Limit speed
    position.add(velocity);     // Update position
    acceleration.mult(0);       // Reset acceleration to zero
  }

  // Display the boid as a triangle and show different behaviors with colors in debug mode
  void display(PGraphics context, ArrayList<Boid> boids) {
    float theta = velocity.heading() + PI / 2; // Calculate heading angle
    if (isLeader) {
      // Display leader differently
      context.fill(255, 0, 0); // Red color for leader
    } else {
      context.fill(127); // Normal color for other boids
    }

    context.stroke(200);

    context.pushMatrix();
    context.translate(position.x, position.y);
    context.rotate(theta);
    context.beginShape();
    context.vertex(0, -boidSize * 2);
    context.vertex(-boidSize, boidSize * 2);
    context.vertex(boidSize, boidSize * 2);
    context.endShape(CLOSE);
    context.popMatrix();

    if (debug) {
      drawFOVCone(context, neighborDist);
      context.noFill();
      context.stroke(255, 0, 0);
      context.ellipse(position.x, position.y, desiredSeparation * 2, desiredSeparation * 2); // Separation radius
    }

    if (influences) {
      // Show influence lines
      showInfluences(context, boids);
    }
  }

  void drawFOVCone(PGraphics context, float radius) {
    // Calculate the angle in radians
    float halfFOV = radians(fov / 2);
    float angle = velocity.heading(); // Angle of the boid's velocity vector

    // Calculate the start and end angles for the arc
    float startAngle = angle - halfFOV;
    float endAngle = angle + halfFOV;

    // Set fill color with transparency (alpha)
    context.noFill(); // Green with alpha 100
    context.stroke(0,255,0, 100);

    // Begin drawing the shape
    context.beginShape();
    context.vertex(position.x, position.y); // Start at the boid's position

    // Draw vertices around the arc
    for (float a = startAngle; a <= endAngle; a += 0.05) { // Increment angle in small steps
      float x = position.x + cos(a) * radius;
      float y = position.y + sin(a) * radius;
      context.vertex(x, y);
    }

    // Close the shape by connecting back to the boid's position
    context.vertex(position.x, position.y);
    context.endShape(CLOSE);
  }


  void drawVector(PGraphics context, PVector v, float scayl) {
    context.pushMatrix();
    // Translate to boid location
    context.translate(position.x, position.y);
    // Line from origin
    context.line(0, 0, v.x * scayl, v.y * scayl);
    context.popMatrix();
  }

  void showInfluences(PGraphics context, ArrayList<Boid> boids) {
    for (Boid other : boids) {
      float d = PVector.dist(position, other.position);

      //// Alignment & Cohecion influence
      if ((d > 0) && (d < neighborDist) && inSight(other) && other.isLeader) {
        context.stroke(0, 255, 0, map(d, 0, neighborDist, 255, 0)); // Green with intensity based on distance
        context.strokeWeight(map(d, 0, neighborDist, 5, 0));
        drawVector(context, PVector.sub(other.position, position), 1.0f);
      }
    }

    for (Boid other : boids) {
      float d = PVector.dist(position, other.position);

      // Separation influence
      if ((d > 0) && (d < desiredSeparation) && other.isLeader) {
        context.stroke(255, 0, 0, map(d, 0, desiredSeparation, 255, 0)); // Red with intensity based on distance
        context.strokeWeight(map(d, 0, desiredSeparation, 15, 0));
        drawVector(context, PVector.sub(other.position, position), 1.0f);
      }
    }
    context.strokeWeight(1);
  }


  void edges() {
    float edgeRepelRange = 100; // Range from edge where repulsion force starts
    float edgeRepelStrength = 0.03; // Strength of the repulsion force

    // Apply edge repulsion forces
    if (position.x > width - edgeRepelRange) {
      float forceMagnitude = map(position.x, width - edgeRepelRange, width, 0, edgeRepelStrength);
      applyForce(new PVector(-forceMagnitude, 0)); // Apply force towards left
    }
    if (position.x < edgeRepelRange) {
      float forceMagnitude = map(position.x, 0, edgeRepelRange, edgeRepelStrength, 0);
      applyForce(new PVector(forceMagnitude, 0)); // Apply force towards right
    }
    if (position.y > height - edgeRepelRange) {
      float forceMagnitude = map(position.y, height - edgeRepelRange, height, 0, edgeRepelStrength);
      applyForce(new PVector(0, -forceMagnitude)); // Apply force towards top
    }
    if (position.y < edgeRepelRange) {
      float forceMagnitude = map(position.y, 0, edgeRepelRange, edgeRepelStrength, 0);
      applyForce(new PVector(0, forceMagnitude)); // Apply force towards bottom
    }
  }
}
