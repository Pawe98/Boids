import java.util.ArrayList; //<>// //<>// //<>//

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

  //for edge wrap, knowing if it is duplicated for being near screen edge / in a buffer zone
  boolean duplicated = false;

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
  void flock(ArrayList<Boid> boids, ArrayList<Obstacle> obstacles) {
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

    // Obstacle avoidance should override flocking behaviors if an obstacle is detected
    PVector avoidance = avoidObstacles(obstacles);
    if (avoidance != null) {
      applyForce(avoidance);
    }
  }

  // Method to find the closest leader in sight
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

        PVector separationForce = separate(boids, leader);
        PVector steerTowardsLeader = PVector.sub(desired, velocity);
        steerTowardsLeader.add(separationForce);
        return steerTowardsLeader;
      }
    }
    return new PVector(0, 0);
  }

  // Separation: Steer to avoid crowding local flockmates
  PVector separate(ArrayList<Boid> neighbors, Boid leader) {
    PVector steer = new PVector(0, 0); // Initialize the steering vector
    float count = 0;  // Number of boids that are too close

    // Loop through all neighbors
    for (Boid other : neighbors) {
      // Calculate distance between this boid and another boid
      float d = position.dist(other.position);

      // If the other boid is too close
      if ((d > 0) && (d < desiredSeparation)) {
        // Calculate vector pointing away from the other boid
        PVector diff = PVector.sub(position, other.position);

        if (other == leader) {
          diff.normalize(); // Normalize to get direction
          diff.div(d);      // Weight by distance
          steer.add(PVector.mult(diff, leaderInfluenceWeightSeparate));  // Apply leader influence weight
          count += leaderInfluenceWeightSeparate;
        } else {
          diff.normalize(); // Normalize to get direction
          diff.div(d);      // Weight by distance
          if (leader != null && inSight(leader)) {
            steer.add(PVector.mult(diff, ((leaderInfluenceWeightSeparate / separationWeight) + (leaderInfluenceWeightCohere / cohesionWeight) + (leaderInfluenceWeightAlign / alignmentWeight)) / 3));  // Apply leader influence weight
            count += leaderInfluenceWeightSeparate;
          } else {
            steer.add(diff);  // Add to steering vector
            count++; // Increment count of close boids
          }
        }
      }
    }

    // Average the steering vector by the number of close boids
    if (count > 0) {
      steer.div(count);
    }

    // As long as the steering vector is non-zero
    if (steer.mag() > 0) {
      steer.normalize();
      steer.mult(maxSpeed);  // Set magnitude to maximum speed
      steer.sub(velocity);     // Subtract current velocity to get the steering force
    }

    // Combine leader influence if present
    if (leader != null) {
      if (OVERRIDE_LIMITS_FOR_LEADER_INFLUENCE) {
        steer.limit(maxForce * (leaderInfluenceWeightSeparate / separationWeight));
        return steer;
      }
    }
    steer.limit(maxForce);
    return steer; // Return the calculated steering force
  }

  // Alignment: Steer towards the average heading of local flockmates
  PVector align(ArrayList<Boid> neighbors, Boid leader) {
    PVector sum = new PVector(0, 0); // Sum of all the velocities
    float count = 0;  // Number of boids that are neighbors

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
        sum.add(PVector.mult(leader.velocity, leaderInfluenceWeightAlign)); // Apply leader influence weight
        count += leaderInfluenceWeightAlign;
      }
    }

    if (count > 0) {
      sum.div(count);   // Average the velocity of neighbors
      sum.setMag(maxSpeed);
      PVector steer = PVector.sub(sum, velocity); // Calculate the steering force

      // Combine leader influence if present
      if (leader != null) {
        if (OVERRIDE_LIMITS_FOR_LEADER_INFLUENCE) {
          steer.limit(maxForce * (leaderInfluenceWeightAlign / alignmentWeight));
          return steer;
        }
      }

      steer.limit(maxForce);
      return steer;  // Return the steering force
    }
    return new PVector(0, 0); // Return zero vector if no neighbors
  }

  // Cohesion: Steer to move towards the average position of local flockmates
  PVector cohere(ArrayList<Boid> neighbors, Boid leader) {
    PVector sum = new PVector(0, 0); // Sum of all positions
    float count = 0;  // Number of boids that are neighbors

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
        sum.add(PVector.mult(leader.position, leaderInfluenceWeightCohere)); // Apply leader influence weight
        count += leaderInfluenceWeightCohere;
      }
    }

    if (count > 0) {
      sum.div(count); // Average the position of neighbors
      return seek(sum); // Steer towards the average position
    }
    return new PVector(0, 0); // Return zero vector if no neighbors
  }

  // Steer towards a target
  PVector seek(PVector target) {
    PVector desired = PVector.sub(target, position);  // A vector pointing from the position to the target
    desired.setMag(maxSpeed);
    PVector steer = PVector.sub(desired, velocity); // Calculate the steering force
    steer.limit(maxForce); // Limit the steering force to the maximum
    return steer; // Return the steering force
  }

  // Method to check for and avoid obstacles
  PVector avoidObstacles(ArrayList<Obstacle> obstacles) {
    // Number of rays to cast within the FOV
    int numRays = 16;
    // Angle increment for each ray
    float angleIncrement = radians(fov) / numRays;
    // Starting angle based on the boid's heading and FOV
    float startAngle = velocity.heading() - radians(fov) / 2;
    float maxDistance = neighborDist;

    // List to store open paths
    ArrayList<PVector> openPaths = new ArrayList<>();

    boolean foundObstacle = false;
    PVector avoidance = new PVector(); // Vector for avoidance direction

    // Loop through each ray direction
    for (int i = 0; i < numRays; i++) {
      // Calculate the direction of the current ray
      float angle = startAngle + angleIncrement * i;
      PVector ray = PVector.fromAngle(angle).setMag(maxDistance); // Set the magnitude of the ray to maxDistance

      // Check for intersections with obstacles
      boolean clear = true;
      for (Obstacle obstacle : obstacles) {
        if (obstacle.intersectsRay(position, ray)) {
          clear = false;
          foundObstacle = true;

          // Calculate the distance to the obstacle
          PVector diff = PVector.sub(position, obstacle.position);
          float distanceToObstacle = diff.mag();

          // Calculate a point on the obstacle closest to the boid
          PVector obstaclePosition = obstacle.position; // Example method to get obstacle position
          PVector closestPointOnObstacle = obstaclePosition.copy().add(ray); // Assuming obstacle position is its center

          // Calculate avoidance direction away from the obstacle
          PVector awayFromObstacle = PVector.sub(position, closestPointOnObstacle);

          // Scale the avoidance force based on distance
          float avoidanceStrength;
          if (distanceToObstacle < neighborDist) {
            // "Explode" avoidance force if distance is less than neighborDist
            avoidanceStrength = map(distanceToObstacle, 0, neighborDist, 10, 1);
          } else {
            // Normal scaling if distance is greater than or equal to neighborDist
            avoidanceStrength = 1 / distanceToObstacle;
          }

          awayFromObstacle.normalize();
          awayFromObstacle.mult(avoidanceStrength);

          avoidance.add(awayFromObstacle);

          break;
        }
      }

      // If the ray is clear, add it to the list of open paths
      if (clear) {
        openPaths.add(ray); // Store the ray direction itself
      }

      if (debug && influences) {
        stroke(clear ? color(0, 255, 0) : color(255, 0, 0));
        line(position.x, position.y, position.x + ray.x, position.y + ray.y);
      }
    }

    // If an obstacle was found, steer away from it
    if (foundObstacle) {
      // Normalize the avoidance vector
      avoidance.normalize();
      // Multiply by maximum speed to get desired velocity
      avoidance.mult(maxSpeed);
      avoidance.sub(velocity);
      avoidance.limit(maxForce*2);

      return avoidance;
    } else {
      return null;
    }
  }

  void drawVector(PGraphics context, PVector v, float scayl) {
    context.pushMatrix();
    // Translate to boid location
    context.translate(position.x, position.y);
    // Line from origin
    context.line(0, 0, v.x * scayl, v.y * scayl);
    context.popMatrix();
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
    context.stroke(0, 255, 0, 100);

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

  void update() {
    velocity.add(acceleration);
    velocity.limit(maxSpeed);
    position.add(velocity);
    acceleration.mult(0);  // Reset acceleration to zero after each update
  }

  /// Display the boid as a triangle and show different behaviors with colors in debug mode
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

  // Method to handle edge wrapping
  void edges() {
    if (position.x > width) position.x = 0;
    else if (position.x < 0) position.x = width;
    if (position.y > height) position.y = 0;
    else if (position.y < 0) position.y = height;
  }

  // Method to check if this boid is equal to another boid
  boolean equals(Boid other) {
    return position.equals(other.position) && velocity.equals(other.velocity);
  }
}
