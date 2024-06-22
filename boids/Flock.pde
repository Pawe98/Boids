import java.io.FileWriter;
import java.io.IOException;
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

  // Save boids' positions and visible neighbors to a CSV file, appending if the file exists
  // Save boids' positions and visible neighbors to a CSV file, appending if the file exists
  void saveToCSV(String filename) {
    try {
      // Determine if the file already exists
      boolean fileExists = new File(filename).exists();

      FileWriter writer = new FileWriter(filename, true);  // Append mode

      // If the file didn't exist before, write the header and configuration parameters
      if (!fileExists) {
        // Write configuration parameters
        writer.append("Configuration Parameters\n");
        writer.append("maxForce,maxSpeed,fov,desiredSeparation,neighborDist,separationWeight,");
        writer.append("alignmentWeight,cohesionWeight,");


        // Write values of configuration parameters
        writer.append(maxForce + ",");
        writer.append(maxSpeed + ",");
        writer.append(fov + ",");
        writer.append(desiredSeparation + ",");
        writer.append(neighborDist + ",");
        writer.append(separationWeight + ",");
        writer.append(alignmentWeight + ",");
        writer.append(cohesionWeight + ",");
        writer.append(leaderInfluenceWeightSeparate + ",");
        writer.append(leaderInfluenceWeightAlign + ",");
        writer.append(leaderInfluenceWeightCohere + ",");
        writer.append(leaderInfluenceWeightChase + "\n");

        // Write header for boids' data
        writer.append("\nFrame,Boid ID,Position X,Position Y,Velocity X,Velocity Y,Visible Neighbors\n");
      }

      // Write each boid's data
      for (int i = 0; i < boids.size(); i++) {
        Boid b = boids.get(i);
        writer.append(frameCounter + ",");
        writer.append(i + ",");
        writer.append(b.position.x + ",");
        writer.append(b.position.y + ",");
        writer.append(b.velocity.x + ",");
        writer.append(b.velocity.y + ",");
        writer.append("[");
        ArrayList<Boid> visibleNeighbors = b.getVisibleNeighbors(boids);
        for (int j = 0; j < visibleNeighbors.size(); j++) {
          Boid neighbor = visibleNeighbors.get(j);
          writer.append(boids.indexOf(neighbor) + "");
          if (j < visibleNeighbors.size() - 1) {
            writer.append(";");
          }
        }
        writer.append("]");
        writer.append("\n");
      }

      writer.flush();
      writer.close();
      System.out.println("Data appended to CSV file " + filename + " successfully!");
    }
    catch (IOException e) {
      e.printStackTrace();
    }
  }


  void display(PGraphics g) {
    for (Boid b : boids) {
      if (b != null) {
        b.display(g, boids); // Draw the boid
      }
    }
  }
}
