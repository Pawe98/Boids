class Obstacle { //<>//
  PVector position;   // Position of the obstacle
  float w;        // Width of the obstacle
  float h;       // Height of the obstacle

  // Constructor for rectangular obstacles
  Obstacle(float x, float y, float w, float h) {
    position = new PVector(x, y);
    this.w = w;
    this.h = h;
  }


  // Method to check if a ray intersects with the obstacle
  boolean intersectsRay(PVector rayStart, PVector ray) {
    return intersectsEllipse(rayStart, ray);
  }


  // Method to check if a ray intersects with an ellipse
  // Method to check if a ray intersects with an ellipse
  boolean intersectsEllipse(PVector rayStart, PVector ray) {
    // Normalize the ray direction
    PVector rayDirNorm = ray.copy().normalize();

    PVector diff = PVector.sub(rayStart, position);
    float a = (rayDirNorm.x * rayDirNorm.x) / ((w / 2) * (w / 2)) + (rayDirNorm.y * rayDirNorm.y) / ((h / 2) * (h / 2));
    float b = 2 * ((rayDirNorm.x * diff.x) / ((w / 2) * (w / 2)) + (rayDirNorm.y * diff.y) / ((h / 2) * (h / 2)));
    float c = (diff.x * diff.x) / ((w / 2) * (w / 2)) + (diff.y * diff.y) / ((h / 2) * (h / 2)) - 1;

    float discriminant = b * b - 4 * a * c;

    if (discriminant < 0) {
      return false;
    }

    float sqrtDiscriminant = sqrt(discriminant);
    float t1 = (-b - sqrtDiscriminant) / (2 * a);
    float t2 = (-b + sqrtDiscriminant) / (2 * a);

    // Check if any of the intersection points are within the ray length
    float rayLength = ray.mag(); // Get the length of the ray from rayDir
    if ((t1 >= 0 && t1 <= rayLength) || (t2 >= 0 && t2 <= rayLength)) {
      return true;
    }

    return false;
  }


  // Method to display the obstacle
  void display(PGraphics buffer) {
    fill(20, 20, 255);
    buffer.ellipse(position.x, position.y, w, h);
  }
}
