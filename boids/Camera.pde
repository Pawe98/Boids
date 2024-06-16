import javax.swing.JFrame;
import processing.awt.PSurfaceAWT;
import processing.awt.PSurfaceAWT.SmoothCanvas;


class Camera extends PApplet {
  //JFrame frame;
  Flock flock;
  Boid target;
  PGraphics pg;
  int startTime;
  int cameraWindowX = 800;  // X-Position des Kamera-Fensters
  int cameraWindowY = 100;  // Y-Position des Kamera-Fensters
  int cameraWindowWidth = 400;  // Breite des Kamera-Fensters
  int cameraWindowHeight = 400;  // Höhe des Kamera-Fensters


  public Camera(PGraphics pg, Flock flock, Boid target) {
    super();
    this.flock = flock;
    this.target = target;
    this.pg = pg;
    PApplet.runSketch(new String[]{this.getClass().getName()}, this);
  }

  public void settings() {
    size(cameraWindowWidth, cameraWindowHeight);
  }

  public void setup() {
    windowTitle("Camera");
    setupWindowPosition(1450, 0); 
    startTime = millis();
  }

  public void draw() {
    image(pg, 0, 0);
    fill(255);  // Set text color to white
    textSize(16);  // Set text size
    text("Frame: " + frameCounter, 10, 40);
  }

  void setupWindowPosition(int x, int y) {
    PSurfaceAWT surf = (PSurfaceAWT) getSurface();
    PSurfaceAWT.SmoothCanvas canvas = (PSurfaceAWT.SmoothCanvas) surf.getNative();
    JFrame window = (JFrame) canvas.getFrame();
    window.setLocation(x, y);
  }
}
