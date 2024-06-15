class Camera extends PApplet {
  //JFrame frame;
  Flock flock;
  Boid target;
  PGraphics pg;

  public Camera(PGraphics pg, Flock flock, Boid target) {
    super();
    this.flock = flock;
    this.target = target;
    this.pg = pg;
    PApplet.runSketch(new String[]{this.getClass().getName()}, this);
  }

  public void settings() {
    size(400, 400);
  }

  public void setup() {
    windowTitle("Camera");
  }

  public void draw() {
    image(pg, 0, 0);
  }
}
