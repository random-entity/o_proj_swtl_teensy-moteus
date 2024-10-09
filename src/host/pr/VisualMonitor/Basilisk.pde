public class Basilisk {
  int suid;
  float lpsx, lpsy;
  float yaw;
  float phil = 0.0, phir = 0.0;
  int mode;
  static final float hb = 30;
  static final float f = 15;
  static final float collthr = 100;

  public Basilisk(int _suid) {
    suid = _suid;
  }
  
  public void update(float _lpsx, float _lpsy) {}

  public void display() {
    pushMatrix();  // Enter Basilisk local xy (+y = yaw)
    translate(lpsx, lpsy);
    rotate((yaw - 0.25) * TWO_PI);

    pushMatrix();
    scale(1, -1);
    fill(0);
    textSize(15);
    text(suid, 0, 15);
    popMatrix();

    // Draw center, heading, bar, collision boundary
    fill(0);
    noStroke();
    ellipse(0, 0, 5, 5);
    stroke(0);
    line(0, 0, 0, 10);
    line(-hb, 0, hb, 0);
    noFill();
    stroke(0, 127);
    ellipse(0, 0, 2 * collthr, 2 * collthr);

    pushMatrix();  // Enter left foot local xy (+y = sig)
    translate(-hb, 0);
    rotate(-phil * TWO_PI);
    // Draw left foot
    stroke(0);
    line(0, 0, 0, f);
    fill(255, 0, 0);
    noStroke();
    ellipse(0, 0, 5, 5);
    popMatrix();  // Exit left foot local xy (+y = sig)

    pushMatrix();  // Enter right foot local xy (+y = sig)
    translate(hb, 0);
    rotate(-phir * TWO_PI);
    // Draw right foot
    stroke(0);
    line(0, 0, 0, f);
    fill(0, 0, 255);
    noStroke();
    ellipse(0, 0, 5, 5);
    popMatrix();  // Exit right foot local xy (+y = sig)

    popMatrix();  // Exit Basilisk local xy (+y = yaw)
  }
}
