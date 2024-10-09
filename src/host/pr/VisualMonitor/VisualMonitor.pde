import processing.serial.*;

Serial xbPort;
XbRR rr;

// ALL lengths in cm
final float stageDimX = 860, stageDimY = 910;
float marginX, marginY;

ArrayList<Basilisk> bs;

void setup() {
  String[] ports = Serial.list();
  printArray(ports);
  xbPort = new Serial(this, "/dev/ttyUSB1", 115200);
  rr = new XbRR(xbPort);

  size(1000, 1000);
  frameRate(1000);

  marginX = (width - stageDimX) / 2;
  marginY = (height - stageDimY) / 2;

  bs = new ArrayList<Basilisk>();
  for (int suid = 1; suid <= 13; suid++) {
    bs.add(new Basilisk(suid));
  }
}

void draw() {
  rr.Run();

  background(255);

  pushMatrix();  // Enter stage local xy
  translate(0, height);
  scale(1, -1);
  translate(marginX, marginY);

  // Draw stage boundary
  noFill();
  stroke(0);
  rect(0, 0, stageDimX, stageDimY);

  // Draw LPS boundary
  rect(150, 150, 710 - 150, 760 - 150);

  // Draw Basilisks
  for (Basilisk b : bs) {
    b.display();
  }

  popMatrix();  // Exit stage local xy
}
