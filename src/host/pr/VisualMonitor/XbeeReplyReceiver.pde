public class XbRR {
  private Serial port;
  private int[] rbuf = new int[46];
  private int bufIdx = 0;
  private boolean receiving = false;
  private int start = 0;
  private int start_time_ms;
  private boolean got_full_packet = false;

  public XbRR(Serial _port) {
    port = _port;
  }

  public void Run() {
    if (!receiving) {
      while (port.available() > 0 && start < 4) {
        int rbyte = port.read();
        if (rbyte == 255) {
          start++;
        } else {
          start = 0;
        }
      }

      if (start < 4) return;

      receiving = true;
      bufIdx = 0;
      got_full_packet = false;
      start_time_ms = millis();
    }

    if (millis() > start_time_ms + 10) {
      receiving = false;
      start = 0;
      return;
    }

    while (port.available() > 0 && bufIdx < 46) {
      rbuf[bufIdx] = port.read();
      bufIdx++;
    }
    if (bufIdx < 46) return;

    got_full_packet = true;

    if ((rbuf[2] & 16) != 0) {
      for (int suid = 1; suid <= 13; suid++) {
        if ((bytesToUint16(rbuf, 0) & (1 << (suid - 1))) != 0) {
          //print("Got Reply from SUID " + suid);
          Basilisk b = bs.get(suid - 1);
          b.mode = rbuf[3];
          b.lpsx = bytesToFloat(rbuf, 4);
          b.lpsy = bytesToFloat(rbuf, 8);
          b.yaw = bytesToFloat(rbuf, 12);
          b.phil = bytesToFloat(rbuf, 16);
          b.phir = bytesToFloat(rbuf, 20);

          break;
        }
      }

      println();
    }

    receiving = true;
    start = 0;
  }
}
