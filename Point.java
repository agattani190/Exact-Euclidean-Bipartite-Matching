public class Point {

  public double x, y, z, dual;
  public int id, matchId;

  Point(double x, double y, int id) {
    this.x = x;
    this.y = y;
    this.z = 0;
    this.id = id;
    this.dual = 0.0;
    this.matchId = -1;
  }

  Point(double x, double y, double z, int id) {
    this.x = x;
    this.y = y;
    this.z = z;
    this.id = id;
    this.dual = 0.0;
    this.matchId = -1;
  }

  @Override
  public String toString() {
    return String.format(
      "X: " +
      this.x +
      " Y: " +
      this.y +
      " Z: " +
      this.z +
      " Dual Weight: " +
      this.dual +
      " Id: " +
      this.id +
      " Match Id: " +
      this.matchId
    );
  }
}
