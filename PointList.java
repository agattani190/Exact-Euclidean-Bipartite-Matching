public class PointList {

  private Point[] points;
  private int size;

  PointList(int capacity) {
    if (capacity < 1) capacity = 1;
    points = new Point[capacity];
    size = 0;
  }

  public void add(Point point) {
    if (size == points.length) {
      Point[] new_array = new Point[2 * size];
      for (int i = 0; i < size; i++) {
        new_array[i] = points[i];
      }
      points = new_array;
    }
    points[size++] = point;
  }

  public Point[] getPoints() {
    if (size != points.length) {
      Point[] new_array = new Point[size];
      for (int i = 0; i < size; i++) {
        new_array[i] = points[i];
      }
      points = new_array;
    }
    return points;
  }
}
