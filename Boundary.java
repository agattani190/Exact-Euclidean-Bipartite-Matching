public class Boundary {

  private double top, bottom, left, right, front, back;

  Boundary(double top, double bottom, double left, double right) {
    this.top = top;
    this.bottom = bottom;
    this.left = left;
    this.right = right;
    this.front = 0;
    this.back = 0;
  }

  Boundary(
    double top,
    double bottom,
    double left,
    double right,
    double front,
    double back
  ) {
    this.top = top;
    this.bottom = bottom;
    this.left = left;
    this.right = right;
    this.front = front;
    this.back = back;
  }

  public double getTop() {
    return this.top;
  }

  public void setTop(double top) {
    this.top = top;
  }

  public double getBottom() {
    return this.bottom;
  }

  public void setBottom(double bottom) {
    this.bottom = bottom;
  }

  public double getLeft() {
    return this.left;
  }

  public void setLeft(double left) {
    this.left = left;
  }

  public double getRight() {
    return this.right;
  }

  public void setRight(double right) {
    this.right = right;
  }

  public double getFront() {
    return this.front;
  }

  public void setFront(double front) {
    this.front = front;
  }

  public double getBack() {
    return this.back;
  }

  public void setBack(double back) {
    this.back = back;
  }

  @Override
  public String toString() {
    return String.format(
      "Top: " +
      this.top +
      " Botttom: " +
      this.bottom +
      " Left: " +
      this.left +
      " Right: " +
      this.right +
      " Front: " +
      this.front +
      " Back: " +
      this.back
    );
  }
}
