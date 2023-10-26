public class DivideAndConquerHungarianAlgorithm {

  private int N, matchingCardinality;
  private Point[] A, B;
  private double[] distance;
  private boolean[] visited;
  private double left, right, top, bottom, matchingCost;
  static final double slackThreshold = 0.0000001;
  static final double INFINITY = Double.MAX_VALUE;
  private int[] matching, mappingB, parent;

  /**
   * @param A - List of points in A
   * @param B - List of points in B
   */
  public DivideAndConquerHungarianAlgorithm(Point[] A, Point[] B) {
    this.A = A;
    this.B = B;
    N = A.length;

    // Clear the stale values
    for (int i = 0; i < N; i++) {
      this.A[i].matchId = -1;
      this.A[i].dual = 0;
      this.B[i].matchId = -1;
      this.B[i].dual = 0;
    }
  }

  /**
   * @param p1
   * @param p2
   * @return - Distance between the two points
   */
  private double getDistance(Point p1, Point p2) {
    return Math.sqrt(
      ((p1.x - p2.x) * (p1.x - p2.x)) +
      ((p1.y - p2.y) * (p1.y - p2.y)) +
      ((p1.z - p2.z) * (p1.z - p2.z))
    );
  }

  /**
   * @return - Matching cost
   */
  public double getMatchingCost() {
    return this.matchingCost;
  }

  /**
   * @return - Matching cardinality
   */
  public int getMatchingCardinality() {
    return this.matchingCardinality;
  }

  /**
   * @return - Min Cost Matching
   */
  public int[] getMatching() {
    return this.matching;
  }

  /**
   * Gets the matching cardinality and cost
   */
  private void getMatchingCardinalityAndCost() {
    matchingCardinality = 0;
    matchingCost = 0;
    matching = new int[N];
    for (int i = 0; i < N; i++) {
      if (A[i].matchId != -1) {
        matchingCardinality++;
        matchingCost += getDistance(A[i], B[A[i].matchId]);
        matching[i] = A[i].matchId;
      }
    }
  }

  /**
   * @param n - Number of points under consideration
   * @return - Index of the point that is not yet visited and has the minimum distance
   */
  private int getMinDistanceNode(int n) {
    double minDist = INFINITY;
    int minIdx = -1;
    for (int i = 0; i < n; i++) {
      if (!visited[i] && distance[i] < minDist) {
        minDist = distance[i];
        minIdx = i;
      }
    }
    return minIdx;
  }

  /**
   * @param b
   * @param bPoint
   * @return Minimum distance of the point from the boundary
   */
  private double getMinDistanceToBoundary(Boundary b, Point bPoint) {
    double topDist = b.getTop() - bPoint.y;
    double rightDist = b.getRight() - bPoint.x;
    double bottomDist = bPoint.y - b.getBottom();
    double leftDist = bPoint.x - b.getLeft();
    return Math.min(
      Math.min(topDist, bottomDist),
      Math.min(rightDist, leftDist)
    );
  }

  private void checkBoundaryDistance(Point bPoint, int u, Boundary b) {
    double slack = getMinDistanceToBoundary(b, bPoint) - bPoint.dual;
    if (Math.abs(slack) <= slackThreshold) {
      slack = 0;
    }
    if (distance[0] > distance[u] + slack) {
      distance[0] = distance[u] + slack;
      parent[0] = u;
    }
  }

  private void performNormalCall(Point bPoint, Point[] A, int u, int m) {
    for (int v = 1; v < m + 1; v++) {
      if (bPoint.matchId != A[v - 1].id) {
        double slack =
          getDistance(A[v - 1], bPoint) - A[v - 1].dual - bPoint.dual;
        if (Math.abs(slack) <= slackThreshold) {
          slack = 0;
        }
        if (distance[v] > distance[u] + slack) {
          distance[v] = distance[u] + slack;
          parent[v] = u;
        }
      }
    }
  }

  /**
   * Augments the matching and updates the dual weights
   * @param A - List of points in set A
   * @param B - List of points in set B
   * @param b - Boundary
   */
  void hungarianSearch(
    Point[] A,
    Point[] B,
    Boundary b,
    int idx
  ) {
    int m = A.length;
    int k = m + B.length + 1;

    // Initialize the distance and visited arrays
    for (int i = 0; i < k; i++) {
      visited[i] = false;
      distance[i] = INFINITY;
      parent[i] = -1;
    }

    // Distance from the source to itself is 0
    distance[m + idx + 1] = 0.0;
    double lMin = INFINITY;
    int freeMinDistanceIdxInA = -1;

    checkBoundaryDistance(B[idx], m + idx + 1, b);
    performNormalCall(B[idx], A, m + idx + 1, m);

    // Conduct Dijsktra's until a free point in set A or a boundary point is found
    while (true) {
      int u = getMinDistanceNode(m + 1);

      // Stop as soon as a free point in A or a boundary point is found
      if (u == 0 || A[u - 1].matchId == -1) {
        freeMinDistanceIdxInA = u;
        lMin = distance[u];
        break;
      }

      // Mark u as visited
      visited[u] = true;

      // Update the distances of the neighbours of u
      // u is a matched point in A and will have only one neighbor
      int w = mappingB[A[u - 1].matchId];
      distance[w] = distance[u];
      parent[w] = u;
      u = w;

      // u is now a point of B
      // All the boundaries and unmatched points in A can be reached from u
      checkBoundaryDistance(B[u - m - 1], u, b);
      performNormalCall(B[u - m - 1], A, u, m);
    }

    // Get the augmenting path
    int pathArraySize = 0;
    int freeMinDistanceIdxInACopy = freeMinDistanceIdxInA;
    while (freeMinDistanceIdxInACopy > -1) {
      pathArraySize++;
      freeMinDistanceIdxInACopy = parent[freeMinDistanceIdxInACopy];
    }
    int[] path = new int[pathArraySize];
    int index = 0;
    while (freeMinDistanceIdxInA > -1) {
      path[index++] = freeMinDistanceIdxInA;
      freeMinDistanceIdxInA = parent[freeMinDistanceIdxInA];
    }

    // Update the matching (Augmenting the path)
    for (int i = 0; i < pathArraySize - 1; i += 2) {
      if (path[i] == 0) {
        B[path[i + 1] - m - 1].matchId = -2;
      } else {
        A[path[i] - 1].matchId = B[path[i + 1] - m - 1].id;
        B[path[i + 1] - m - 1].matchId = A[path[i] - 1].id;
      }
    }

    // Update the dual weights
    for (int i = 1; i < k; i++) {
      if (distance[i] < lMin) {
        if (i >= m + 1) {
          B[i - m - 1].dual = B[i - m - 1].dual + (lMin - distance[i]); // dual weight update of points in set B
        } else {
          A[i - 1].dual = A[i - 1].dual - (lMin - distance[i]); // dual weight update of points in set A
        }
      }
    }
  }

  /**
   * Solver function
   * @param A - List of points in set A
   * @param B - List of points in set B
   * @param b - Boundary
   */
  void solverHelper(Point[] A, Point[] B, Boundary b) {
    Boundary bNew = new Boundary(
      b.getTop(),
      b.getBottom(),
      b.getLeft(),
      b.getRight()
    );
    if (b.getLeft() == left) {
      bNew.setLeft(-INFINITY);
    }
    if (b.getRight() == right) {
      bNew.setRight(INFINITY);
    }
    if (b.getTop() == top) {
      bNew.setTop(INFINITY);
    }
    if (b.getBottom() == bottom) {
      bNew.setBottom(-INFINITY);
    }

    // If no points of set B are present or both left and right or both top and bottom boundaries are the same, do nothing
    if (B.length == 0) {
      return;
    }

    // If two opposite boundaries coincide, do nothing
    if (
      b.getTop() - b.getBottom() < slackThreshold ||
      b.getRight() - b.getLeft() < slackThreshold
    ) {
      return;
    }

    // If no points of set A are present, update the dual weights of all points in set B to be the shortest distance to the boundary
    if (A.length == 0) {
      for (int i = 0; i < B.length; i++) {
        B[i].dual = getMinDistanceToBoundary(bNew, B[i]);
      }
      return;
    }

    // If only one point of set B is present, match it to the nearest point or the boundary
    if (B.length == 1) {
      double minDist = getMinDistanceToBoundary(bNew, B[0]);

      int matchedPointIdx = -1;
      for (int i = 0; i < A.length; i++) {
        if (getDistance(A[i], B[0]) <= minDist) {
          minDist = getDistance(A[i], B[0]);
          matchedPointIdx = i;
        }
      }
      if (matchedPointIdx != -1) {
        B[0].matchId = A[matchedPointIdx].id;
        A[matchedPointIdx].matchId = B[0].id;
      }
      B[0].dual = minDist;
      return;
    }

    // Else split the box into 4 smaller boxes and recursively solve for the smaller subproblems
    // Divide Step
    double xSplit = (b.getLeft() + b.getRight()) / 2;
    double ySplit = (b.getTop() + b.getBottom()) / 2;
    int a1 = 0, a2 = 0, a3 = 0, a4 = 0, b1 = 0, b2 = 0, b3 = 0, b4 = 0;

    // Assign the boxes to each point in set A
    for (int i = 0; i < A.length; i++) {
      Point p = A[i];
      if (p.x < xSplit) {
        if (p.y > ySplit) {
          a1++;
        } else {
          a3++;
        }
      } else {
        if (p.y > ySplit) {
          a2++;
        } else {
          a4++;
        }
      }
    }

    Point[] A1 = new Point[a1];
    Point[] A2 = new Point[a2];
    Point[] A3 = new Point[a3];
    Point[] A4 = new Point[a4];

    int i1 = 0, i2 = 0, i3 = 0, i4 = 0;
    for (int i = 0; i < A.length; i++) {
      Point p = A[i];
      if (p.x < xSplit) {
        if (p.y > ySplit) {
          A1[i1++] = p;
        } else {
          A3[i3++] = p;
        }
      } else {
        if (p.y > ySplit) {
          A2[i2++] = p;
        } else {
          A4[i4++] = p;
        }
      }
    }

    // Assign the boxes to each point in set B
    for (int i = 0; i < B.length; i++) {
      Point p = B[i];
      if (p.x < xSplit) {
        if (p.y > ySplit) {
          b1++;
        } else {
          b3++;
        }
      } else {
        if (p.y > ySplit) {
          b2++;
        } else {
          b4++;
        }
      }
    }

    i1 = 0;
    i2 = 0;
    i3 = 0;
    i4 = 0;
    Point[] B1 = new Point[b1];
    Point[] B2 = new Point[b2];
    Point[] B3 = new Point[b3];
    Point[] B4 = new Point[b4];
    for (int i = 0; i < B.length; i++) {
      Point p = B[i];
      if (p.x < xSplit) {
        if (p.y > ySplit) {
          B1[i1++] = p;
        } else {
          B3[i3++] = p;
        }
      } else {
        if (p.y > ySplit) {
          B2[i2++] = p;
        } else {
          B4[i4++] = p;
        }
      }
    }

    // Get the new boundaries for the 4 subproblems
    Boundary boundary1 = new Boundary(b.getTop(), ySplit, b.getLeft(), xSplit);
    Boundary boundary2 = new Boundary(b.getTop(), ySplit, xSplit, b.getRight());
    Boundary boundary3 = new Boundary(
      ySplit,
      b.getBottom(),
      b.getLeft(),
      xSplit
    );
    Boundary boundary4 = new Boundary(
      ySplit,
      b.getBottom(),
      xSplit,
      b.getRight()
    );

    // Solve the 4 subproblems independently
    solverHelper(A1, B1, boundary1);
    solverHelper(A2, B2, boundary2);
    solverHelper(A3, B3, boundary3);
    solverHelper(A4, B4, boundary4);

    for (int i = 0; i < B.length; i++) {
      mappingB[B[i].id] = i + A.length + 1;
    }

    // Perform hungarian searches for the points matched to the boundaries
    // Conquer Step
    for (int i = 0; i < B.length; i++) {
      if (B[i].matchId >= 0) {
        continue;
      }

      // Match to the boundary if possible
      double minDist = getMinDistanceToBoundary(bNew, B[i]);
      if (Math.abs(minDist - B[i].dual) <= slackThreshold) {
        B[i].matchId = -2;
        continue;
      }
      hungarianSearch(A, B, bNew, i);
    }
  }

  /**
   * @param b - Boundary
   * @return - Minimum cost for exact euclidean bipartite matching of points in A and B
   */
  public void solver(Boundary b) {
    left = b.getLeft();
    right = b.getRight();
    top = b.getTop();
    bottom = b.getBottom();
    distance = new double[2 * N + 1];
    parent = new int[2 * N + 1];
    visited = new boolean[2 * N + 1];
    mappingB = new int[N];
    solverHelper(A, B, b);
    getMatchingCardinalityAndCost();
  }
}
