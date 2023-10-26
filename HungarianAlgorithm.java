import java.util.Arrays;

public class HungarianAlgorithm {

  private int N, matchingCardinality;
  private double matchingCost;
  private Point[] A, B;
  private int[] matching, parent;
  private static double slackThreshold = 0.0000001;
  private static double INFINITY = Double.MAX_VALUE;
  private double[] distance;
  private boolean[] visited;

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
   * Constructor to get minimum cost perfect matching
   * @param A - List of nodes in set A
   * @param B - List of nodes in set B
   */
  public HungarianAlgorithm(
    Point[] A,
    Point[] B
    
  ) {
    this.A = A;
    this.B = B;
    N = A.length;

    // Clear the stale values
    for (int i = 0; i < N; i++) {
      A[i].matchId = -1;
      A[i].dual = 0;
      B[i].matchId = -1;
      B[i].dual = 0;
    }
  }

  /**
   * @return - Index of the point that is not yet visited and has the minimum distance
   */
  private int getMinDistanceNode() {
    double minDist = INFINITY;
    int minIdx = -1;
    for (int i = 0; i < visited.length; i++) {
      if (!visited[i] && distance[i] < minDist) {
        minDist = distance[i];
        minIdx = i;
      }
    }
    return minIdx;
  }

  /**
   * Augments the matching by 1 and updates the dual weights.
   */
  private void hungarianSearch() {
    int n = 2 * N;

    // Initialize the distance and visited arrays
    Arrays.fill(visited, false);
    Arrays.fill(distance, INFINITY);
    Arrays.fill(parent, -1);
    double lMin = INFINITY;
    int freeMinDistanceIdxInA = -1;

    // Distance to every free point of B is 0
    for (int i = 0; i < N; i++) {
      Point bPoint = B[i];
      if (bPoint.matchId == -1) {
        distance[N + i] = 0;
        for (int v = 0; v < N; v++) {
          double slack = getDistance(A[v], bPoint) - A[v].dual - bPoint.dual;
          if (Math.abs(slack) <= slackThreshold) {
            slack = 0;
          }
          if (distance[v] > slack) {
            distance[v] = slack;
            parent[v] = N + i;
          }
        }
      }
    }

    // Conduct Dijsktra's until a free point in set A is found
    while (true) {
      int u = getMinDistanceNode();

      // Stop as soon as a free point in set A is found
      if (A[u].matchId == -1) {
        lMin = distance[u];
        freeMinDistanceIdxInA = u;
        break;
      }

      // Mark u as visited
      visited[u] = true;

      // Update the distances of the neighbours of u
      // u is a matched point in A and will have only one neighbor
      int w = A[u].matchId + N;
      distance[w] = distance[u];
      parent[w] = u;
      u = w - N;

      // u is now a point of B
      // All the boundaries and unmatched points in A can be reached from u
      Point bPoint = B[u];
      for (int v = 0; v < N; v++) {
        if (bPoint.matchId != v) {
          double slack = getDistance(A[v], bPoint) - A[v].dual - bPoint.dual;
          if (Math.abs(slack) <= slackThreshold) {
            slack = 0;
          }
          if (distance[v] > distance[w] + slack) {
            distance[v] = distance[w] + slack;
            parent[v] = w;
          }
        }
      }
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
    for (int i = 1; i < pathArraySize; i += 2) {
      A[path[i - 1]].matchId = B[path[i] - N].id;
      B[path[i] - N].matchId = A[path[i - 1]].id;
    }

    // Update the dual weights
    for (int i = 0; i < n; i++) {
      if (distance[i] < lMin) {
        if (i >= N) {
          B[i - N].dual = B[i - N].dual + (lMin - distance[i]); // dual weight update of nodes in set B
        } else {
          A[i].dual = A[i].dual - (lMin - distance[i]); // dual weight update of nodes in set A
        }
      }
    }
  }

  /**
   * Gets the minimum cost for exact euclidean bipartite matching of points in A and B
   */
  public void solver() {
    distance = new double[2 * N];
    parent = new int[2 * N];
    visited = new boolean[N];
    for (int i = 0; i < N; i++) {
      hungarianSearch();
    }
    getMatchingCardinalityAndCost();
  }
}
