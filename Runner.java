import java.io.File;
import java.io.FileNotFoundException;
import java.util.*;

public class Runner {

  static int N, pythonMatchingCardinality;
  static long startTime, endTime, totalTime, pythonMatchingCalculationTime;
  static int[] pythonMatching;
  static double totalCost, pythonMatchingCost;
  static Point[] A, B;
  static HungarianAlgorithm ha;
  static DivideAndConquerHungarianAlgorithm dacha;
  static double slackThreshold = 0.0000001;
  static String path1, path2;

  /**
   * Read the two datasets
   */
  private static void initializeDatasets() {
    A = new Point[N];
    B = new Point[N];
    String[] arr;
    try {
      Scanner scanner = new Scanner(new File(path1));

      for (int i = 0; i < N; i++) {
        arr = scanner.nextLine().split(" ");
        A[i] =
          new Point(
            Double.parseDouble(arr[0]),
            Double.parseDouble(arr[1]),
            arr.length > 2 ? Double.parseDouble(arr[2]) : 0,
            i
          );
      }
      scanner.close();

      scanner = new Scanner(new File(path2));
      for (int i = 0; i < N; i++) {
        arr = scanner.nextLine().split(" ");
        B[i] =
          new Point(
            Double.parseDouble(arr[0]),
            Double.parseDouble(arr[1]),
            arr.length > 2 ? Double.parseDouble(arr[2]) : 0,
            i
          );
      }
      scanner.close();
    } catch (FileNotFoundException e) {
      System.out.println("File not found");
      e.printStackTrace();
    }
  }

  /**
   *
   * @param p1
   * @param p2
   * @return - Distance between the two points
   */
  private static double getDistance(Point p1, Point p2) {
    return Math.sqrt(
      ((p1.x - p2.x) * (p1.x - p2.x)) +
      ((p1.y - p2.y) * (p1.y - p2.y)) +
      ((p1.z - p2.z) * (p1.z - p2.z))
    );
  }

  /**
   * Validates the dual weights
   */
  private static boolean validateDualWeights() {
    for (int i = 0; i < N; i++) {
      for (int j = 0; j < N; j++) {
        double slack = getDistance(A[i], B[j]) - A[i].dual - B[j].dual;
        if (slack < -1 * slackThreshold) {
          System.out.println("Slack: " + slack + " is negative");
          return false;
        }
        if (A[i].matchId == j && slack > slackThreshold) {
          return false;
        }
      }
    }
    return true;
  }

  /**
   * Runs the Hungarian algorithm solver
   */
  private static void runHungarianSolver() {
    System.out.println(
      "--------------------------Hungarian Algorithm Solver---------------------------"
    );
    ha = new HungarianAlgorithm(A, B);
    startTime = System.currentTimeMillis();
    ha.solver();
    endTime = System.currentTimeMillis();
    if (!validateDualWeights()) {
      System.out.println("Infeasible dual weights");
      System.exit(0);
    }
    System.out.println("Feasible dual weights");
    System.out.printf("Matching Cost: %.4f \n", ha.getMatchingCost());
    System.out.println("Time taken: " + (endTime - startTime) + " ms");
    System.out.println(
      "-------------------------------------------------------------------------------"
    );
  }

  /**
   * Runs the Divide and Conquer Hungarian algorithm solver
   */
  private static void runDivideAndConquerHungarianSolver() {
    System.out.println(
      "-----------------Divide and Conquer Hungarian Algorithm Solver-----------------"
    );
    Boundary boundary = new Boundary(1, 0, 0, 1, 1, 0);
    dacha = new DivideAndConquerHungarianAlgorithm(A, B);
    startTime = System.currentTimeMillis();
    dacha.solver(boundary);
    endTime = System.currentTimeMillis();
    if (!validateDualWeights()) {
      System.out.println("Infeasible dual weights");
      System.exit(0);
    }
    System.out.println("Feasible dual weights");

    System.out.printf("Matching Cost: %.4f \n", dacha.getMatchingCost());
    System.out.println("Time taken: " + (endTime - startTime) + " ms");
    System.out.println(
      "-------------------------------------------------------------------------------"
    );
  }

  public static void main(String[] args) {
    try {
      // Add the path to the two datasets and the number of points to be used from each dataset
      path1 = "Datasets/A0.txt";
      path2 = "Datasets/B0.txt";
      N = 1000;
      initializeDatasets();
      runHungarianSolver();
      runDivideAndConquerHungarianSolver();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
