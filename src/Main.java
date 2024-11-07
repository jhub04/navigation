import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.PriorityQueue;

public class Main {

  static ShortestPath sp = new ShortestPath();
  static int[][] distFromNodesToLandmarks;
  static int[][] distFromLandmarksToNodes;
  static Node[] landmarks;


  public static void main(String[] args) {
    Node[] nodes = readDataFromFiles();

    int drinkingPlaceCode = 16;
    int eatingPlaceCode = 8;
    int chargingStationCode = 4;
    Node gloshaugen = nodes[2001238];
    Node levanger = nodes[2106148];
    Node aareBjornen = nodes[790843];
    Node otilienborg= nodes[1987066];
    Node fosnavåg = nodes[2486870];
    Node espoo = nodes[5394165];
    findNearestTypes(nodes, levanger, chargingStationCode, 4);
    System.out.println("------------------------------");
    findNearestTypes(nodes, gloshaugen, drinkingPlaceCode, 4);
    System.out.println("------------------------------");
    findNearestTypes(nodes, aareBjornen, eatingPlaceCode, 4);
    System.out.println("------------------------------");
    initializeAStar(nodes);
    System.out.println("Gløshaugen - Othilienborg");
    runDijkstra(nodes, gloshaugen, otilienborg);
    runAStar(nodes, gloshaugen, otilienborg);
    System.out.println("------------------------------");
    System.out.println("Fosnavåg - Espoo");
    runDijkstra(nodes, fosnavåg, espoo);
    runAStar(nodes, fosnavåg, espoo);

  }

  static void initializeAStar(Node[] nodes) {
    landmarks = new Node[]{nodes[2531818], nodes[7021334], nodes[4909517], nodes[3167529]};//0:Nordkapp, 1:Ilomantsi, 2:Bergen, 3:Padborg
    Node[] transposedNodes = transposeGraph(nodes);
    Node[] landmarksTransposed = {transposedNodes[2531818], transposedNodes[7021334], transposedNodes[4909517], transposedNodes[3167529]};
    distFromLandmarksToNodes = sp.getDistancesFromLandmarkToNodes(landmarks, nodes);
    distFromNodesToLandmarks = sp.getDistancesFromLandmarkToNodes(landmarksTransposed, transposedNodes);
  }

  static void runDijkstra(Node[] nodes, Node start, Node end) {
    long d0 = System.currentTimeMillis();
    int count = sp.Dijkstra(start, end, nodes);
    long d1 = System.currentTimeMillis();

    int secondsTravel = end.distanceToStart / 100;
    int hourTravel = secondsTravel / 3600;
    int minTravel = (secondsTravel - hourTravel * 3600) / 60;
    System.out.println(
        "Dijkstra: Travel takes " + hourTravel + " hours " + minTravel + " minutes " + (secondsTravel % 60)
            + " seconds from node " + start.nodeNum + " to " + end.nodeNum +" . Dijkstra spent " + (d1 - d0) + " milliseconds and explored " + count + " nodes");
  }

  static void runAStar(Node[] nodes, Node start, Node end) {
    long a0 = System.currentTimeMillis();
    int count = sp.AStar(start, end, nodes, landmarks, distFromLandmarksToNodes, distFromNodesToLandmarks);
    long a1 = System.currentTimeMillis();
    int secondsTravel = end.distanceToStart / 100;
    int hourTravel = secondsTravel / 3600;
    int minTravel = (secondsTravel - hourTravel * 3600) / 60;
    System.out.println(
        "ALT: Travel takes " + hourTravel + " hours " + minTravel + " minutes " + (secondsTravel % 60)
            + " seconds from node " + start.nodeNum + " to " + end.nodeNum +  ". ALT spent " + (a1 - a0) + " milliseconds and explored " + count + " nodes");
  }

  static void findNearestTypes(Node[] nodes, Node startNode, int typeCode, int amount) {
    Node[] nodesFound = sp.DijkstraFindNearestTypes(startNode, typeCode, amount, nodes);
    for (Node node : nodesFound) {
      System.out.println(
          "Node " + node.nodeNum + " coords " + node.latitude + " " + node.longitude + " is of type"
              + node.typeCode);
    }
  }

  static Node[] readDataFromFiles() {
    Node[] nodes = GraphFileReader.readNodesFromFile("norden/noder.txt");
    GraphFileReader.readEdgesFromFile("norden/kanter.txt", nodes);
    GraphFileReader.readTypeCodesFromFile("norden/interessepkt.txt", nodes);
    return nodes;
  }

  static Node[] transposeGraph(Node[] nodes) {
    Node[] transposedGraph = new Node[nodes.length];

    for (int i = 0; i < nodes.length; ++i) {
      transposedGraph[i] = new Node(i, nodes[i].latitude, nodes[i].longitude);
    }

    for (int i = 0; i < nodes.length; ++i) {
      //u->v to v->u
      Node u = nodes[i];
      int nodeUNum = i;
      for (Edge edge : u.edges) {
        int nodeVNum = edge.to.nodeNum;
        Node newVNode = transposedGraph[nodeVNum];
        Node newUNode = transposedGraph[nodeUNum];
        newVNode.addEdge(new Edge(edge.drivingTime, newUNode, edge.distance));
      }
    }
    return transposedGraph;
  }
}

class Node implements Comparable<Node> {

  int nodeNum;
  double latitude;
  double longitude;
  LinkedList<Edge> edges;
  Node previousNode;
  int distanceToStart;
  int estimatedDistanceToGoal;
  boolean found = false;
  int amountOfNodesToStart;
  int typeCode;

  //DEnne for dijkstra
  public Node(int nodeNum, double latitude, double longitude) {
    this.nodeNum = nodeNum;
    this.latitude = latitude;
    this.longitude = longitude;
    edges = new LinkedList<>();
    this.distanceToStart = Integer.MAX_VALUE;
  }

  //Bruk denne konstruktør for ALT
  public Node(int nodeNum, double latitude, double longitude, int estimatedDistanceToGoal) {
    this.nodeNum = nodeNum;
    this.latitude = latitude;
    this.longitude = longitude;
    this.estimatedDistanceToGoal = estimatedDistanceToGoal;
    edges = new LinkedList<>();
    this.distanceToStart = Integer.MAX_VALUE;
  }

  public void addEdge(Edge edge) {
    edges.add(edge);
  }

  void clearDistanceToGoal() {
    estimatedDistanceToGoal = 0;
  }

  public void setPreviousNode(Node previousNode) {
    this.previousNode = previousNode;
  }

  public void setDistanceToStart(int distanceToStart) {
    this.distanceToStart = distanceToStart;
  }

  public void clearDistanceToStart() {
    this.distanceToStart = 0;
  }

  @Override
  public int compareTo(Node o) {
    if (this.distanceToStart + this.estimatedDistanceToGoal
        < o.distanceToStart + o.estimatedDistanceToGoal) {
      return -1;
    } else if (this.distanceToStart + this.estimatedDistanceToGoal
        > o.distanceToStart + o.estimatedDistanceToGoal) {
      return 1;
    } else {
      return 0;
    }
  }
}

class Edge {

  int drivingTime; //I hundredels sekunder
  Node to;
  int distance;

  public Edge(int drivingTime, Node to, int distance) {
    this.drivingTime = drivingTime;
    this.to = to;
    this.distance = distance;
  }
}

class ShortestPath {
  public int Dijkstra(Node startNode, Node goalNode, Node[] nodes) {
    PriorityQueue<Node> pq = new PriorityQueue<>();
    initDijkstraSearch(nodes);
    int count = 0;
    startNode.distanceToStart = 0;
    pq.add(startNode);
    pq.add(goalNode);
    while (!goalNode.found) {
      Node exploreNode = pq.poll();
      count++;
      exploreNode.found = true;
      for (Edge edge : exploreNode.edges) {
        if (!edge.to.found) {

          if (edge.to.previousNode == null) {
            edge.to.previousNode = exploreNode;
            edge.to.amountOfNodesToStart = exploreNode.amountOfNodesToStart + 1;
            edge.to.distanceToStart = exploreNode.distanceToStart + edge.drivingTime;
            pq.add(edge.to);
          } else if (edge.to.distanceToStart + edge.to.estimatedDistanceToGoal
              > exploreNode.distanceToStart + edge.to.estimatedDistanceToGoal + edge.drivingTime) {
            edge.to.previousNode = exploreNode;
            edge.to.amountOfNodesToStart = exploreNode.amountOfNodesToStart + 1;
            edge.to.distanceToStart = exploreNode.distanceToStart + edge.drivingTime;
            pq.remove(edge.to);
            pq.add(edge.to);
          }
          /**System.out.println("Nodenum" + edge.to.nodeNum + " coming from node " + exploreNode.nodeNum);
           System.out.println("Dist start" + edge.to.distanceToStart);
           System.out.println("est dist goal" + edge.to.estimatedDistanceToGoal);*/
        }
      }
    }
    return count;
  }

  public int AStar(Node startNode, Node goalNode, Node[] nodes, Node[] landmarks,
      int[][] distancesFromLandmarksToNodes, int[][] distancesFromNodesToLandmarks) {
    PriorityQueue<Node> pq = new PriorityQueue<>();
    initDijkstraSearch(nodes);
    int count = 0;
    startNode.distanceToStart = 0;
    setDistanceToGoal(startNode, landmarks, goalNode, distancesFromLandmarksToNodes,
        distancesFromNodesToLandmarks);
    pq.add(startNode);
    pq.add(goalNode);
    while (!goalNode.found) {
      Node exploreNode = pq.poll();
      count++;
      exploreNode.found = true;
      for (Edge edge : exploreNode.edges) {
        if (!edge.to.found) {
          if (edge.to.previousNode == null) {
            edge.to.previousNode = exploreNode;
            edge.to.amountOfNodesToStart = exploreNode.amountOfNodesToStart + 1;
            edge.to.distanceToStart = exploreNode.distanceToStart + edge.drivingTime;
            setDistanceToGoal(edge.to, landmarks, goalNode, distancesFromLandmarksToNodes,
                distancesFromNodesToLandmarks);
            pq.add(edge.to);
          } else if (edge.to.distanceToStart + edge.to.estimatedDistanceToGoal
              > exploreNode.distanceToStart + edge.to.estimatedDistanceToGoal + edge.drivingTime) {
            edge.to.previousNode = exploreNode;
            edge.to.amountOfNodesToStart = exploreNode.amountOfNodesToStart + 1;
            edge.to.distanceToStart = exploreNode.distanceToStart + edge.drivingTime;
            pq.remove(edge.to);
            pq.add(edge.to);
          }
        }
      }
    }
    return count;
  }

  void setDistanceToGoal(Node from, Node[] landmarks, Node goalNode,
      int[][] distancesFromLandmarksToNodes,
      int[][] distancesFromNodesToLandmarks) {
    int maxEstimate = 0;
    for (int i = 0; i < landmarks.length; i++) {
      int distanceFrom = distancesFromLandmarksToNodes[i][from.nodeNum];
      int distanceGoal = distancesFromLandmarksToNodes[i][goalNode.nodeNum];
      int distanceEstimate = distanceGoal - distanceFrom;
      distanceFrom = distancesFromNodesToLandmarks[i][from.nodeNum];
      distanceGoal = distancesFromNodesToLandmarks[i][goalNode.nodeNum];
      int newDistanceEstimate = distanceFrom - distanceGoal;
      if (distanceEstimate < newDistanceEstimate) {
        distanceEstimate = newDistanceEstimate;
      }
      if (distanceEstimate < 0) {
        distanceEstimate = 0;
      }
      if (distanceEstimate > maxEstimate) {
        maxEstimate = distanceEstimate;
      }
    }
    from.estimatedDistanceToGoal = maxEstimate;
  }


  public Node[] DijkstraFindNearestTypes(Node startNode, int typeCode, int amount, Node[] nodes) {
    int amountFound = 0;
    initDijkstraSearch(nodes);
    Node[] nodesFoundOfType = new Node[amount];
    PriorityQueue<Node> pq = new PriorityQueue<>();
    startNode.distanceToStart = 0;
    pq.add(startNode);
    while (amountFound < amount) {
      Node exploreNode = pq.poll();
      exploreNode.found = true;
      if ((exploreNode.typeCode & typeCode) == typeCode && !Arrays.asList(nodesFoundOfType).contains(exploreNode)) {
        nodesFoundOfType[amountFound] = exploreNode;
        amountFound++;
      }
      for (Edge edge : exploreNode.edges) {
        if (!edge.to.found) {
          if (edge.to.previousNode == null) {
            edge.to.previousNode = exploreNode;
            edge.to.amountOfNodesToStart = exploreNode.amountOfNodesToStart + 1;
            edge.to.distanceToStart = exploreNode.distanceToStart + edge.drivingTime;
            pq.add(edge.to);
          } else if (edge.to.distanceToStart + edge.to.estimatedDistanceToGoal
              > exploreNode.distanceToStart + edge.to.estimatedDistanceToGoal + edge.drivingTime) {
            edge.to.previousNode = exploreNode;
            edge.to.amountOfNodesToStart = exploreNode.amountOfNodesToStart + 1;
            edge.to.distanceToStart = exploreNode.distanceToStart + edge.drivingTime;
            pq.remove(edge.to);
            pq.add(edge.to);
          }
        }
      }
    }
    return nodesFoundOfType;
  }

  public int[][] getDistancesFromLandmarkToNodes(Node[] landmarks, Node[] nodes) {
    int[][] distanceFromLandmarkToNodes = new int[landmarks.length][nodes.length];
    for (int i = 0; i < landmarks.length; i++) {
      initDijkstraSearch(nodes);
      PriorityQueue<Node> pq = new PriorityQueue<>();
      landmarks[i].distanceToStart = 0;
      pq.add(landmarks[i]);
      while (!pq.isEmpty()) {
        Node exploreNode = pq.poll();
        exploreNode.found = true;
        for (Edge edge : exploreNode.edges) {
          if (!edge.to.found) {
            if (edge.to.previousNode == null) {
              edge.to.previousNode = exploreNode;
              edge.to.amountOfNodesToStart = exploreNode.amountOfNodesToStart + 1;
              edge.to.distanceToStart = exploreNode.distanceToStart + edge.drivingTime;
              pq.add(edge.to);
            } else if (edge.to.distanceToStart + edge.to.estimatedDistanceToGoal
                > exploreNode.distanceToStart + edge.to.estimatedDistanceToGoal
                + edge.drivingTime) {
              edge.to.previousNode = exploreNode;
              edge.to.amountOfNodesToStart = exploreNode.amountOfNodesToStart + 1;
              edge.to.distanceToStart = exploreNode.distanceToStart + edge.drivingTime;
              pq.remove(edge.to);
              pq.add(edge.to);
            }
            distanceFromLandmarkToNodes[i][edge.to.nodeNum] = edge.to.distanceToStart;
          }
        }
      }
    }
    return distanceFromLandmarkToNodes;
  }

  void initDijkstraSearch(Node[] nodes) {
    for (Node node : nodes) {
      node.setDistanceToStart(Integer.MAX_VALUE);
      node.previousNode = null;
      node.found = false;
      node.amountOfNodesToStart = 1;
    }
  }
}

class GraphFileReader {

  static String[] felt = new String[10];

  public static Node[] readNodesFromFile(String fileName) {
    System.out.println("Lese nodes");
    try (BufferedReader br = new BufferedReader(new FileReader(fileName))) {
      String line = br.readLine();
      hsplit(line, 1);
      int nodeAmount = Integer.parseInt(felt[0]);
      Node[] nodes = new Node[nodeAmount];

      while ((line = br.readLine()) != null) {
        hsplit(line, 3);
        int nodeNum = Integer.parseInt(felt[0]);
        Node node = new Node(nodeNum, Double.parseDouble(felt[1]), Double.parseDouble(felt[2]));
        nodes[nodeNum] = node;
      }
      System.out.println("Ferdig nodes");
      return nodes;
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public static void readEdgesFromFile(String pathName, Node[] nodes) {
    try (BufferedReader br = new BufferedReader(new FileReader(pathName))) {
      System.out.println("lese kanter");
      String line = br.readLine();
      hsplit(line, 1);
      int edgeAmount = Integer.parseInt(felt[0]);
      while ((line = br.readLine()) != null) {
        hsplit(line, 5);
        int nodeFromNum = Integer.parseInt(felt[0]);
        int nodeToNum = Integer.parseInt(felt[1]);
        int drivingTime = Integer.parseInt(felt[2]);
        int distance = Integer.parseInt(felt[3]);
        Edge edge = new Edge(drivingTime, nodes[nodeToNum], distance);
        nodes[nodeFromNum].addEdge(edge);
      }
      System.out.println("Feridg kant");
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public static void readTypeCodesFromFile(String path, Node[] nodes) {
    try (BufferedReader br = new BufferedReader(new FileReader(path))) {
      br.readLine();
      String line;

      while ((line = br.readLine()) != null) {
        hsplit(line, 3);
        int nodeNum = Integer.parseInt(felt[0]);
        nodes[nodeNum].typeCode = Integer.parseInt(felt[1]);
      }
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  static void hsplit(String linje, int antall) {
    int j = 0;
    int lengde = linje.length();
    for (int i = 0; i < antall; ++i) {
      //Hopp over innledende blanke, finn starten på ordet
      while (linje.charAt(j) <= ' ') {
        ++j;
      }
      int ordstart = j;
      //Finn slutten på ordet, hopp over ikke-blanke
      while (j < lengde && linje.charAt(j) > ' ') {
        ++j;
      }
      felt[i] = linje.substring(ordstart, j);
    }
  }

}