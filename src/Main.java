import java.util.LinkedList;
import java.util.PriorityQueue;

public class Main {
    public static void main(String[] args) {
        Node A = new Node(1, 1, 1);
        Node B = new Node(2, 1, 1);
        Node C = new Node(3, 1, 1);
        Node D = new Node(4, 1, 1);
        Node E = new Node(5, 1, 1);
        Edge AtoB = new Edge(1, B);
        Edge AtoC = new Edge(4, C);
        Edge CtoD = new Edge(3, D);
        Edge BtoC = new Edge(2, C);
        Edge DtoE = new Edge(5, E);
        A.addEdge(AtoB);
        A.addEdge(AtoC);
        B.addEdge(BtoC);
        C.addEdge(CtoD);
        D.addEdge(DtoE);
        ShortestPath sp = new ShortestPath();
        sp.Dijkstra(A, E);
    }
}

class Node implements Comparable<Node> {
    int nodeNum;
    double latitude;
    double longitude;
    LinkedList<Edge> edges;
    Node previousNode;
    int distanceToStart;
    int estimatedDistanceToGoal = 0;
    boolean found = false;

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

    public void setPreviousNode(Node previousNode) {
        this.previousNode = previousNode;
    }

    public void setDistanceToStart(int distanceToStart) {
        this.distanceToStart = distanceToStart;
    }

    @Override
    public int compareTo(Node o) {
        if (this.distanceToStart + this.estimatedDistanceToGoal < o.distanceToStart + o.estimatedDistanceToGoal) {
            return -1;
        } else if (this.distanceToStart + this.estimatedDistanceToGoal > o.distanceToStart + o.estimatedDistanceToGoal) {
            return 1;
        } else return 0;
    }
}

class Edge {
    int drivingTime; //I hundredels sekunder
    Node to;

    public Edge(int drivingTime, Node to) {
        this.drivingTime = drivingTime;
        this.to = to;
    }
}

class ShortestPath {
    public void Dijkstra(Node startNode, Node goalNode) {
        PriorityQueue<Node> pq = new PriorityQueue<>();
        startNode.distanceToStart = 0;
        pq.add(startNode);
        pq.add(goalNode);
        while (!goalNode.found) {
            Node exploreNode = pq.poll();
            exploreNode.found = true;
            for (Edge edge : exploreNode.edges) {
                if(!edge.to.found) {
                    pq.add(edge.to);
                    System.out.println("Reach");
                    if (edge.to.previousNode == null) {
                        edge.to.previousNode = exploreNode;
                        edge.to.distanceToStart = edge.to.distanceToStart + edge.drivingTime;
                    } else if (edge.to.distanceToStart > exploreNode.distanceToStart + edge.drivingTime) {
                        //TODO legge til estimatedGoalDistance i else if greia
                        edge.to.previousNode = exploreNode;
                        edge.to.distanceToStart = edge.to.distanceToStart + edge.drivingTime;
                    }
                }
            }
        }
    }
}