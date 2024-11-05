import java.util.LinkedList;

public class Main {
    public static void main(String []args) {
        System.out.println("Hello world");
    }
}

class Node {
    int nodeNum;
    double latitude;
    double longitude;
    LinkedList<Edge> edges;
    Node previousNode;
    int distanceToStart;
    int estimatedDistanceToGoal = 0;

    //DEnne for dijkstra
    public Node(int nodeNum, double latitude, double longitude) {
        this.nodeNum = nodeNum;
        this.latitude = latitude;
        this.longitude = longitude;
        edges = new LinkedList<>();
    }

    //Bruk denne konstruktør for ALT
    public Node(int nodeNum, double latitude, double longitude, int estimatedDistanceToGoal) {
        this.nodeNum = nodeNum;
        this.latitude = latitude;
        this.longitude = longitude;
        this.estimatedDistanceToGoal = estimatedDistanceToGoal;
        edges = new LinkedList<>();
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

}

class Edge {
    int drivingTime; //I hundredels sekunder
    Node from;
    Node to;
}