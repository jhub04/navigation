import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.LinkedList;
import java.util.PriorityQueue;

public class Main {
    public static void main(String[] args) {
        Node []nodes = GraphFileReader.readNodesFromFile("norden/noder.txt");
        GraphFileReader.readEdgesFromFile("norden/kanter.txt", nodes);
        Node start = nodes[6441311];
        Node end = nodes[3168086];
        ShortestPath sp = new ShortestPath();
        sp.Dijkstra(start, end);
        int secondsTravel = end.distanceToStart / 100;
        int hourTravel = secondsTravel / 3600;
        int minTravel = (secondsTravel - hourTravel*3600)/60;
        System.out.println("Travel takes " + hourTravel + "hours" + minTravel + " minutes" + (secondsTravel % 60) + " seconds");
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
    int amountOfNodesToStart = 0;

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
    int distance;

    public Edge(int drivingTime, Node to, int distance) {
        this.drivingTime = drivingTime;
        this.to = to;
        this.distance = distance;
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
                    if (edge.to.previousNode == null) {
                        edge.to.previousNode = exploreNode;
                        edge.to.amountOfNodesToStart = exploreNode.amountOfNodesToStart++;
                        edge.to.distanceToStart = exploreNode.distanceToStart + edge.drivingTime;
                        pq.add(edge.to);
                    } else if (edge.to.distanceToStart + edge.to.estimatedDistanceToGoal > exploreNode.distanceToStart + edge.to.estimatedDistanceToGoal + edge.drivingTime) {
                        edge.to.previousNode = exploreNode;
                        edge.to.amountOfNodesToStart = exploreNode.amountOfNodesToStart++;
                        edge.to.distanceToStart = exploreNode.distanceToStart + edge.drivingTime;
                        pq.remove(edge.to);
                        pq.add(edge.to);
                    }
                }
            }
        }
    }
}

class GraphFileReader{
    static String[] felt = new String[10];

    public static Node[] readNodesFromFile(String fileName) {
        System.out.println("Lese nodes");
        try (BufferedReader br = new BufferedReader(new FileReader(fileName))) {
            String line = br.readLine();
            hsplit(line, 1);
            int nodeAmount = Integer.parseInt(felt[0]);
            Node []nodes = new Node[nodeAmount];

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

    public static void readEdgesFromFile(String pathName, Node []nodes) {
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

    static void hsplit(String linje, int antall) {
        int j = 0;
        int lengde = linje.length();
        for (int i = 0; i < antall; ++i) {
            //Hopp over innledende blanke, finn starten på ordet
            while (linje.charAt(j) <= ' ') ++j;
            int ordstart = j;
            //Finn slutten på ordet, hopp over ikke-blanke
            while (j < lengde && linje.charAt(j) > ' ') ++j;
            felt[i] = linje.substring(ordstart, j);
        }
    }

}