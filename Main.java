import java.util.*;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
public class Main {
    public static void main(String[] args) {
            Graph g = new AdjListGraph();

            try {
                g.readFileAndConstructGraph("distDK.txt");
                g.printGraph(); // This will print the graph
                System.out.println("The graph is " + (g.isConnected() ? "connected." : "not connected."));
                // Part 2 :the complexity is O(E)
            } catch (IOException e) {
                e.printStackTrace();
            }
            // answer for part 3 : Esbjerg    done    Vejen    2734
        dijkstra(g,Graph.vertex("Helsingør"));

    }
    static void dijkstra(Graph g,Vertex w0){
        // w0 is the start node
        int mx=g.vertices().size();
        int maxint = 10000000;
        // create table for done, prev and weight from start
        HashSet<Vertex> done=new HashSet<>();
        HashMap<Vertex,Vertex> prev=new HashMap<>();
        HashMap<Vertex,Integer> weight=new HashMap<>();
        for(Vertex w:g.vertices())weight.put(w,maxint);
        // start node is done, distance 0 from start and has no prev
        weight.put(w0,0);
        done.add(w0);
        prev.put(w0,null);
        for(int i=0;i<2000;i++) {
            // find nearest node from a done node
            Vertex from = null;
            Vertex nearest = null;
            int neardist = maxint;
            for(Vertex w1:done)
                for (Edge e : g.outEdge(w1)) {
                    Vertex w2=e.to;
                    if (done.contains(w2)) continue;
                    if ((weight.get(w1)+e.weight) < neardist) {
                        nearest = e.to;
                        from=w1;
                        neardist = weight.get(w1)+e.weight;
                    }
                }
            // if no more then we are done
            if(nearest==null)break;
            // update distance from this node to other nodes
            for(Edge e:g.outEdge(nearest)){
                Vertex w3=e.to; int wght=e.weight;
                if(weight.get(w3)>(neardist+wght)){
                    weight.put(w3,neardist+wght);
                }
            }
            done.add(nearest);
            prev.put(nearest,from);
            // System.out.println("near: " + nearest + " " + neardist+" "+weight.get(nearest)+" "+from);
            weight.put(nearest,neardist);
        }
        // print the path to all nodes in the graph
        // paths from start vertex
        for(Vertex w:g.vertices()){
            String path=w.name+"("+weight.get(w)+")";
            Vertex w1=prev.get(w);
            while(w1!=null){path=w1.name+"("+weight.get(w1)+")"+" > "+path; w1=prev.get(w1);}
            System.out.println(path);
        }
        // print tables
        // make list vertex sorted by name
        ArrayList<Vertex> vertices =new ArrayList<>();
        vertices.addAll(done);
        Collections.sort(vertices, new Comparator<Vertex>() {
            public int compare(Vertex o1, Vertex o2) {return o1.name.compareTo(o2.name);}});
        System.out.println("vertex  done   prev   weight");
        for(Vertex v: vertices){
            System.out.println(v+"    done    "+prev.get(v)+"    "+weight.get(v));
        }
    }
}



abstract class Graph{
    void readFileAndConstructGraph(String filePath) throws IOException {
        try (BufferedReader reader = new BufferedReader(new FileReader(filePath))) {
            String line;
            while ((line = reader.readLine()) != null) {
                String[] parts = line.split(",");
                insertEdge(parts[0], parts[1], Integer.parseInt(parts[2]));
            }
        }
    }
    boolean isConnected() {
        if (vertex.isEmpty()) return true; // An empty graph is always connected

        Set<Vertex> visited = new HashSet<>();
        Vertex start = vertex.values().iterator().next(); // Start from any vertex
        visitDepthFirst(start, visited);

        return visited.size() == vertex.size(); // The graph is connected if all vertices were visited
    }
    abstract void insertEdge(String v,String u,int w);
    abstract void printGraph();
    static HashMap<String,Vertex> vertex=new HashMap<>();
    static Vertex vertex(String s){
        if(!vertex.containsKey(s))vertex.put(s,new Vertex(s));
        return vertex.get(s);
    }
    Collection<Vertex> vertices(){return vertex.values();}
    abstract Collection<Edge> edges();
    abstract Collection<Edge> outEdge(Vertex v);
    void visitDepthFirst(Vertex v,Set<Vertex> visited){
        if(visited.contains(v))return;
        // commented out because too much output
        // System.out.println("visited "+v);
        visited.add(v);
        for(Edge e: outEdge(v))
            visitDepthFirst(e.to,visited);
    }
    void visitBreadthFirst(Vertex v){
        HashSet<Vertex> thisLevel=new HashSet<>();
        HashSet<Vertex> nextLevel=new HashSet<>();
        HashSet<Vertex> visited=new HashSet<>();
        thisLevel.add(v);
        while(thisLevel.size()>0){
            System.out.println("level "+thisLevel);
            for(Vertex w:thisLevel){
                System.out.println("visited "+w);
                visited.add(w);
                Collection<Edge> outedge=outEdge(v);
                if(outedge==null)continue;
                for(Edge e: outedge){
                    if(visited.contains(e.to))continue;
                    if(thisLevel.contains(e.to))continue;
                    nextLevel.add(e.to);
                }
            }
            thisLevel=nextLevel;
            nextLevel=new HashSet<Vertex>();
        }
    }
}

class Vertex{
    String name;
    Vertex(String s){name=s;}
    public String toString(){return name;}
}

class Edge{
    Vertex from,to;
    int weight;
    Edge(Vertex from,Vertex to,int w){this.from=from; this.to=to; weight=w;}
    public String toString(){return from.name+" - "+weight+" -> "+to.name; }
}

class EdgeGraph extends Graph{
    HashSet<Edge> edges=new HashSet<>();
    void insertEdge(String u,String v,int w){
        edges.add(new Edge(vertex(u),vertex(v),w));
    }
    void printGraph() {
        for(Edge e:edges) System.out.println(e);
    }
    Collection<Edge> edges(){return edges;}
    Collection<Edge> outEdge(Vertex v){
        ArrayList<Edge> outEdge=new ArrayList<>();
        for(Edge e:edges)if(e.from==v)outEdge.add(e);
        return outEdge;
    }
}

class AdjListGraph extends Graph {
    HashMap<Vertex, Set<Edge>> outEdge = new HashMap<>();

    void insertEdge(String u, String v, int w) {
        Vertex vertexU = vertex(u);
        Vertex vertexV = vertex(v);

        // Create an edge from u to v
        Edge edgeUV = new Edge(vertexU, vertexV, w);
        outEdge.computeIfAbsent(vertexU, k -> new HashSet<>()).add(edgeUV);

        // Create an edge from v to u for bidirectional graph
        Edge edgeVU = new Edge(vertexV, vertexU, w);
        outEdge.computeIfAbsent(vertexV, k -> new HashSet<>()).add(edgeVU);
    }
    Collection<Edge> edges(){
        Set<Edge> edges=new HashSet<>();
        for(Vertex v:outEdge.keySet())edges.addAll(outEdge.get(v));
        return edges;
    }
    void printGraph() {
        for(Vertex v:outEdge.keySet()){
            System.out.println("vertex "+v);
            for(Edge e:outEdge.get(v))System.out.println(" "+e);
        }
    }
    Collection<Edge> outEdge(Vertex v){
        if(!outEdge.containsKey(v))
            return new HashSet<Edge>();
        return outEdge.get(v);
    }
}
