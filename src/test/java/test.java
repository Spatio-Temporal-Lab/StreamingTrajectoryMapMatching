import com.fasterxml.jackson.core.JsonProcessingException;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.urbcomp.cupid.db.algorithm.kalman.AdaptiveKalmanFilter;
import org.urbcomp.cupid.db.algorithm.kalman.KalmanFilter;
import org.urbcomp.cupid.db.algorithm.mapmatch.stream.StreamMapMatcher;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.TiHmmMapMatcher;
import org.urbcomp.cupid.db.algorithm.shortestpath.ManyToManyShortestPath;
import org.urbcomp.cupid.db.exception.AlgorithmExecuteException;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.model.point.MapMatchedPoint;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.sample.ModelGenerator;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;

import java.util.*;

public class test {
    public static void main(String[] args) throws JsonProcessingException, AlgorithmExecuteException {

//        RoadNetwork roadNetwork = ModelGenerator.generateRoadNetwork();
//        TiHmmMapMatcher mapMatcher = new TiHmmMapMatcher(roadNetwork, new ManyToManyShortestPath(roadNetwork));
//        Trajectory trajectory = ModelGenerator.generateTrajectory(6);
//        MapMatchedTrajectory mmTrajectory = mapMatcher.mapMatch(trajectory, 0.5);
//        List<GPSPoint> pointList = trajectory.getGPSPointList();
//        List<GPSPoint> filterPointList = new ArrayList<>();
//        double[][] F = new double[][]{
//                {1, 0, 1, 0},
//                {0, 1, 0, 1},
//                {0, 0, 1, 0},
//                {0, 0, 0, 1}
//        };
//        double[][] H = new double[][]{
//                {1, 0, 0, 0},
//                {0, 1, 0, 0},
//                {0, 0, 1, 0},
//                {0, 0, 0, 1}
//        };
//        double pre_var = 1 * 1;
//        double gps_var = 1 * 1;
//        AdaptiveKalmanFilter kalmanFilter = new AdaptiveKalmanFilter(F, H, gps_var, pre_var);
//        int index = 0;
//        for (GPSPoint p : pointList){
//            MapMatchedPoint mapMatchedPoint = mmTrajectory.getMmPtList().get(index);
//            double[] estimate = kalmanFilter.process(p.getX(),p.getY(), mapMatchedPoint.getTime());
//            GPSPoint filterPoint = new GPSPoint(p.getTime(), estimate[0], estimate[1]);
//            filterPointList.add(filterPoint);
//            kalmanFilter.update(p.getX(),p.getY(), mapMatchedPoint.getTime());
////            kalmanFilter.updateByReal(mapMatchedPoint.getCandidatePoint().getX(), mapMatchedPoint.getCandidatePoint().getY());
////            kalmanFilter.updateNoiseCovariances(mapMatchedPoint.getCandidatePoint().getX(), mapMatchedPoint.getCandidatePoint().getY());
//            index++;
//        }
//        Trajectory filterTrajectory = new Trajectory(trajectory.getTid(),trajectory.getOid(),filterPointList);
//        System.out.println(trajectory.toGeoJSON());
//        System.out.println(filterTrajectory.toGeoJSON());

//        int[] list = new int[]{7, 5, 3, 1, 8, 6, 2, 4};
//        ListNode head = createLinkList(list);
//        head = insertionSortList(head);
//        ListNode.printList(head);
//        System.out.println();
//        head = reverseList(head);
//        ListNode.printList(head);

        Map<Integer, Map<Integer, Integer>> graph = new HashMap<>();
        // 添加边和权重
//        addEdge(graph, 1, 2, 20);
//        addEdge(graph, 1, 3, 50);
//        addEdge(graph, 1, 4, 30);
//        addEdge(graph, 2, 3, 25);
//        addEdge(graph, 2, 6, 70);
//        addEdge(graph, 3, 4, 40);
//        addEdge(graph, 3, 5, 25);
//        addEdge(graph, 3, 6, 50);
//        addEdge(graph, 4, 5, 55);
//        addEdge(graph, 5, 6, 10);
//        addEdge(graph, 5, 7, 70);
//        addEdge(graph, 6, 7, 50);
//        for (int i = 1; i <= 7; i++) {
//            addEdge(graph, i, i, 0);
//        }
        addEdge(graph, 1, 2, 3);
        addEdge(graph, 1, 3, 6);
        addEdge(graph, 1, 4, 7);
        addEdge(graph, 2, 1, 3);
        addEdge(graph, 2, 3, 1);
        addEdge(graph, 2, 5, 4);
        addEdge(graph, 3, 1, 6);
        addEdge(graph, 3, 2, 1);
        addEdge(graph, 3, 6, 2);
        addEdge(graph, 4, 1, 7);
        addEdge(graph, 4, 6, 3);
        addEdge(graph, 4, 7, 4);
        addEdge(graph, 5, 2, 4);
        addEdge(graph, 5, 8, 1);
        addEdge(graph, 6, 3, 2);
        addEdge(graph, 6, 4, 3);
        addEdge(graph, 6, 8, 1);
        addEdge(graph, 6, 9, 2);
        addEdge(graph, 7, 4, 4);
        addEdge(graph, 7, 9, 5);
        addEdge(graph, 8, 5, 1);
        addEdge(graph, 8, 6, 1);
        addEdge(graph, 8, 9, 2);
        addEdge(graph, 9, 6, 2);
        addEdge(graph, 9, 7, 5);
        addEdge(graph, 9, 8, 2);

        for (int i = 1; i <= 9; i++) {
            addEdge(graph, i, i, 0);
        }
        // 打印图的边和权重
//        printGraph(graph);
//        dijkstra(graph, 1);
        bi_dijkstra(graph, 1, 9);
    }

    private static ListNode createLinkList(int[] values) {
        ListNode head = new ListNode(values[0]);
        ListNode node = head;
        for (int i = 1; i < values.length; i++) {
            node.next = new ListNode(values[i]);
            node = node.next;
        }
        return head;
    }

    private static ListNode reverseList(ListNode head) {
        ListNode pre = null, curr = head;
        while (curr != null) {
            ListNode next = curr.next;
            curr.next = pre;
            pre = curr;
            curr = next;
        }
        return pre;
    }

    private static ListNode insertionSortList(ListNode head) {
        ListNode dummy = new ListNode(-1, head);
        ListNode lastSorted = head, curr = head.next;
        while (curr != null) {
            if (lastSorted.val <= curr.val) {
                // 大于所有 curr 的位置就是已排序好的最后位置
                lastSorted = lastSorted.next;
            } else {
                ListNode prev = dummy;
                while (prev.next.val <= curr.val) prev = prev.next;
                lastSorted.next = curr.next;
                curr.next = prev.next;
                prev.next = curr;
            }
            curr = lastSorted.next;
        }
        return dummy.next;
    }

    private static void bi_dijkstra(Map<Integer, Map<Integer, Integer>> graph, int startVertex, int endVertex) {

        List<Integer> shortest_path = new ArrayList<>();
        int shortest_path_length = Integer.MAX_VALUE;

        PriorityQueue<GraphNode> forward_pq = new PriorityQueue<>();
        PriorityQueue<GraphNode> backward_pq = new PriorityQueue<>();

        Map<Integer, Integer> forward_dists = new HashMap<>();
        Map<Integer, Integer> backward_dists = new HashMap<>();

        Map<Integer, Integer> forward_prev = new HashMap<>();
        Map<Integer, Integer> backward_prev = new HashMap<>();

        for (int vertex : graph.keySet()) {
            forward_dists.put(vertex, Integer.MAX_VALUE);
            backward_dists.put(vertex, Integer.MAX_VALUE);
            backward_prev.put(vertex, null);
            forward_prev.put(vertex, null);
        }

        forward_dists.put(startVertex, 0);
        backward_dists.put(endVertex, 0);
        forward_pq.add(new GraphNode(startVertex, 0));
        backward_pq.add(new GraphNode(endVertex, 0));

        boolean isFound = false;

        while (!backward_pq.isEmpty() && !forward_pq.isEmpty() && !isFound) {
            expandGraph(graph, forward_pq, backward_pq, forward_dists, backward_dists, forward_prev, backward_prev);

            if (backward_pq.isEmpty() || forward_pq.isEmpty()) break;

            int min_point_distance = forward_pq.peek().distance + backward_pq.peek().distance;

            for (Integer forward_node : forward_prev.keySet()) {
                if (forward_prev.get(forward_node) == null || backward_prev.get(forward_node) == null) continue;
                // 存在相交路径
                int min_path_distance = forward_dists.get(forward_node) + backward_dists.get(forward_node);
                if (min_path_distance <= min_point_distance) {
                    shortest_path = generate_shortest_path(forward_node, forward_prev, backward_prev);
                    shortest_path_length = min_path_distance;
                    isFound = true;
                    break;
                }
            }
        }

        if (shortest_path.isEmpty()) {
            System.out.println("no path found!");
        } else {
            System.out.printf("The shortest path from %d to %d is:\n", startVertex, endVertex);
            System.out.println(shortest_path);
            System.out.println("Length: " + shortest_path_length);
        }
    }

    private static List<Integer> generate_shortest_path(Integer common_node,
                                                        Map<Integer, Integer> forward_prev, Map<Integer, Integer> backward_prev) {
        List<Integer> shortest_path = new ArrayList<>();
        for (Integer v = common_node; v != null; v = forward_prev.get(v)) {
            shortest_path.add(v);
        }
        Collections.reverse(shortest_path);
        for (Integer v = backward_prev.get(common_node); v != null; v = backward_prev.get(v)) {
            shortest_path.add(v);
        }
        return shortest_path;
    }

    private static void expandGraph(Map<Integer, Map<Integer, Integer>> graph,
                                    PriorityQueue<GraphNode> forward_pq, PriorityQueue<GraphNode> backward_pq,
                                    Map<Integer, Integer> forward_dists, Map<Integer, Integer> backward_dists,
                                    Map<Integer, Integer> forward_prev, Map<Integer, Integer> backward_prev) {
        assert !forward_pq.isEmpty();
        assert !backward_pq.isEmpty();

        GraphNode forward_node = forward_pq.peek();
        GraphNode backward_node = backward_pq.peek();

        GraphNode curr_node;
        Map<Integer, Integer> distances;
        Map<Integer, Integer> prev;
        PriorityQueue<GraphNode> pq;

        if (forward_node.distance < backward_node.distance) {
            forward_pq.poll();
            curr_node = forward_node;
            distances = forward_dists;
            prev = forward_prev;
            pq = forward_pq;
            expandSingleGraph(graph, pq, distances, prev, curr_node);
        } else if (forward_node.distance > backward_node.distance) {
            backward_pq.poll();
            curr_node = backward_node;
            distances = backward_dists;
            prev = backward_prev;
            pq = backward_pq;
            expandSingleGraph(graph, pq, distances, prev, curr_node);
        } else {
            forward_pq.poll();
            backward_pq.poll();
            expandSingleGraph(graph, forward_pq, forward_dists, forward_prev, forward_node);
            expandSingleGraph(graph, backward_pq, backward_dists, backward_prev, backward_node);
        }
    }

    private static void expandSingleGraph(Map<Integer, Map<Integer, Integer>> graph, PriorityQueue<GraphNode> pq,
                                          Map<Integer, Integer> distances, Map<Integer, Integer> prev, GraphNode curr_node) {
        Integer vertex = curr_node.vertex;
        for (int neighbor : graph.get(vertex).keySet()) {
            int newDistance = distances.get(vertex) + graph.get(vertex).get(neighbor);
            if (newDistance < distances.get(neighbor)) {
                distances.put(neighbor, newDistance);
                prev.put(neighbor, vertex);
                pq.add(new GraphNode(neighbor, newDistance));
            }
        }
    }


    private static void dijkstra(Map<Integer, Map<Integer, Integer>> graph, int startVertex) {
        Map<Integer, Integer> distances = new HashMap<>();
        Map<Integer, Integer> prev = new HashMap<>();
        PriorityQueue<GraphNode> pq = new PriorityQueue<>();

        for (int vertex : graph.keySet()) {
            distances.put(vertex, Integer.MAX_VALUE);
            prev.put(vertex, null);
        }

        distances.put(startVertex, 0);
        pq.add(new GraphNode(startVertex, 0));

        while (!pq.isEmpty()) {
            GraphNode currNode = pq.poll();
            Integer vertex = currNode.vertex;
            for (int neighbor : graph.get(vertex).keySet()) {
                int newDistance = distances.get(vertex) + graph.get(vertex).get(neighbor);
                if (newDistance < distances.get(neighbor)) {
                    distances.put(neighbor, newDistance);
                    prev.put(neighbor, currNode.vertex);
                    pq.add(new GraphNode(neighbor, newDistance));
                }
            }
        }

        printPath(prev, startVertex);
        printDistances(distances);
    }

    // 添加边到图中
    private static void addEdge(Map<Integer, Map<Integer, Integer>> graph, int from, int to, int weight) {
        graph.putIfAbsent(from, new HashMap<>());
        graph.get(from).put(to, weight);
    }

    // 打印图的边和权重
    private static void printGraph(Map<Integer, Map<Integer, Integer>> graph) {
        for (Map.Entry<Integer, Map<Integer, Integer>> entry : graph.entrySet()) {
            int vertex = entry.getKey();
            for (Map.Entry<Integer, Integer> neighbor : entry.getValue().entrySet()) {
                System.out.println("v" + vertex + " -> v" + neighbor.getKey() + " : " + neighbor.getValue());
            }
        }
    }

    private static void printPath(Map<Integer, Integer> prev, int startVertex) {
        System.out.println("The shortest path from " + startVertex + " to all other vertices is: ");
        for (Integer vertex : prev.keySet()) {
            List<Integer> path = new ArrayList<>();
            for (Integer v = vertex; v != null; v = prev.get(v)) {
                path.add(v);
            }
            Collections.reverse(path);
            System.out.println("Path to vertex " + vertex + ": " + path);
        }
    }

    private static void printDistances(Map<Integer, Integer> distances) {
        System.out.println("The distance of all vertices from startVertex: ");
        for (Map.Entry<Integer, Integer> entry : distances.entrySet()) {
            System.out.println("Vertex " + entry.getKey() + " distance from startVertex is " + entry.getValue());
        }
    }

    static class GraphNode implements Comparable<GraphNode> {
        Integer vertex;
        Integer distance;

        GraphNode(int i, int dis) {
            vertex = i;
            distance = dis;
        }

        GraphNode(int i) {
            vertex = i;
            distance = Integer.MAX_VALUE;
        }

        @Override
        public int compareTo(GraphNode o) {
            return this.distance - o.distance;
        }
    }

    static class ListNode {
        int val;
        ListNode next;

        ListNode(int x, ListNode next) {
            val = x;
            this.next = next;
        }

        ListNode(int x) {
            val = x;
            next = null;
        }

        @Override
        public String toString() {
            return "ListNode{" +
                    "val=" + val +
                    ", next=" + next +
                    '}';
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            ListNode listNode = (ListNode) o;
            return val == listNode.val && Objects.equals(next, listNode.next);
        }

        @Override
        public int hashCode() {
            return Objects.hash(val, next);
        }

        public static void printList(ListNode head) {
            ListNode node = head;
            while (node != null) {
                System.out.print(node.val + " ");
                node = node.next;
            }
        }
    }
}
