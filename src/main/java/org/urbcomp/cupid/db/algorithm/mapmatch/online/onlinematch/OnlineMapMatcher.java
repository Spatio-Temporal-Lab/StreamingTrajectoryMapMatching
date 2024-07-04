package org.urbcomp.cupid.db.algorithm.mapmatch.online.onlinematch;

import org.urbcomp.cupid.db.algorithm.mapmatch.online.inner.*;
import org.urbcomp.cupid.db.algorithm.mapmatch.online.inner.twowaylink.TwoWayLinkedList;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.HmmProbabilities;
import org.urbcomp.cupid.db.algorithm.shortestpath.AbstractManyToManyShortestPath;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import org.urbcomp.cupid.db.model.point.MapMatchedPoint;
import org.urbcomp.cupid.db.model.roadnetwork.Path;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNetwork;
import org.urbcomp.cupid.db.model.roadnetwork.RoadNode;
import org.urbcomp.cupid.db.model.roadnetwork.RoadSegment;
import org.urbcomp.cupid.db.model.trajectory.MapMatchedTrajectory;
import org.urbcomp.cupid.db.model.trajectory.Trajectory;
import org.urbcomp.cupid.db.util.GeoFunctions;

import java.util.*;

/**
 * 在线地图匹配算法的实现类。
 */
public class OnlineMapMatcher {
    /**
     * 计算观测和转移概率
     */
    private static HmmProbabilities probabilities;
    /**
     * 路网
     */
    private final RoadNetwork roadNetwork;
    /**
     * 最短路径算法
     */
    private final AbstractManyToManyShortestPath pathAlgo;
    /**
     * 概率表
     */
    private final LinkedList<OnlineMapProbNode<CandidatePoint, Double>> probList;
    /**
     * 状态表
     */
    private final LinkedList<OnlineMapStateNode<CandidatePoint, CandidatePoint>> stateList;
    /**
     * 隐结点表
     */
    private final LinkedList<OnlineNode<CandidatePoint>> nodeList;
    /**
     * 收敛点
     */
    private OnlineNode<CandidatePoint> root;
    /**
     * 上一个收敛点
     */
    private OnlineNode<CandidatePoint> prevRoot;
    /**
     * 当前时间到收敛点所在时间的差值
     */
    private int deltaT;
    /**
     * 地图匹配结果
     */
    private final List<CandidatePoint> matchRes;
    /**
     * 当前时间候选点
     */
    private List<CandidatePoint> currCandiPts;
    /**
     * 上一时间候选点
     */
    private List<CandidatePoint> preCandiPts;
    /**
     * 当前时间轨迹点
     */
    private GPSPoint currPt;
    /**
     * 上一时间轨迹点
     */
    private GPSPoint prePt;

    /**
     * 构造方法，初始化地图匹配器。
     *
     * @param roadNetwork 路网对象。
     * @param pathAlgo 最短路径算法对象。
     */
    public OnlineMapMatcher(RoadNetwork roadNetwork, AbstractManyToManyShortestPath pathAlgo) {
        this.roadNetwork = roadNetwork;
        this.pathAlgo = pathAlgo;
        this.probList = new TwoWayLinkedList<>();
        this.stateList = new TwoWayLinkedList<>();
        this.nodeList = new TwoWayLinkedList<>();
        this.root = null;
        this.prevRoot = null;
        this.deltaT = 0;
        this.matchRes = new ArrayList<>();
        this.currCandiPts = new ArrayList<>();
        this.preCandiPts = new ArrayList<>();
        this.currPt = null;
        this.prePt = null;
        probabilities = new HmmProbabilities(50.0, 2.0);
    }

    /**
     * 构造方法，初始化地图匹配器。
     *
     * @param roadNetwork 路网对象。
     * @param pathAlgo 最短路径算法对象。
     * @param sigma HMM观测概率的参数。
     * @param beta HMM转移概率的参数。
     */
    public OnlineMapMatcher(RoadNetwork roadNetwork, AbstractManyToManyShortestPath pathAlgo, double sigma, double beta) {
        this.roadNetwork = roadNetwork;
        this.pathAlgo = pathAlgo;
        this.probList = new TwoWayLinkedList<>();
        this.stateList = new TwoWayLinkedList<>();
        this.nodeList = new TwoWayLinkedList<>();
        this.root = null;
        this.prevRoot = null;
        this.deltaT = 0;
        this.matchRes = new ArrayList<>();
        this.currCandiPts = new ArrayList<>();
        this.preCandiPts = new ArrayList<>();
        this.currPt = null;
        this.prePt = null;
        probabilities = new HmmProbabilities(sigma, beta);
    }

    /**
     * 清空所有列表。
     */
    private void clearAllLists() {
        this.matchRes.clear();
        this.probList.clear();
        this.stateList.clear();
        this.nodeList.clear();
        this.currCandiPts.clear();
        this.preCandiPts.clear();
    }

    /**
     * 初始化地图匹配器。
     *
     * @param pt 初始GPS点。
     */
    public void initialization(GPSPoint pt) {
        this.root = null;
        this.prevRoot = null;
        this.currPt = null;
        this.prePt = null;

        clearAllLists();

        Map<CandidatePoint, Double> initialProb = new LinkedHashMap<>();
        Map<CandidatePoint, CandidatePoint> initialState = new LinkedHashMap<>();

        List<CandidatePoint> candidates = CandidatePoint.getCandidatePoint(pt, roadNetwork, probabilities.getSigma());

        for (CandidatePoint candiPt : candidates) {
            initialState.put(candiPt, null);
            currCandiPts.add(candiPt);
            double dist = candiPt.getErrorDistanceInMeter();
            initialProb.put(candiPt, probabilities.emissionLogProbability(dist));
        }

        preCandiPts = currCandiPts;
        prePt = currPt;
        currPt = pt;

        this.stateList.add(new OnlineMapStateNode<>(initialState));
        this.probList.add(new OnlineMapProbNode<>(initialProb));
    }

    /**
     * 压缩节点列表，移除无用节点。
     *
     * @param currentTime 当前时间。
     */
    private void compress(int currentTime) {
        OnlineNode<CandidatePoint> current = this.nodeList.getLast();
        while (current != null) {
//            int state = current.state;
            int time = current.time;
            OnlineNode<CandidatePoint> parent = current.parent;
            int numChild = current.numChild;

            if (numChild == 0 && time != currentTime) {
                if (parent != null) {
                    parent.numChild = parent.numChild - 1;      // logic delete children
                }
            } else {
                while (parent != null && parent.numChild == 1) {
                    current.parent = current.parent.parent;     // logic delete children's parent
                    parent = current.parent;
                }
            }

            current = current.prev;
        }
    }

    /**
     * 释放无用节点。
     *
     * @param currentTime 当前时间。
     */
    private void freeDummyNodes(int currentTime) {
        OnlineNode<CandidatePoint> current = this.nodeList.getLast();
        while (current != null) {
            int time = current.time;
            int numChild = current.numChild;
            OnlineNode<CandidatePoint> prev = current.prev;
            if (numChild <= 0 && time != currentTime) this.nodeList.remove(current);
            current = prev;
        }
    }

    /**
     * 是否找到新的收敛点。
     * @return 真或者假
     */
    private boolean findNewRoot() {
        if (this.root == null) {
            List<OnlineNode<CandidatePoint>> tracedRoot = findTracedRoot();

            boolean result = false;
            if (!tracedRoot.isEmpty()) {
                // check if exists convergence point
                result = tracedRoot.stream().allMatch(elem -> elem.equals(tracedRoot.get(0)));
            }

            if (!result) {
                return false;
            }
        }

        OnlineNode<CandidatePoint> current = this.nodeList.getLast();
        OnlineNode<CandidatePoint> aux = null;

        this.deltaT = current.time;

        while (current != null) {
            if (current.numChild >= 2) aux = current; // convergence point
            current = current.parent;
        }

        if (aux != null) {
            if (this.root == null) {
                this.root = aux;
                this.deltaT = this.deltaT - aux.time;  // delta between last time and convergence time
                return this.deltaT != 0;    // confirm not the same time
            } else {
                if (!aux.equals(this.root)) {   // confirm not the same point
                    this.deltaT = this.deltaT - aux.time;
                    if (this.deltaT == 0) {
                        return false;
                    } else {
                        this.prevRoot = this.root;
                        this.root = aux;
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /**
     * 获取当前候选点的父节点列表。
     * @return 当前候选点的父节点
     */
    private List<OnlineNode<CandidatePoint>> findTracedRoot() {
        List<OnlineNode<CandidatePoint>> tracedRoot = new ArrayList<>();
        OnlineNode<CandidatePoint> leaf = this.nodeList.getLast();

        for (int i = 0; i < currCandiPts.size(); i++) {
            OnlineNode<CandidatePoint> current = leaf;
            while (current != null) {
                OnlineNode<CandidatePoint> parent = current;
                current = current.parent;
                if (current == null) {
                    tracedRoot.add(parent);
                }
            }
            leaf = leaf.prev;
        }

        return tracedRoot;
    }

    /**
     * 回溯找到部分解。
     */
    private void traceback() {
        List<CandidatePoint> solution = new ArrayList<>();

        OnlineMapProbNode<CandidatePoint, Double> pCol = this.probList.getLast();
        OnlineMapStateNode<CandidatePoint, CandidatePoint> sCol = this.stateList.getLast();

        CandidatePoint output = this.root.state;
        solution.add(output);

        for (int i = 0; i < this.deltaT; i++) {
            pCol = pCol != null ? pCol.prev : null;
            sCol = sCol != null ? sCol.prev : null;
        }

        // calculate the time delta between root and previous root
        int depth = this.prevRoot != null ? this.root.time - this.prevRoot.time - 1 : this.root.time;

        // remove states and probabilities that have been used
        for (int i = 0; i < depth; i++) {
            output = sCol.states.get(output);
            solution.add(output);
            OnlineMapStateNode<CandidatePoint, CandidatePoint> tempS = sCol;
            OnlineMapProbNode<CandidatePoint, Double> tempP = pCol;
            sCol = sCol.prev;
            pCol = pCol.prev;
            this.stateList.remove(tempS);
            this.probList.remove(tempP);
        }

        // remove the rest states and probabilities
        while (pCol != null && sCol != null) {
            OnlineMapStateNode<CandidatePoint, CandidatePoint> tempS = sCol;
            OnlineMapProbNode<CandidatePoint, Double> tempP = pCol;
            sCol = sCol.prev;
            pCol = pCol.prev;
            this.stateList.remove(tempS);
            this.probList.remove(tempP);
        }

        Collections.reverse(solution);
        this.matchRes.addAll(solution);
        System.out.println("partial trace:" + this.matchRes);
    }

    /**
     * 到达最后一个轨迹点，直接回溯。
     * @param lastTime 最后一个轨迹点对应的时间。
     */
    public void traceLastPart(int lastTime) {
        List<CandidatePoint> solution = new ArrayList<>();

        OnlineMapProbNode<CandidatePoint, Double> pCol = this.probList.getLast();
        OnlineMapStateNode<CandidatePoint, CandidatePoint> sCol = this.stateList.getLast();

        Map<CandidatePoint, Double> probs = pCol.probs;
        CandidatePoint output = null;    // null symbols not exists

        double maxProb = Double.NEGATIVE_INFINITY;

        for (CandidatePoint candiPts : probs.keySet()) {
            double prob = probs.get(candiPts);
            if (prob > maxProb) {
                maxProb = prob;
                output = candiPts;
            }
        }

        solution.add(output);

        int depth = this.root != null ? (lastTime - this.root.time - 1) : lastTime;

        for (int i = 0; i < depth; i++) {
            output = sCol.states.get(output);
            solution.add(output);
            sCol = sCol.prev;
        }

        Collections.reverse(solution);
        this.matchRes.addAll(solution);

        System.out.println("complete trace:" + this.matchRes);
    }

    /**
     * 更新维特比序列。
     * @param t 当前时间
     * @param pt 当前轨迹点
     */
    public void update(int t, GPSPoint pt) {
        OnlineMapProbNode<CandidatePoint, Double> pCol = this.probList.getLast();
        OnlineMapStateNode<CandidatePoint, CandidatePoint> sCol = this.stateList.getLast();

        Map<CandidatePoint, Double> newProbs = new LinkedHashMap<>();
        Map<CandidatePoint, CandidatePoint> newStates = new LinkedHashMap<>();

        preCandiPts = currCandiPts;
        currCandiPts = CandidatePoint.getCandidatePoint(pt, roadNetwork, probabilities.getSigma());

//        if (currCandiPts.isEmpty()) {
//            System.out.println("time="+t+":"+"candidate points=0");
//        }

        prePt = currPt;
        currPt = pt;

        int nodeSize = this.nodeList.size();
        int prevSize = preCandiPts.size();

        Map<RoadNode, Map<RoadNode, Path>> candidatePaths = getCandidatePaths();

        for (int j = 0; j < currCandiPts.size(); j++) {
            double maxProb = Double.NEGATIVE_INFINITY;
            int maxIdx = -1;
            CandidatePoint maxState = null;
            CandidatePoint currCandiPt = currCandiPts.get(j);

            for (int i = 0; i < preCandiPts.size(); i++) {
                CandidatePoint preCandiPt = preCandiPts.get(i);

//                System.out.println("time:" + t);
//                System.out.println("previous point id:"+ i);

                double transProb = computeTransitionProb(candidatePaths, preCandiPt, currCandiPt);
                double emissionProb = probabilities.emissionLogProbability(currCandiPt.getErrorDistanceInMeter());
                double jointProb = pCol.probs.get(preCandiPt);

//                System.out.println("emission probability:"+ emissionProb);
//                System.out.println("transition probability:" + transProb);
//                System.out.println("joint probability:" + jointProb);

                double aux = transProb + emissionProb + jointProb;

                if (aux > maxProb) {
                    maxProb = aux;
                    maxState = preCandiPt;
                    maxIdx = i;
                }
            }

//            if (maxState == null) {
//                System.out.println("current point id:" + j + " maxState is null");
//            }

            newProbs.put(currCandiPt, maxProb);
            newStates.put(currCandiPt, maxState);

            OnlineNode<CandidatePoint> parentNode;
            if (t == 0 || maxState == null) {
                parentNode = null;
            } else {
                parentNode = this.nodeList.get(nodeSize - prevSize + maxIdx);
                parentNode.numChild = parentNode.numChild + 1;
            }

            this.nodeList.add(new OnlineNode<>(t, maxState, 0, parentNode));
        }

        this.probList.add(new OnlineMapProbNode<>(newProbs));
        this.stateList.add(new OnlineMapStateNode<>(newStates));

        compress(t);
        freeDummyNodes(t);

        if (findNewRoot()) {
            traceback();
        }

    }

    /**
     * 地图匹配算法。
     * @param trajectory 待匹配轨迹
     * @return mapMatchTrajectory 已匹配轨迹
     */
    public MapMatchedTrajectory mapMatch(Trajectory trajectory) {
        List<GPSPoint> gpsPointList = trajectory.getGPSPointList();
        initialization(gpsPointList.get(0));

        for (int i = 1; i < gpsPointList.size(); i++) {
            update(i - 1, gpsPointList.get(i));
        }

        traceLastPart(gpsPointList.size() - 1);

        List<CandidatePoint> matchRes = this.matchRes;

        List<MapMatchedPoint> mapMatchedPointList = new ArrayList<>(matchRes.size());

        for (int k = 0; k < matchRes.size(); k++) {
            mapMatchedPointList.add(new MapMatchedPoint(gpsPointList.get(k), matchRes.get(k)));
        }

        return new MapMatchedTrajectory(trajectory.getTid(), trajectory.getOid(), mapMatchedPointList);
    }

    /**
     * 获取候选节点组成的路径网。
     * @return 候选节点组成的路径网
     */
    private Map<RoadNode, Map<RoadNode, Path>> getCandidatePaths() {

        Set<CandidatePoint> startPoints = new LinkedHashSet<>(preCandiPts);
        Set<CandidatePoint> endPoints = new LinkedHashSet<>(currCandiPts);

        return pathAlgo.findShortestPath(startPoints, endPoints);
    }

    /**
     * 获取两个候选点的对应路径。
     * @param candidatePaths 候选路径网。
     * @param srcPt 上一个候选点。
     * @param destPt 当前候选点。
     * @return 两个候选点的对应路径。
     */
    private Path getPath(Map<RoadNode, Map<RoadNode, Path>> candidatePaths,
                        CandidatePoint srcPt, CandidatePoint destPt) {

        RoadSegment startRoadSegment = roadNetwork.getRoadSegmentById(srcPt.getRoadSegmentId());
        RoadSegment endRoadSegment = roadNetwork.getRoadSegmentById(destPt.getRoadSegmentId());
        Path subPath = candidatePaths.get(startRoadSegment.getEndNode()).get(endRoadSegment.getStartNode());

        return pathAlgo.getCompletePath(srcPt, destPt, subPath);
    }

    /**
     * 计算两个候选点的转移概率。
     * @param candidatePaths 候选路径网。
     * @param srcPt 上一候选点。
     * @param destPt 当前候选点。
     * @return 两个候选点的转移概率。
     */
    private double computeTransitionProb(Map<RoadNode, Map<RoadNode, Path>> candidatePaths,
                                        CandidatePoint srcPt, CandidatePoint destPt) {
        Path path = getPath(candidatePaths, srcPt, destPt);

        double linearDist = GeoFunctions.getDistanceInM(prePt, currPt);
        return probabilities.transitionLogProbability(path.getLengthInMeter(), linearDist);
    }

    /**
     * 打印概率表。
     */
    public void printProbList() {
        for (int i = this.probList.size() - 1; i >= 0; i--) {
            System.out.println(this.probList.get(i));
        }
        System.out.println("\n\n");
    }

    /**
     * 打印状态表。
     */
    public void printStateList() {
        for (int i = this.stateList.size() - 1; i >= 0; i--) {
            System.out.println(this.stateList.get(i));
        }
        System.out.println("\n\n");
    }

    /**
     * 打印维特比结点表。
     */
    public void printNodeList() {
        for (int i = this.nodeList.size() - 1; i >= 0; i--) {
            System.out.println(this.nodeList.get(i));
        }
        System.out.println("\n\n");
    }
}
