package org.urbcomp.cupid.db.algorithm.mapmatch.online.viterbi;

import org.urbcomp.cupid.db.algorithm.mapmatch.online.inner.*;
import org.urbcomp.cupid.db.algorithm.mapmatch.online.inner.twowaylink.TwoWayLinkedList;

import java.util.*;

public class OnlineViterbiDecoder {
    private final int K;
    private final int T;
    private final LinkedList<OnlineViterbiProbNode<Double>> probList;
    private final LinkedList<OnlineViterbiStateNode<Integer>> stateList;
    private final LinkedList<OnlineNode<Integer>> nodeList;
    private OnlineNode<Integer> root;
    private OnlineNode<Integer> prevRoot;
    private int deltaT;
    private List<Integer> decodedStream;

    public OnlineViterbiDecoder(int K, int T) {
        this.K = K;
        this.T = T;
        this.probList = new TwoWayLinkedList<>();
        this.stateList = new TwoWayLinkedList<>();
        this.nodeList = new TwoWayLinkedList<>();
        this.root = null;
        this.prevRoot = null;
        this.deltaT = 0;
        this.decodedStream = new ArrayList<>();
    }

    private void clearAllLists() {
        this.probList.clear();
        this.stateList.clear();
        this.nodeList.clear();
    }

    public void initialization(int startingState, List<Double> initial) {
        this.root = null;
        this.prevRoot = null;
        this.decodedStream.clear();
        clearAllLists();

        List<Double> initialProb = new ArrayList<>();
        for (Double prob : initial) {
            initialProb.add(Auxiliary.boundedLog(prob));
        }

        List<Integer> initialState = new ArrayList<>();
        for (int i = 0; i < this.K; i++) {
            initialState.add(startingState);
        }

        this.probList.add(new OnlineViterbiProbNode<>(initialProb));
        this.stateList.add(new OnlineViterbiStateNode<>(initialState));
    }

    private void compress(int currentTime) {
        OnlineNode<Integer> current = this.nodeList.getLast();
        while (current != null) {
//            int state = current.state;
            int time = current.time;
            OnlineNode<Integer> parent = current.parent;
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

    private void freeDummyNodes(int currentTime) {
        OnlineNode<Integer> current = this.nodeList.getLast();
        while (current != null) {
            int time = current.time;
            int numChild = current.numChild;
            OnlineNode<Integer> prev = current.prev;
            if (numChild <= 0 && time != currentTime) this.nodeList.remove(current);
            current = prev;
        }
    }

    private boolean findNewRoot() {
        if (this.root == null) {
            List<OnlineNode<Integer>> tracedRoot = findTracedRoot();

            boolean result = false;
            if (!tracedRoot.isEmpty()) {
                // check if exists convergence point
                result = tracedRoot.stream().allMatch(elem -> elem.equals(tracedRoot.get(0)));
            }

            if (!result) {
                return false;
            }
        }

        OnlineNode<Integer> current = this.nodeList.getLast();
        OnlineNode<Integer> aux = null;

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

    private List<OnlineNode<Integer>> findTracedRoot() {
        List<OnlineNode<Integer>> tracedRoot = new ArrayList<>();
        OnlineNode<Integer> leaf = this.nodeList.getLast();

        for (int i = 0; i < this.K; i++) {
            OnlineNode<Integer> current = leaf;
            while (current != null) {
                OnlineNode<Integer> parent = current;
                current = current.parent;
                if (current == null) {
                    tracedRoot.add(parent);
                }
            }
            leaf = leaf.prev;
        }

        return tracedRoot;
    }

    private void traceback() {
        List<Integer> completeSolution = this.decodedStream;
        List<Integer> solution = new ArrayList<>();
        OnlineViterbiProbNode<Double> pCol = this.probList.getLast();
        OnlineViterbiStateNode<Integer> sCol = this.stateList.getLast();

        int output = this.root.state;
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
            OnlineViterbiStateNode<Integer> tempS = sCol;
            OnlineViterbiProbNode<Double> tempP = pCol;
            sCol = sCol.prev;
            pCol = pCol.prev;
            this.stateList.remove(tempS);
            this.probList.remove(tempP);
        }

        // remove the rest states and probabilities
        while (pCol != null) {
            OnlineViterbiStateNode<Integer> tempS = sCol;
            OnlineViterbiProbNode<Double> tempP = pCol;
            sCol = sCol.prev;
            pCol = pCol.prev;
            this.stateList.remove(tempS);
            this.probList.remove(tempP);
        }

        Collections.reverse(solution);
        completeSolution.addAll(solution);
        this.decodedStream = completeSolution;
//        System.out.println("partial trace:" + this.decodedStream);
    }

    public void traceLastPart() {
        List<Integer> completeSolution = this.decodedStream;
        List<Integer> solution = new ArrayList<>();
        OnlineViterbiProbNode<Double> pCol = this.probList.getLast();
        OnlineViterbiStateNode<Integer> sCol = this.stateList.getLast();

        List<Double> probs = pCol.probs;
        int output = 0;
        double max = probs.get(0);
        for (int i = 1; i < probs.size(); i++) {
            if (probs.get(i) > max) {
                max = probs.get(i);
                output = i;
            }
        }
        solution.add(output);

        int depth = this.root != null ? ((this.T - 1) - this.root.time - 1) : (this.T - 1);

        for (int i = 0; i < depth; i++) {
            output = sCol.states.get(output);
            solution.add(output);
            sCol = sCol.prev;
        }

        Collections.reverse(solution);
        completeSolution.addAll(solution);

        this.decodedStream = completeSolution;
//        System.out.println("complete trace:" + this.decodedStream);
    }

    public void update(int t, int observation, List<List<Double>> A, List<List<Double>> E) {
        OnlineViterbiProbNode<Double> pCol = this.probList.getLast();
        OnlineViterbiStateNode<Integer> sCol = this.stateList.getLast();
        int nodeSize = this.nodeList.size();
        List<Double> newProbs = new ArrayList<>();
        List<Integer> newStates = new ArrayList<>();

        for (int j = 0; j < this.K; j++) {
            double maxVal = Double.NEGATIVE_INFINITY;
            int maxIndex = 0;

            for (int i = 0; i < this.K; i++) {
                double aux = Auxiliary.boundedLogSum(
                        pCol.probs.get(i),
                        Auxiliary.boundedLog(A.get(i).get(j)),
                        Auxiliary.boundedLog(E.get(j).get(observation)));
                if (aux > maxVal) {
                    maxVal = aux;
                    maxIndex = i;
                }
            }

            newProbs.add(maxVal);
            newStates.add(maxIndex);

            OnlineNode<Integer> parentNode;
            if (t == 0) {
                parentNode = null;
            } else {
                parentNode = this.nodeList.get(nodeSize - this.K + maxIndex); // find the right parent node
                parentNode.numChild = parentNode.numChild + 1;
            }

            this.nodeList.add(new OnlineNode<Integer>(t, j, 0, parentNode));
        }

        this.probList.add(new OnlineViterbiProbNode<Double>(newProbs));
        this.stateList.add(new OnlineViterbiStateNode<Integer>(newStates));

        compress(t);
        freeDummyNodes(t);

        if (findNewRoot()) {
            traceback();
        }
    }

    public List<Integer> getDecodedStream() {
        return this.decodedStream;
    }

    public void printProbList() {
        for (int i = this.probList.size() - 1; i >= 0; i--) {
            System.out.println(this.probList.get(i));
        }
        System.out.println("\n\n");
    }

    public void printStateList() {
        for (int i = this.stateList.size() - 1; i >= 0; i--) {
            System.out.println(this.stateList.get(i));
        }
        System.out.println("\n\n");
    }

    public void printNodeList() {
        for (int i = this.nodeList.size() - 1; i >= 0; i--) {
            System.out.println(this.nodeList.get(i));
        }
        System.out.println("\n\n");
    }
}











