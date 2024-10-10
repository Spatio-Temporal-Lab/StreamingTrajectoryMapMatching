package org.urbcomp.cupid.db.algorithm.mapmatch.amm.inner;

import org.urbcomp.cupid.db.algorithm.mapmatch.amm.AMM;
import org.urbcomp.cupid.db.algorithm.shortestpath.SimpleManyToManyShortestPath;
import org.urbcomp.cupid.db.model.roadnetwork.Path;

import java.util.List;

public class PointsSet {
    private final AMMGPSPoint observation;
    private final List<Candidate> candidates_list;
    public double v0;
    private double min_path = Double.MAX_VALUE;
    public double score;
    public PointsSet con_previous = null;
    public Candidate matched_candidate;
    public Path matched_path;

    public PointsSet(AMMGPSPoint gps, List<Candidate> candidates) {
        observation = gps;
        observation.parent = this;
        candidates_list = candidates;
        for (Candidate candidate : candidates) {
            candidate.parent = this;
        }
    }

    public List<Candidate> getCandidates() {
        return candidates_list;
    }

    public AMMGPSPoint getObservation() {
        return observation;
    }

    public boolean setScore(PointsSet previous) {
        boolean connected = false;
        this.con_previous = previous;
        for (Candidate current : candidates_list) {
            // 方向和位置得分参数设置为 10
            current.setPositionScore(observation,10.0);
            if (!current.setVelocityScore(v0, AMM.getDeltaV())) continue;
            if (current.getMinLength() < min_path) min_path = current.getMinLength();
            connected = true;
        }
        return connected;
    }

    public void backward() {
        System.out.println("fix and backward");
        if (con_previous == null) return;
        PointsSet prev_last = con_previous.con_previous;
        SimpleManyToManyShortestPath algo = new SimpleManyToManyShortestPath(AMM.getRoadNetwork());
        while (prev_last != null) {
            double new_length = min_path;
            for (Candidate current : candidates_list) {
                for (Candidate past : prev_last.getCandidates()) {
                    Path path = algo.findShortestPathBetweenCandidates(past.candidate, current.candidate);
                    if (!path.getPoints().isEmpty()) {
                        double length = path.getLengthInMeter() + past.getMinLength();
                        if (length < new_length) {
                            new_length = length;
                            current.best_previous = past;
                            con_previous = prev_last;
                        }
                    }
                }
            }
            // 如果最短路径没有变短，则退出回溯
            if (new_length < min_path) {
                min_path = new_length;
                prev_last = prev_last.con_previous;
            } else break;
        }
    }
}
