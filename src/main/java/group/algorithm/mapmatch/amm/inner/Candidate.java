package group.algorithm.mapmatch.amm.inner;

import group.algorithm.mapmatch.amm.AmmMapMatcher;
import group.algorithm.shortestpath.SimpleManyToManyShortestPath;
import group.model.point.CandidatePoint;
import group.model.roadnetwork.Path;
import group.util.MapUtil;

import java.util.List;

public class Candidate {

    public CandidatePoint candidate;
    public PointsSet parent;
    public double probability;
    public double min_length = Double.MAX_VALUE;
    public Candidate best_previous = null;
    public Path best_path = null;
    public double velocity_score;
    public double position_score;
    public int index;

    public Candidate(CandidatePoint point) {
        candidate = point;
    }

    public void setPositionScore(AMMGPSPoint observation, double sigma_z) {
        double pos_delta = MapUtil.calculateDistance(candidate, observation);
        position_score = calculatePositionScore(pos_delta, sigma_z);
    }

    public boolean setVelocityScore(double v0, double delta_v) {
        if (parent.con_previous == null) {
            velocity_score = calculateVelocityScore(v0, v0, delta_v);
            min_length = 0;
        } else {
            List<Candidate> previous_list = parent.con_previous.getCandidates();
            SimpleManyToManyShortestPath algo = new SimpleManyToManyShortestPath(AmmMapMatcher.getRoadNetwork());
            velocity_score = 0;
            for (Candidate start : previous_list) {

                Path path = algo.findShortestPathBetweenCandidates(start.candidate, this.candidate);
                double score = 0;
                if (!path.getPoints().isEmpty()) {
                    double length = path.getLengthInMeter();

                    if (length + start.min_length < min_length) {
                        min_length = length + start.min_length;
                        best_previous = start;
                        best_path = path;
                    }

                    double velocity = length / (double) (parent.getObservation().getTimestamp() - parent.con_previous.getObservation().getTimestamp());
                    score = calculateVelocityScore(velocity, v0, delta_v);
                }

                velocity_score += score * start.probability;
            }
        }
        return velocity_score > 0;
    }

    private double calculatePositionScore(double pos_delta, double sigma_z) {
        double fraction = Math.pow(pos_delta / sigma_z, 2.0);
        return Math.sqrt(2.0) * Math.exp(-fraction / 2.0);
    }

    private double calculateVelocityScore(double velocity, double v0, double deltaV) {
        double vMax = v0 + deltaV;
        double c = Math.pow(v0 + vMax, 2.0) * 3.0 / (v0 * 8.0 + vMax * 4.0);
        if (velocity < v0) return c * 2.0 / (v0 + vMax);
        else if (velocity < vMax) return c * 2.0 * (vMax - velocity) / (Math.pow(vMax, 2.0) - Math.pow(v0, 2.0));
        else return 0.0;
    }

    public double getMinLength() {
        return min_length;
    }
}
