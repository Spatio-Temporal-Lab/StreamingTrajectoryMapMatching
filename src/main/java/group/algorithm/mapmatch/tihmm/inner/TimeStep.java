package group.algorithm.mapmatch.tihmm.inner;

import group.model.point.CandidatePoint;
import group.model.point.GPSPoint;
import scala.Tuple2;

import java.util.HashMap;
import java.util.List;
import java.util.Map;


public class TimeStep {

    private CandidatePoint match;

    private final GPSPoint observation;

    private final List<CandidatePoint> candidates;

    private final Map<CandidatePoint, Double> emissionLogProbabilities = new HashMap<>();

    private final Map<Tuple2<CandidatePoint, CandidatePoint>, Double> transitionLogProbabilities =
            new HashMap<>();

    public TimeStep(GPSPoint observation, List<CandidatePoint> candidates) {
        if (observation == null || candidates == null) {
            throw new NullPointerException("Observation and candidates must not be null.");
        }
        this.observation = observation;
        this.candidates = candidates;
    }


    public void addEmissionLogProbability(CandidatePoint candidate, Double emissionLogProbability) {
        if (emissionLogProbabilities.containsKey(candidate)) {
            throw new IllegalArgumentException("Candidate has already been added.");
        }
        emissionLogProbabilities.put(candidate, emissionLogProbability);
    }


    public void addTransitionLogProbability(
            CandidatePoint fromPosition,
            CandidatePoint toPosition,
            Double transitionLogProbability
    ) {
        final Tuple2<CandidatePoint, CandidatePoint> transition = new Tuple2<>(
                fromPosition,
                toPosition
        );

        if (transitionLogProbabilities.containsKey(transition)) {
            transitionLogProbabilities.remove(transition);
        }
        transitionLogProbabilities.put(transition, transitionLogProbability);
    }

    public GPSPoint getObservation() {
        return observation;
    }

    public List<CandidatePoint> getCandidates() {
        return candidates;
    }

    public Map<CandidatePoint, Double> getEmissionLogProbabilities() {
        return emissionLogProbabilities;
    }

    public Map<Tuple2<CandidatePoint, CandidatePoint>, Double> getTransitionLogProbabilities() {
        return transitionLogProbabilities;
    }

    public void addCandidate(CandidatePoint point) {
        candidates.add(point);
    }

    public CandidatePoint getMatch() {
        return this.match;
    }

    public void setMatch(CandidatePoint match) {
        this.match = match;
    }
}
