package group.algorithm.mapmatch.tihmm.inner;

import group.model.point.CandidatePoint;

import java.util.HashMap;
import java.util.Map;

public class ForwardStepResult {

    private final Map<CandidatePoint, Double> newMessage;

    private final Map<CandidatePoint, ExtendedState> newExtendedStates;

    public ForwardStepResult(int numberStates) {
        newMessage = new HashMap<>(numberStates);
        newExtendedStates = new HashMap<>(numberStates);
    }

    public Map<CandidatePoint, Double> getNewMessage() {
        return newMessage;
    }

    public Map<CandidatePoint, ExtendedState> getNewExtendedStates() {
        return newExtendedStates;
    }
}
