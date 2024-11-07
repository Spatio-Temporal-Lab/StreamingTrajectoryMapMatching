/*
 * Copyright (C) 2022  ST-Lab
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner;

import org.urbcomp.cupid.db.algorithm.weightAdjuster.WeightAdjuster;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import scala.Tuple2;

import java.util.*;

public class TiViterbi {

    public Map<CandidatePoint, ExtendedState> lastExtendedStates;

    public List<CandidatePoint> prevCandidates;

    public Map<CandidatePoint, Double> message;

    public Boolean isBroken = false;

    public TiViterbi(Map<CandidatePoint, ExtendedState> lastExtendedStates, List<CandidatePoint> prevCandidates, Map<CandidatePoint, Double> message, Boolean isBroken) {
        this.lastExtendedStates = lastExtendedStates;
        this.prevCandidates = prevCandidates;
        this.message = message;
        this.isBroken = isBroken;
    }

    public TiViterbi() {
    }


    private void initializeStateProbabilities(
            GPSPoint observation,
            List<CandidatePoint> candidates,
            Map<CandidatePoint, Double> initialLogProbabilities
    ) {

        Map<CandidatePoint, Double> initialMessage = new HashMap<>(initialLogProbabilities.size());


        for (CandidatePoint candidate : candidates) {
            if (!initialLogProbabilities.containsKey(candidate))
                throw new IllegalArgumentException("No initial probability for " + candidate);
            double logProbability = initialLogProbabilities.get(candidate);
            initialMessage.put(candidate, logProbability);
        }

        isBroken = hmmBreak(initialMessage);
        if (isBroken) return;

        message = initialMessage;
        lastExtendedStates = new HashMap<>(candidates.size());
        prevCandidates = new ArrayList<>(candidates.size());

        for (CandidatePoint candidate : candidates) {
            lastExtendedStates.put(candidate, new ExtendedState(candidate, null, observation));
            prevCandidates.add(candidate);
        }
    }


    protected Boolean hmmBreak(Map<CandidatePoint, Double> message) {
        for (Double logProbability : message.values())
            if (!logProbability.equals(Double.NEGATIVE_INFINITY)) return false;
        return true;
    }


    private ForwardStepResult forwardStep(
            GPSPoint observation,
            List<CandidatePoint> prevCandidates,
            List<CandidatePoint> curCandidates,
            Map<CandidatePoint, Double> message,
            Map<CandidatePoint, Double> emissionLogProbabilities,
            Map<Tuple2<CandidatePoint, CandidatePoint>, Double> transitionLogProbabilities,
            WeightAdjuster weightAdjuster
    ) {
        final ForwardStepResult result = new ForwardStepResult(curCandidates.size());
        assert !prevCandidates.isEmpty();


        double accumulatedEmissionLogProb = 0.0;
        double accumulatedTransitionLogProb = 0.0;

        for (CandidatePoint curState : curCandidates) {
            double maxTransitionLogProb = Double.NEGATIVE_INFINITY;
            double maxLogProb = Double.NEGATIVE_INFINITY;
            CandidatePoint maxPreState = null;
            for (CandidatePoint preState : prevCandidates) {
                double transitionLogProb = transitionLogProbability(
                        preState,
                        curState,
                        transitionLogProbabilities
                );
                final double logProb = message.get(preState) + weightAdjuster.getTransitionWeight() * transitionLogProb;
                if (logProb > maxLogProb) {
                    maxTransitionLogProb = transitionLogProb;
                    maxLogProb = logProb;
                    maxPreState = preState;
                }
            }
            double emissionLogProb = emissionLogProbabilities.get(curState);
            result.getNewMessage()
                    .put(curState, (maxLogProb + weightAdjuster.getEmissionWeight() * emissionLogProb));
            if (maxTransitionLogProb != Double.NEGATIVE_INFINITY && emissionLogProb != Double.NEGATIVE_INFINITY) {
                accumulatedEmissionLogProb += emissionLogProb;
                accumulatedTransitionLogProb += maxTransitionLogProb;
            }

            // Note that max_prev_state == None if there is no transition with non-zero probability.
            // In this case cur_state has zero probability and will not be part of the most likely
            // sequence,
            // so we don't need an ExtendedState.

            if (maxPreState != null) {
                final ExtendedState extendedState = new ExtendedState(
                        curState,
                        lastExtendedStates.get(maxPreState),
                        observation
                );
                result.getNewExtendedStates().put(curState, extendedState);
            }
        }

        if (accumulatedTransitionLogProb != Double.NEGATIVE_INFINITY && accumulatedEmissionLogProb != Double.NEGATIVE_INFINITY) {
            weightAdjuster.updateWeights(accumulatedEmissionLogProb, accumulatedTransitionLogProb);
        }
        return result;
    }

    protected double transitionLogProbability(
            CandidatePoint preState,
            CandidatePoint curState,
            Map<Tuple2<CandidatePoint, CandidatePoint>, Double> transitionLogProbabilities
    ) {
        Tuple2<CandidatePoint, CandidatePoint> transition = new Tuple2<>(preState, curState);
        return transitionLogProbabilities.getOrDefault(transition, Double.NEGATIVE_INFINITY);
    }


    private CandidatePoint mostLikelyState() {
        assert !message.isEmpty();
        CandidatePoint result = null;
        Double maxLogProbability = Double.NEGATIVE_INFINITY;
        for (Map.Entry<CandidatePoint, Double> entry : message.entrySet()) {
            if (entry.getValue() > maxLogProbability) {
                result = entry.getKey();
                maxLogProbability = entry.getValue();
            }
        }
        assert result != null;
        return result;
    }


    private List<SequenceState> retrieveMostLikelySequence() {
        assert !message.isEmpty();
        CandidatePoint lastState = mostLikelyState();
        List<SequenceState> result = new ArrayList<>();
        ExtendedState es = lastExtendedStates.get(lastState);
        while (es != null) {
            SequenceState ss = new SequenceState(es.getState(), es.getObservation());
            result.add(ss);
            es = es.getBackPointer();
        }
        Collections.reverse(result);
        return result;
    }


    public void startWithInitialObservation(
            GPSPoint observation,
            List<CandidatePoint> candidates,
            Map<CandidatePoint, Double> emissionLogProbabilities
    ) {
        initializeStateProbabilities(observation, candidates, emissionLogProbabilities);
    }


    public void nextStep(
            GPSPoint observation,
            List<CandidatePoint> candidates,
            Map<CandidatePoint, Double> emissionLogProbabilities,
            Map<Tuple2<CandidatePoint, CandidatePoint>, Double> transitionLogProbabilities,
            WeightAdjuster weightAdjuster
    ) {
        if (message == null) {
            throw new IllegalStateException(
                    "start with initial observation() must be called first."
            );
        }
        if (isBroken) {
            throw new IllegalStateException("Method must not be called after an HMM break.");
        }
        ForwardStepResult forwardStepResult = forwardStep(
                observation,
                prevCandidates,
                candidates,
                message,
                emissionLogProbabilities,
                transitionLogProbabilities,
                weightAdjuster
        );
        isBroken = hmmBreak(forwardStepResult.getNewMessage());
        if (isBroken) {
            return;
        }
        message = forwardStepResult.getNewMessage();
        lastExtendedStates = forwardStepResult.getNewExtendedStates();
        prevCandidates = new ArrayList<>(candidates);
    }


    public List<SequenceState> computeMostLikelySequence() {
        if (message == null) {
            return new ArrayList<>();
        } else {
            return retrieveMostLikelySequence();
        }
    }
}
