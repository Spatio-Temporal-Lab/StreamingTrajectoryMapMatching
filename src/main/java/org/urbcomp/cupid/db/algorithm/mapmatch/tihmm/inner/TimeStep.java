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

import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import scala.Tuple2;

import java.io.BufferedWriter;
import java.io.IOException;
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
