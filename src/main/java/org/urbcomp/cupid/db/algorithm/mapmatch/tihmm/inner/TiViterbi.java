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

import org.urbcomp.cupid.db.algorithm.mapmatch.stream.SlidingWindow;
import org.urbcomp.cupid.db.algorithm.mapmatch.stream.StreamMapMatcher;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import scala.Tuple2;

import java.util.*;

import static org.urbcomp.cupid.db.util.CalculateErrors.*;

/**
 * Tihmm 核心算法class
 */
public class TiViterbi {
    /**
     * 上一个状态下每一个candidate point 对应的extended state
     */
    public Map<CandidatePoint, ExtendedState> lastExtendedStates;
    /**
     * 上一个状态下的观测点
     */
    public GPSPoint prevObservation;
    /**
     * 上一个状态下原始轨迹点对应的所有candidate point
     */
    public List<CandidatePoint> prevCandidates;
    /**
     * 上一个状态下的匹配点
     */
    public CandidatePoint prevMatch;
    /**
     * 每个candidate point 对应的p
     */
    public Map<CandidatePoint, Double> message;
    /**
     * 是否停止初始化状态概率函数
     */
    public Boolean isBroken = false;
    /**
     * 滑动窗口
     */
    public SlidingWindow slidingWindow = new SlidingWindow(30);
    /**
     * 转移概率权重
     */
    public double weight = 0.5;

    public TiViterbi(Map<CandidatePoint, ExtendedState> lastExtendedStates, GPSPoint prevObservation, List<CandidatePoint> prevCandidates, Map<CandidatePoint, Double> message, Boolean isBroken){
        this.lastExtendedStates = lastExtendedStates;
        this.prevObservation = prevObservation;
        this.prevCandidates = prevCandidates;
        this.message = message;
        this.isBroken = isBroken;
    }
    public TiViterbi(){}


    /**
     * 初始状态概率函数
     *
     * @param observation             原始轨迹点
     * @param candidates              原始轨迹点对应的candidates
     * @param initialLogProbabilities 初始状态概率
     */
    private void initializeStateProbabilities(
            GPSPoint observation,
            List<CandidatePoint> candidates,
            Map<CandidatePoint, Double> initialLogProbabilities
    ) {
        if (message != null) {
            throw new IllegalArgumentException("Initial probabilities have already been set.");
        }
        Map<CandidatePoint, Double> initialMessage = new HashMap<>(initialLogProbabilities.size());
        for (CandidatePoint candidate : candidates) {
            if (!initialLogProbabilities.containsKey(candidate)) {
                throw new IllegalArgumentException("No initial probability for " + candidate);
            }
            double logProbability = initialLogProbabilities.get(candidate);
            initialMessage.put(candidate, logProbability);
        }//复制所有候选点的观测概率
        isBroken = hmmBreak(initialMessage);
        if (isBroken) {
            return;
        }
        message = initialMessage; //保存初始状态概率
        lastExtendedStates = new HashMap<>(candidates.size());
        prevObservation = observation;
        prevCandidates = new ArrayList<>(candidates.size());
        for (CandidatePoint candidate : candidates) {
            lastExtendedStates.put(candidate, new ExtendedState(candidate, null, observation));
            prevCandidates.add(candidate);
        }
        prevMatch = StreamMapMatcher.findMaxValuePoint(message);
    }

    /**
     * 如果初始化的概率有接近武无穷小的，停止初始化
     *
     * @param message 状态概率
     */
    private Boolean hmmBreak(Map<CandidatePoint, Double> message) {
        for (Double logProbability : message.values()) {
            if (!logProbability.equals(Double.NEGATIVE_INFINITY)) {
                return false;
            }
        }
        return true;
    }

    /**
     * 开始viterbi计算，向前extend
     *
     * @param observation                原始轨迹点
     * @param prevCandidates             之前的candidates
     * @param curCandidates              当前的candidates
     * @param message                    状态概率
     * @param emissionLogProbabilities   emission p
     * @param transitionLogProbabilities transition p
     * @return 向前extend后的结果
     */
    private ForwardStepResult forwardStep(
            GPSPoint observation,
            List<CandidatePoint> prevCandidates,
            List<CandidatePoint> curCandidates,
            Map<CandidatePoint, Double> message,
            Map<CandidatePoint, Double> emissionLogProbabilities,
            Map<Tuple2<CandidatePoint, CandidatePoint>, Double> transitionLogProbabilities
    ) {
        final ForwardStepResult result = new ForwardStepResult(curCandidates.size());
        assert !prevCandidates.isEmpty();
        for (CandidatePoint curState : curCandidates) {
            double maxLogProbability = Double.NEGATIVE_INFINITY;
            CandidatePoint maxPreState = null;
            for (CandidatePoint preState : prevCandidates) {
                final double logProbability = message.get(preState) + weight * transitionLogProbability(
                        preState,
                        curState,
                        transitionLogProbabilities
                );
                if (logProbability > maxLogProbability) {
                    maxLogProbability = logProbability;
                    maxPreState = preState;
                }
            }
            result.getNewMessage()
                    .put(curState, (maxLogProbability + (1 - weight) * emissionLogProbabilities.get(curState)));
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
        return result;
    }

    /**
     * 给定transition 计算对应概率
     *
     * @param preState                   之前的state
     * @param curState                   现在的state
     * @param transitionLogProbabilities 转移概率的map
     * @return 概率p
     */
    private double transitionLogProbability(
            CandidatePoint preState,
            CandidatePoint curState,
            Map<Tuple2<CandidatePoint, CandidatePoint>, Double> transitionLogProbabilities
    ) {
        Tuple2<CandidatePoint, CandidatePoint> transition = new Tuple2<>(preState, curState);
        return transitionLogProbabilities.getOrDefault(transition, Double.NEGATIVE_INFINITY);
    }

    /**
     * 计算当前状态下概率最大对应的state
     *
     * @return candidate point
     */
    private CandidatePoint mostLikelyState() {
        assert message.size() != 0;
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

    /**
     * 取回概率最大的一系列转移过程
     *
     * @return list，包含每一步转移对应的状态
     */
    private List<SequenceState> retrieveMostLikelySequence() {
        assert message.size() != 0;
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

    /**
     * 初始化启动第一步viterbi计算
     *
     * @param observation              原始轨迹点
     * @param candidates               对应的candidates
     * @param emissionLogProbabilities 每一个candidate对应的 emission p
     */
    public void startWithInitialObservation(
            GPSPoint observation,
            List<CandidatePoint> candidates,
            Map<CandidatePoint, Double> emissionLogProbabilities
    ) {
        initializeStateProbabilities(observation, candidates, emissionLogProbabilities);
    }

    /**
     * 向下一步viterbi计算， 更新当前状态的信息
     *
     * @param observation                原始轨迹点
     * @param candidates                 对应的candidates
     * @param emissionLogProbabilities   每一个candidate对应的 emission p
     * @param transitionLogProbabilities 每一个transition对应的 transition p
     */
    public void nextStep(
            GPSPoint observation,
            List<CandidatePoint> candidates,
            Map<CandidatePoint, Double> emissionLogProbabilities,
            Map<Tuple2<CandidatePoint, CandidatePoint>, Double> transitionLogProbabilities
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
                transitionLogProbabilities
        );
        isBroken = hmmBreak(forwardStepResult.getNewMessage());
        if (isBroken) {
            return;
        }
        message = forwardStepResult.getNewMessage();
        // calculate errors and adjust weight
        CandidatePoint matchedPoint = StreamMapMatcher.findMaxValuePoint(message);//找到最大概率的候选点
        double positionError = calculatePositionError(observation, matchedPoint);
        double directionError = calculateDirectionError(observation, prevObservation, matchedPoint, prevMatch);
        double speedError = calculateSpeedError(observation, prevObservation, matchedPoint, prevMatch);
        updateWeights(positionError, directionError, speedError);
        lastExtendedStates = forwardStepResult.getNewExtendedStates();
        prevObservation = observation;
        prevCandidates = new ArrayList<>(candidates);
        prevMatch = matchedPoint;
    }

    /**
     * 根据误差更新权重
     * @param positionError
     * @param directionError
     * @param speedError
     */
    public void updateWeights(double positionError, double directionError, double speedError) {
        if (slidingWindow.isFull()) {
            // 计算均值和标准差
            double positionMean = slidingWindow.getPositionMean();
            double positionStdDev = slidingWindow.getPositionStandardDeviation();

            double directionMean = slidingWindow.getDirectionMean();
            double directionStdDev = slidingWindow.getDirectionStandardDeviation();

//            double speedMean = slidingWindow.getSpeedMean();
//            double speedStdDev = slidingWindow.getSpeedStandardDeviation();

            // 计算误差差异
            double positionErrorDifference = positionError - positionMean;
            double directionErrorDifference = directionError - directionMean;
//            double speedErrorDifference = speedError - speedMean;

            // 动态调整量，仅当误差变化超过阈值（标准差）时才进行调整
            double positionAdjustment = Math.abs(positionErrorDifference) > positionStdDev ? calculateAdjustmentFactor(positionErrorDifference, positionStdDev) : - 0.02;
            double directionAdjustment = Math.abs(directionErrorDifference) > directionStdDev ? calculateAdjustmentFactor(directionErrorDifference, directionStdDev) : - 0.02;
 //           double speedAdjustment = Math.abs(speedErrorDifference) > speedStdDev ? calculateAdjustmentFactor(speedErrorDifference, speedStdDev) : 0;
            weight *= (1 + (0.2*positionAdjustment + 0.8*directionAdjustment));
            if (weight > 0.7) {
                weight = 0.7;
            }
            else if (weight < 0.3) {
                weight = 0.3;
            }
//            System.out.println("positionAdjustment:" + positionAdjustment);
//            System.out.println("directionAdjustment:" + directionAdjustment);
//            System.out.println("weight:" + weight);
        }

        // 添加误差到滑动窗口
        slidingWindow.addError(positionError, directionError, speedError);
    }

    /**
     * 根据误差计算调整因子大小
     * @param errorDifference
     * @param stdDevError
     * @return
     */
    private double calculateAdjustmentFactor(double errorDifference, double stdDevError) {
        double scalingFactor = 0.02; // 调整幅度的缩放因子，可以调整为合适的值
        return Math.min(Math.max(errorDifference / stdDevError * scalingFactor, -0.1), 0.1);
    }

    /**
     * 辅助函数，下概率最大对应的每一步的extended state 组成的list
     *
     * @return list 包含每一步对应的extended state
     */
    public List<SequenceState> computeMostLikelySequence() {
        if (message == null) {
            return new ArrayList<>();
        } else {
            return retrieveMostLikelySequence();
        }
    }
}
