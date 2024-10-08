package org.urbcomp.cupid.db.algorithm.mapmatch.onlinemm;

import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.ExtendedState;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.ForwardStepResult;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.SequenceState;
import org.urbcomp.cupid.db.algorithm.mapmatch.tihmm.inner.TiViterbi;
import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.point.GPSPoint;
import scala.Tuple2;

import java.util.*;

import static org.urbcomp.cupid.db.algorithm.mapmatch.stream.StreamMapMatcher.findMaxValuePoint;

public class OnlineViterbi extends TiViterbi {
    // 维特比状态集合
    private final LinkedList<OnlineExtendedState> stateList;
    // 局部解
    private final List<SequenceState> sequenceStates;
    // 是否收敛
    public boolean isConverge;
    public int deltaT;
    // 收敛点
    private OnlineExtendedState root;
    // 上一个收敛点
    private OnlineExtendedState prevRoot;
    // 算法中断后，全局序列的起始插入位置
    public int insertPosition;

    public OnlineViterbi() {
        stateList = new LinkedList<>();
        root = null;
        prevRoot = null;
        sequenceStates = new ArrayList<>();
        deltaT = 0;
        insertPosition = 0;
    }

    public OnlineViterbi(int insertPosition) {
        stateList = new LinkedList<>();
        root = null;
        prevRoot = null;
        sequenceStates = new ArrayList<>();
        deltaT = 0;
        this.insertPosition = insertPosition;
    }

    public void nextStep(
            GPSPoint observation,
            List<CandidatePoint> candidates,
            Map<CandidatePoint, Double> emissionLogProbabilities,
            Map<Tuple2<CandidatePoint, CandidatePoint>, Double> transitionLogProbabilities,
            int time
    ) {
        if (message == null) throw new IllegalStateException("start with initial observation() must be called first.");
        if (isBroken) throw new IllegalStateException("Method must not be called after an HMM break.");

        // 调用前向计算
        ForwardStepResult forwardStepResult = forwardStep(
                observation,
                prevCandidates,
                candidates,
                message,
                emissionLogProbabilities,
                transitionLogProbabilities,
                time
        );

        isBroken = hmmBreak(forwardStepResult.getNewMessage());
        if (isBroken) {
            System.out.println("viterbi stops when executing [FORWARD STEP]");
            return;
        }

        // 更新信息
        message = forwardStepResult.getNewMessage();
        lastExtendedStates = forwardStepResult.getNewExtendedStates();
        prevCandidates = new ArrayList<>(candidates);
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
    protected ForwardStepResult forwardStep(
            GPSPoint observation,
            List<CandidatePoint> prevCandidates,
            List<CandidatePoint> curCandidates,
            Map<CandidatePoint, Double> message,
            Map<CandidatePoint, Double> emissionLogProbabilities,
            Map<Tuple2<CandidatePoint, CandidatePoint>, Double> transitionLogProbabilities,
            int time
    ) {
        assert !prevCandidates.isEmpty();

        // 当前时间下候选点个数
        int currCandiSize = curCandidates.size();
        // 前向计算的结果
        final ForwardStepResult result = new ForwardStepResult(currCandiSize);

        // 用于累积当前时间步的梯度
        double accumulatedEmissionLogProb = 0.0;
        double accumulatedTransitionLogProb = 0.0;

        // 当前时间有效的状态个数
        int validStateCount = 0;
        // 状态序列中的最后一个状态
        OnlineExtendedState lastState = null;
        // 从当前最后一个状态往前迭代的迭代器
        ListIterator<OnlineExtendedState> iterator;

        if (!stateList.isEmpty()) lastState = stateList.getLast();

        System.out.println("time: " + time);

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
                final double logProb = message.get(preState) + weightOptimizer.getTransitionWeight() * transitionLogProb;
                if (logProb > maxLogProb) {
                    maxTransitionLogProb = transitionLogProb;
                    maxLogProb = logProb;
                    maxPreState = preState;
                }
            }

            double emissionLogProb = emissionLogProbabilities.get(curState);
            result.getNewMessage().put(curState, (maxLogProb + weightOptimizer.getEmissionWeight() * emissionLogProb));

            if (maxTransitionLogProb != Double.NEGATIVE_INFINITY && emissionLogProb != Double.NEGATIVE_INFINITY) {
                accumulatedEmissionLogProb += emissionLogProb;
                accumulatedTransitionLogProb += maxTransitionLogProb;
            }

            // 如果上一个时间步的最大概率状态存在
            if (maxPreState != null) {
                OnlineExtendedState onlineExtendedState = null;

                /*
                 * time = 0: 初始化
                 * time = 1: 初始化后的第一个时间步
                 * time > 1: 第二个及之后时间步
                 * */

                if (time == 1)
                    onlineExtendedState = new OnlineExtendedState(
                            curState, lastExtendedStates.get(maxPreState), observation,
                            time, 0, currCandiSize, null);

                else if (time > 1) {
                    OnlineExtendedState parentState = lastState;
                    iterator = stateList.listIterator(stateList.indexOf(lastState));

                    assert parentState != null;

                    // 找到上一个时间步的父节点
                    while (parentState.getTime() != time - 1 || parentState.getState() != maxPreState)
                        parentState = iterator.previous();

                    // 子节点个数加1
                    parentState.setNumOfChild(parentState.getNumOfChild() + 1);
                    onlineExtendedState = new OnlineExtendedState(
                            curState, lastExtendedStates.get(maxPreState), observation,
                            time, 0, currCandiSize, parentState);
                }

                // 添加 ExtendedState
                result.getNewExtendedStates().put(curState, onlineExtendedState);
                // 添加节点
                stateList.add(onlineExtendedState);
                // 记录有效节点个数
                validStateCount++;
            }
        }

        // 处理完所有 curStates 后更新权重
        if (accumulatedTransitionLogProb != Double.NEGATIVE_INFINITY &&
                accumulatedEmissionLogProb != Double.NEGATIVE_INFINITY) {
            weightOptimizer.updateWeights(accumulatedEmissionLogProb, accumulatedTransitionLogProb);
        }
//            System.out.println("emission:" + weightOptimizer.getEmissionWeight());
//            System.out.println("transition:" + weightOptimizer.getTransitionWeight());

        // 当存在节点并且当前时间步有效节点个数大于0时
        if (!stateList.isEmpty() && validStateCount > 0) {
            lastState = stateList.getLast();
            iterator = stateList.listIterator(stateList.indexOf(lastState) + 1);

            // 更新[新添加节点]中记录的[当前时间步有效节点数信息]
//            System.out.println("valid count: " + validStateCount);
            for (int i = 0; i < validStateCount; i++) {
                OnlineExtendedState current = iterator.previous();
                current.setNumOfState(validStateCount);
            }

            // 压缩无用节点
            compress(time);
            // 删除无用节点
            freeDummyState(time);
            // 寻找收敛点并记录局部解
            if (findNewRoot()) {
                isConverge = true;
                traceback();
            }
        }

        return result;
    }

    private void compress(int currTime) {
        OnlineExtendedState lastState = stateList.getLast();
        ListIterator<OnlineExtendedState> iterator = stateList.listIterator(stateList.indexOf(lastState) + 1);

        while (iterator.hasPrevious()) {
            OnlineExtendedState current = iterator.previous();
            int time = current.getTime();
            int numOfChild = current.getNumOfChild();
            OnlineExtendedState parent = current.getParent();

            if (numOfChild == 0 && time != currTime) {
                // 删除子节点
                if (parent != null) parent.setNumOfChild(parent.getNumOfChild() - 1);
            } else {
                // 收缩父节点
                while (parent != null && parent.getNumOfChild() == 1) {
                    current.setParent(current.getParent().getParent());
                    parent = current.getParent();
                }
            }
        }
    }

    private void freeDummyState(int currTime) {
        OnlineExtendedState lastState = stateList.getLast();
        // 便于删除，索引加一
        ListIterator<OnlineExtendedState> iterator = stateList.listIterator(stateList.indexOf(lastState) + 1);
        OnlineExtendedState current;

        while (iterator.hasPrevious()) {
            current = iterator.previous();
            int time = current.getTime();
            int numOfChild = current.getNumOfChild();
            if (numOfChild <= 0 && time != currTime) iterator.remove();
        }
    }

    private boolean findNewRoot() {
        // 寻找收敛点
        if (root == null) {
            OnlineExtendedState lastState = stateList.getLast();
            ListIterator<OnlineExtendedState> iterator = stateList.listIterator(stateList.indexOf(lastState) + 1);
            OnlineExtendedState ancestor = null;
            int currCandiSize = lastState.getNumOfState();

            for (int i = 0; i < currCandiSize; i++) {
                OnlineExtendedState current = iterator.previous();
                while (current != null) {
                    OnlineExtendedState prev = current;
                    current = current.getParent();
                    if (current == null) {
                        if (ancestor == null) ancestor = prev;
                        else {
                            if (ancestor != prev) return false;
                        }
                    }
                }
            }
        }

        // 存在收敛点
        OnlineExtendedState current = stateList.getLast();
        OnlineExtendedState ancestor = null;

        // 当前收敛点和上一个收敛点的时间差
        deltaT = current.getTime();

        while (current != null) {
            if (current.getNumOfChild() >= 2) ancestor = current;
            current = current.getParent();
        }

        if (ancestor != null) {
            if (root == null) {
                root = ancestor;
                deltaT = deltaT - ancestor.getTime();
                return deltaT != 0;
            } else {
                if (ancestor != root) {
                    deltaT = deltaT - ancestor.getTime();
                    if (deltaT == 0) return false;
                    else {
                        prevRoot = root;
                        root = ancestor;
                        return true;
                    }
                }
            }
        }

        return false;
    }

    private void traceback() {
        System.out.println("Local path found!");
        List<SequenceState> interLocalPath = new ArrayList<>();
        interLocalPath.add(new SequenceState(root.getState(), root.getObservation()));

        int depth = prevRoot == null ? root.getTime() : root.getTime() - prevRoot.getTime() - 1;
        ExtendedState current = root.getBackPointer();

        for (int i = 0; i < depth; i++) {
            interLocalPath.add(new SequenceState(current.getState(), current.getObservation()));
            current = current.getBackPointer();
        }

        assert current == null || current.getState() == prevRoot.getState();

        Collections.reverse(interLocalPath);
        sequenceStates.addAll(interLocalPath);
    }

    public void tracebackLastPart(GPSPoint observation) {
        System.out.println("traceback last part");
        List<SequenceState> interLocalPath = new ArrayList<>();

        CandidatePoint maxValuePoint = findMaxValuePoint(message);
        interLocalPath.add(new SequenceState(maxValuePoint, observation));

        ExtendedState current = lastExtendedStates.get(maxValuePoint);

        while (current != root) {
            interLocalPath.add(new SequenceState(current.getState(), current.getObservation()));
            current = current.getBackPointer();
        }

        Collections.reverse(interLocalPath);
        sequenceStates.addAll(interLocalPath);

    }

    public LinkedList<OnlineExtendedState> getStateList() {
        return stateList;
    }

    public List<SequenceState> getSequenceStates() {
        return sequenceStates;
    }

    public boolean isConvergedBefore() {
        return prevRoot != null;
    }
}
