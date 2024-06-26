package org.urbcomp.cupid.db.algorithm.mapmatch.amm.inner;

import org.urbcomp.cupid.db.model.point.CandidatePoint;
import org.urbcomp.cupid.db.model.roadnetwork.Path;

public class CandidateAttributes {
    /**
     * 当前候选点的概率
     */
    public double prob = 0.0;
    /**
     * 第1个候选点到该候选点的长度
     */
    public double len = Double.MAX_VALUE;
    /**
     * 第1个候选点到该候选点的累积概率
     */
    public double accProb = 0.0;
    /**
     * 该候选点对应的上一个候选点
     */
    public CandidatePoint pre = null;
    /**
     * 上一个候选点到该候选点的路径
     */
    public Path path = null;
    /**
     * 该候选点的速度
     */
    public double velocity = 5.0;

    public double getLen() {
        return len;
    }

    public double getProb() {
        return prob;
    }

    public void setProb(double prob) {
        this.prob = prob;
    }

    public void setLen(double len) {
        this.len = len;
    }

    public CandidatePoint getPre() {
        return pre;
    }

    public void setPre(CandidatePoint pre) {
        this.pre = pre;
    }

    public Path getPath() {
        return path;
    }

    public void setPath(Path path) {
        this.path = path;
    }

    public double getVelocity() {
        return velocity;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public double getAccProb() {
        return accProb;
    }

    public void setAccProb(double accProb) {
        this.accProb = accProb;
    }

    @Override
    public String toString() {
        return "CandidateAttributes{" +
                "prob=" + prob +
                ", len=" + len +
                ", accProb=" + accProb +
                ", pre=" + pre +
                ", path=" + path +
                ", velocity=" + velocity +
                '}';
    }
}
