package org.urbcomp.cupid.db.algorithm.mapmatch.online.inner.twowaylink;

public class TwoWayListNode<T> {
    public T prev;
    public T next;

    public TwoWayListNode() {
        this.prev = null;
        this.next = null;
    }

    public void setPrev(T prev) {
        this.prev = prev;
    }

    public void setNext(T next) {
        this.next = next;
    }

}
