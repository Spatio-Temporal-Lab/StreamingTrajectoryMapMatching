package org.urbcomp.cupid.db.algorithm.mapmatch.online.inner.twowaylink;

import java.util.LinkedList;

public class TwoWayLinkedList<T extends TwoWayListNode<T>> extends LinkedList<T> {

    @Override
    public boolean add(T node) {
        boolean added = super.add(node);
        if (added) {
            int size = size();
            if (size > 1) {
                T prevNode = get(size - 2);
                T currentNode = get(size - 1);
                currentNode.setPrev(prevNode);
                prevNode.setNext(currentNode);
            }
        }
        return added;
    }

    @Override
    public boolean remove(Object o) {
        int index = indexOf(o);
        if (index != -1) {
            T removedNode = remove(index);
            T prevNode = removedNode.prev;
            T nextNode = removedNode.next;
            if (prevNode != null) {
                prevNode.setNext(nextNode);
            }
            if (nextNode != null) {
                nextNode.setPrev(prevNode);
            }
            return true;
        }
        return false;
    }
}
