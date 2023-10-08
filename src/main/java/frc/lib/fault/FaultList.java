
package frc.lib.fault;

import java.util.ArrayList;

public class FaultList extends ArrayList<Fault> {

    public int getCurrentFaultCount() {
        int count = 0;
        for (Fault fault : this) {
            if (fault.get()) {
                count += fault.getCount();
            }
        }
        return count;
    }

    public int getStickyFaultCount() {
        int count = 0;
        for (Fault fault : this) {
            if (fault.getSticky()) {
                count += 1;
            }
        }
        return count;
    }

    public boolean hasFaults() {
        return getCurrentFaultCount() > 0;
    }

    public boolean hasStickyFaults() {
        return getStickyFaultCount() > 0;
    }

    /**
     * plus: merge two faultlists into one and return it
     * @param other FaultList to merge with
     * @return new FaultList with both FaultLists
     */
    public FaultList plus(FaultList other) {
        FaultList newList = new FaultList();
        newList.addAll(this);
        newList.addAll(other);
        return newList;
    }

    /**
     * plus: merge another faultlist into this one
     * @param other FaultList to merge
     */
    public void merge(FaultList other) {
        this.addAll(other);
    }

    public boolean[] currentToBooleanArray() {
        Fault[] array = new Fault[this.size()];
        array = this.toArray(array);
        boolean[] output = new boolean[array.length];

        for (int i = 0; i < array.length; i++) {
            output[i] = !array[i].get(); //invert to make shuffleboard show red when there is a fault
        }

        return output;
    }

    public boolean[] stickyToBooleanArray() {
        Fault[] array = new Fault[this.size()];
        array = this.toArray(array);
        boolean[] output = new boolean[array.length];

        for (int i = 0; i < array.length; i++) {
            output[i] = !array[i].getSticky(); //invert to make shuffleboard show red when there is a fault
        }

        return output;
    }

    public String getCurrentFaults() {
        String out = "";
        for (Fault fault : this) {
            if (fault.get()) {
                out += fault.getName();
            }
        }
        return out;
    }

    public String getStickyFaults() {
        String out = "";
        for (Fault fault : this) {
            if (fault.getSticky()) {
                out += fault.getName();
            }
        }
        return out;
    }
}
