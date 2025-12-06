package org.firstinspires.ftc.teamcode;

public class MotorPosition {

    private int[] positions;

    public MotorPosition(int[] positions) {
        this.positions = positions;
    }

    public void setPositions(int[] positions) {
        this.positions = validatePositions(positions);
    }

    public void set(int index, int value) {
        int[] holder = new int[4];
        for (int i = 0; i < 4; i++) {
            if (i == index) {
                holder[i] = value;
            } else {
                holder[i] = positions[i];
            }
        }

        positions = holder;
    }

    public void setPositions(MotorPosition positions) {
        this.positions = validatePositions(positions.toArray());
    }

    public MotorPosition getPositions() {
        return new MotorPosition(positions);
    }

    public int get(int index) {
        return positions[index];
    }

    public int[] toArray() {
        return positions;
    }

    private int[] validatePositions(int[] positions) {
        int[] holder = new int[4];

        if (positions.length > 4) {
            System.arraycopy(positions, 0, holder, 0, 4);
        } else if (positions.length < 4) {
            System.arraycopy(positions, 0, holder, 0, positions.length);

            for (int i = positions.length; i < 4; i++) {
                holder[i] = 0;
            }
        } else {
            holder = positions;
        }

        return holder;
    }
}
