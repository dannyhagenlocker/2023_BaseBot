package org.team5557.Vision;

public class VisionTarget {
    double x;
    double y;
    double z;
    TargetType targetType;

    public VisionTarget(double x, double y, double z, TargetType targetType) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.targetType = targetType;

    }

    public enum TargetType {
        APRIL_TAG,
        RETROREFLECTIVE_TAPE
    }

    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getZ() {
        return z;
    }
    public TargetType getTargetType() {
        return targetType;
    }
}
