package frc.Utils;

public class MathUtils {
    public static boolean isWithinTolerance(double value, double target, double tolerance) {
        return Math.abs(value - target) <= tolerance;
    }
    
}
