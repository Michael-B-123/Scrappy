package frc.robot;

public class Utils {
    public static boolean IsDoubleApproximately(double left, double right, double sigma){
        return left >= right - sigma && left <= right + sigma;
    }

    public static double RoundByPlaces(double value, int decimalPlaces) {
        return Math.round(value * Math.pow(10, decimalPlaces)) / Math.pow(10, decimalPlaces);
    }

    public static double deadband(double value, double deadband) {
        if (Math.abs(value) < deadband) return 0.0;
        return value;
    }
}
