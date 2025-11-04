package org.firstinspires.ftc.teamcode;

public class MathExtended {
    public int factorial(int number) {
        return number == 1 ? 1 : number * factorial(number - 1);
    }

    public double exp(double base, double exponent) {
        return Math.exp(exponent * Math.log(base));
    }
}
