package org.firstinspires.ftc.teamcode;

public class MathExtended {
    /**
     * The factorial function solves for a repeated sequence of multiplication,
     * where each term is 1 less than the one before it,
     * so x! = 1 * 2 * 3 ... up to x
     * @param number the value of x
     * @return the factorial of x
     */
    public int factorial(int number) {
        return number == 1 ? 1 : number * factorial(number - 1);
    }

    /**
     * Solves for the exponentiation of the base and the exponent
     * @param base      the base for the exponentiation. ex. the base in a^b would be a
     * @param exponent  the exponent for the exponentiation. ex. the base in a^b would be b
     * @return base ^ exponent
     */
    public double exp(double base, double exponent) {
        return Math.exp(exponent * Math.log(base));
    }

    /**
     * Solves for the log of a number with a custom base (the log function is just finding which
     * exponent with a certain base makes the number, so log10(100) would be 2 as 10^2 = 100)
     * @param number the number to calculate the logarithm of
     * @param base   the base of the log function
     * @return the value of log_base(number)
     */
    public double logN(double number, double base) {
        return Math.log(number)/Math.log(base);
    }
}
