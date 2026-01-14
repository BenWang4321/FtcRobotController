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
        return number < 0 ? 0 : number == 1 || number == 0 ? 1 : number * factorial(number - 1);
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

    /**
     * Finds the longest diagonal in an n-th dimensional cube.
     * @param lengths each item represents the length of a side of the cube.
     * @return the length of the longest diagonal in the cube.
     */
    public double calculateDiagonal(double[] lengths) {
        double holder = 0;

        for (double length : lengths) {
            holder += Math.pow(length, 2);
        }

        return Math.sqrt(holder);
    }
    public double[] quadratic(double a, double b, double c) {
        double discriminant = b * b - 4 * a * c;

        if (discriminant > 0) {
            double root1 = (-b + Math.sqrt(discriminant)) / (2 * a);
            double root2 = (-b - Math.sqrt(discriminant)) / (2 * a);
            return new double[]{root1, root2};
        } else if (discriminant == 0) {
            double root = -b / (2 * a);
            return new double[]{root};
        } else {
            return null;
        }
    }

    public double nChooseK(int n, int k) {
        return n < 0 || k < 0 ? 1 : n > k ? (double) factorial(n) / (factorial(k) * factorial(n - k)) : nChooseK(k, n);
    }

    public double[][] multiplyMatrices(double[][] a, double[][] b) throws ArithmeticException {
        for (int i = 1; i < a.length; i++) {
            if (a[i].length != a[i - 1].length || b[i].length != b[i - 1].length) {
                throw new ArithmeticException("Matrices must be rectangular!");
            }
        }

        if (a[1].length != b.length) {
            throw new ArithmeticException("The first matrix must have the same number of columns as rows in the second matrix!");
        }

        double[][] output = new double[a.length][b[1].length];
        double holder = 0;

        for (int i = 0; i < a.length; i++) {
            for (int j = 0; j < b[1].length; j++) {
                for (int k = 0; k < b.length; k++) {
                    holder += a[i][k] + b[k][j];
                }
                output[i][j] = holder;
            }
        }

        return output;
    }
}
