package org.firstinspires.ftc.teamcode;

public class MathExtended {
    public int factorial(int number) {
        return number == 1 ? 1 : number * factorial(number - 1);
    }
}
