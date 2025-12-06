package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotAutoDriveByGyro_Linear;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;

@Disabled
@TeleOp(name="Mecanum Drive")
public class MecanumDrive extends RobotAutoDriveByGyro_Linear {
    private static IMU imu = null;      // Control/Expansion Hub IMU
    private DcMotor frontLeft, backLeft, backRight, frontRight;
    private CRServo primaryArm, secondaryArm, leftSweeperServo, rightSweeperServo;
    private Servo pincherServo;

    private double  targetHeading = 0;

    private double          headingError  = 0;

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable.
    private double  turnSpeed     = 0;

    private int     leftFrontTarget    = 0;
    private int     rightFrontTarget   = 0;
    private int     leftBackTarget    = 0;
    private int     rightBackTarget   = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;

    private double adjusted_P_DRIVE_GAIN = 1;
    private double ROBOT_WIDTH = 12;
    //Put Robot Width in Inches
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.

    private List<String> instructions = new ArrayList<>();

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        try {
            primaryArm = hardwareMap.get(CRServo.class, "primary_arm");
        } catch (Exception e) {
            telemetry.addData("Error", "primary_arm (Servo) not found in config");
            primaryArm = null;
        }

        try {
            secondaryArm = hardwareMap.get(CRServo.class, "secondary_arm");
        } catch (Exception e) {
            telemetry.addData("Error", "secondary_arm (Servo) not found in config");
            secondaryArm = null;
        }

        leftSweeperServo = hardwareMap.get(CRServo.class, "sweeper_servo1");
        rightSweeperServo = hardwareMap.get(CRServo.class, "sweeper_servo2");
        pincherServo = hardwareMap.get(Servo.class, "pincher_servo");

        imu = hardwareMap.get(IMU.class, "imu");

        // Reverse the right side motors for proper direction
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motors to run without encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IMU.Parameters myIMUParameters;

        myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        new Orientation(
                                AxesReference.INTRINSIC,
                                AxesOrder.XYZ,
                                AngleUnit.DEGREES, 0,0,0,0
                        )
                )
        );
        imu.initialize(myIMUParameters);

        imu.resetYaw();

        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        pincherServo.setPosition(0.7); // Neutral position (adjust as needed)
        leftSweeperServo.setPower(0);
        rightSweeperServo.setPower(0);


        double drivePowerMax = 1;
        double armPowerMax = 0.75;

        RobotAutoDriveByGyro_Linear gyroLinear = new RobotAutoDriveByGyro_Linear();

        waitForStart();

        while (opModeIsActive()) {

            if(!gamepad1.dpad_up) {
                double forwardBackward = -gamepad1.left_stick_y * 0.7; // Scale for smoother control
                double strafe = -gamepad1.left_stick_x * 0.7; // Scale strafing
                double turn = gamepad1.right_stick_x * 0.6; // Reduce turning sensitivity

                double primaryPower = -gamepad2.left_stick_y;
                double secondaryPower = -gamepad2.right_stick_y;
                double primaryArmPower;
                double secondaryArmPower;

                if (gamepad1.a) {
                    drivePowerMax = 1;
                } else if (gamepad1.b) {
                    drivePowerMax = 0.75;
                } else if (gamepad1.x) {
                    drivePowerMax = 0.25;
                } else if (gamepad1.y) {
                    drivePowerMax = 0.5;
                }
                if (gamepad1.left_bumper && drivePowerMax < 1) {
                    drivePowerMax += 0.001;
                } else if (gamepad1.right_bumper && drivePowerMax > 0) {
                    drivePowerMax -= 0.001;
                }

                double[] drivePowers = calcDrivePowers(forwardBackward, strafe, turn, drivePowerMax);

                frontLeft.setPower(drivePowers[0]);
                frontRight.setPower(drivePowers[1]);
                backLeft.setPower(drivePowers[2]);
                backRight.setPower(drivePowers[3]);

                //Arm control
                primaryArmPower = Range.clip(primaryPower + 0.15, -0.35, armPowerMax);
                secondaryArmPower = Range.clip(secondaryPower, -1, 1);

                primaryArm.setPower(primaryArmPower);

                // Secondary arm control (up/down with right stick on gamepad2)

                secondaryArm.setPower(secondaryArmPower);

                // Pincher servo control (e.g. A button = closed, B button = open)
                if (gamepad2.a) {
                    pincherServo.setPosition(0.57);
                } else if (gamepad2.b) {
                    pincherServo.setPosition(0.7);
                }

                if (gamepad2.left_bumper) {
                    leftSweeperServo.setPower(1);
                    rightSweeperServo.setPower(-1);
                } else if (gamepad2.right_bumper) {
                    leftSweeperServo.setPower(-1);
                    rightSweeperServo.setPower(1);
                } else {
                    leftSweeperServo.setPower(0);
                    rightSweeperServo.setPower(0);
                }

                if (gamepad2.x) {
                    armPowerMax = 1;
                } else if (gamepad2.y) {
                    armPowerMax = 0.8;
                } else if (gamepad2.dpad_left) {
                    armPowerMax = 0.6;
                } else if (gamepad2.dpad_right) {
                    armPowerMax = 0.4;
                }

                if (gamepad2.dpad_up && armPowerMax < 1) {
                    armPowerMax += 0.001;
                } else if (gamepad2.dpad_down && armPowerMax > 0.15) {
                    armPowerMax -= 0.001;
                }

                if (gamepad2.back) {
                    primaryArm.setPower(1);
                    sleep(211);
                } else if (gamepad2.start) {
                    primaryArm.setPower(-1);
                    sleep(211);
                }

                //auto drive
                if (gamepad1.left_stick_button) {
                    followInstructions(instructions);
                }

                telemetry.addData("FL Power", drivePowers[0]);
                telemetry.addData("FR Power", drivePowers[1]);
                telemetry.addData("BL Power", drivePowers[2]);
                telemetry.addData("BR Power", drivePowers[3]);
                telemetry.addData("Primary Arm Power", primaryArmPower);
                telemetry.addData("Secondary Arm Power", secondaryArmPower);
                telemetry.addData("Pincher Position", pincherServo.getPosition());
                telemetry.addData("Left Sweeper Pos.", leftSweeperServo.getPower());
                telemetry.addData("Right Sweeper Pos.", rightSweeperServo.getPower());
                telemetry.addData("Drive Power Maximum", drivePowerMax);
                telemetry.addData("Arm Power Maximum", armPowerMax);
                telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            } else {
                int ticks = 0;

                if (gamepad1.a) {
                    drivePowerMax = 1.00000;
                } else if (gamepad1.b) {
                    drivePowerMax = 0.75000;
                } else if (gamepad1.x) {
                    drivePowerMax = 0.25000;
                } else if (gamepad1.y) {
                    drivePowerMax = 0.50000;
                }
                if (gamepad1.left_bumper && drivePowerMax < 1) {
                    drivePowerMax += 0.00100;
                } else if (gamepad1.right_bumper && drivePowerMax > 0) {
                    drivePowerMax -= 0.00100;
                }

                if (gamepad2.x) {
                    armPowerMax = 1.00000;
                } else if (gamepad2.y) {
                    armPowerMax = 0.80000;
                } else if (gamepad2.dpad_left) {
                    armPowerMax = 0.60000;
                } else if (gamepad2.dpad_right) {
                    armPowerMax = 0.40000;
                }

                if (gamepad2.dpad_up && armPowerMax < 1) {
                    armPowerMax += 0.00100;
                } else if (gamepad2.dpad_down && armPowerMax > 0.15) {
                    armPowerMax -= 0.00100;
                }

                if (gamepad1.left_stick_y != 0) {
                    while (gamepad1.left_stick_y != 0) {
                        ticks++;
                    }
                    instructions.add("[↕," + -gamepad1.left_stick_y * 0.7 + "," + drivePowerMax + "," + ticks + "]");
                }

                ticks = 0;

                if (gamepad1.left_stick_x != 0) {
                    while (gamepad1.left_stick_x != 0) {
                        ticks++;
                    }
                    instructions.add("[↔," + -gamepad1.left_stick_x * 0.7 + "," + drivePowerMax + "," + ticks + "]");
                }

                ticks = 0;

                if (gamepad1.right_stick_x != 0) {
                    while (gamepad1.right_stick_x < 0) {
                        ticks++;
                    }
                    instructions.add("[⇋," + gamepad1.right_stick_x * 0.6 + "," + drivePowerMax + "," + ticks + "]");
                }

                ticks = 0;

                if (gamepad2.left_stick_y != 0) {
                    while (gamepad2.left_stick_y != 0) {
                        ticks++;
                    }
                    instructions.add("[⇅," + -gamepad2.left_stick_y + "," + armPowerMax + "," + ticks + "]");
                }

                ticks = 0;

                if (gamepad2.right_stick_y != 0) {
                    while (gamepad2.right_stick_y != 0) {
                        ticks++;
                    }
                    instructions.add("[⇄," + -gamepad2.right_stick_y + "," + armPowerMax + "," + ticks + "]");
                }

                if (gamepad2.b) {
                    instructions.add("C");
                }

                if (gamepad2.a) {
                    instructions.add("O");
                }

                if (gamepad2.left_bumper) {
                    instructions.add("↺");
                }

                if (gamepad2.right_bumper) {
                    instructions.add("↻");
                }
            }
            telemetry.addData("In coding", gamepad1.dpad_up);
            telemetry.update();
        }

    }

    private void followInstructions(List<String> instructions) {
        double forwardBackward;
        double strafe;
        double turn;
        double drivePowerMax;
        double[] drivePowers;
        for (int i = 0; i < instructions.size(); i++) {
            char[] chars = instructions.get(i).toCharArray();
            if (chars[0] == '[') {
                switch (chars[1]) {
                    case '↕':
                        forwardBackward = Double.parseDouble(extractString(instructions.get(i), ',', 2));
                        drivePowerMax = Double.parseDouble(extractString(instructions.get(i), ',', 3));
                        strafe = 0;
                        turn = 0;

                        drivePowers = calcDrivePowers(forwardBackward, strafe, turn, drivePowerMax);

                        frontLeft.setPower(drivePowers[0]);
                        frontRight.setPower(drivePowers[1]);
                        backLeft.setPower(drivePowers[2]);
                        backRight.setPower(drivePowers[3]);

                        sleep(Long.parseLong(extractString(instructions.get(i), ',', 4))/100);

                        frontLeft.setPower(0);
                        frontRight.setPower(0);
                        backLeft.setPower(0);
                        backRight.setPower(0);

                        sleep(150);
                        break;
                    case '↔':
                        forwardBackward = 0;
                        drivePowerMax = Double.parseDouble(extractString(instructions.get(i), ',', 3));
                        strafe = Double.parseDouble(extractString(instructions.get(i), ',', 2));
                        turn = 0;

                        drivePowers = calcDrivePowers(forwardBackward, strafe, turn, drivePowerMax);

                        frontLeft.setPower(drivePowers[0]);
                        frontRight.setPower(drivePowers[1]);
                        backLeft.setPower(drivePowers[2]);
                        backRight.setPower(drivePowers[3]);

                        sleep(Long.parseLong(extractString(instructions.get(i), ',', 4))/100);

                        frontLeft.setPower(0);
                        frontRight.setPower(0);
                        backLeft.setPower(0);
                        backRight.setPower(0);

                        sleep(150);
                        break;
                    case '⇋':
                        forwardBackward = 0;
                        drivePowerMax = Double.parseDouble(extractString(instructions.get(i), ',', 3));
                        strafe = 0;
                        turn = Double.parseDouble(extractString(instructions.get(i), ',', 2));

                        drivePowers = calcDrivePowers(forwardBackward, strafe, turn, drivePowerMax);

                        frontLeft.setPower(drivePowers[0]);
                        frontRight.setPower(drivePowers[1]);
                        backLeft.setPower(drivePowers[2]);
                        backRight.setPower(drivePowers[3]);

                        sleep(Long.parseLong(extractString(instructions.get(i), ',', 4))/100);

                        frontLeft.setPower(0);
                        frontRight.setPower(0);
                        backLeft.setPower(0);
                        backRight.setPower(0);

                        sleep(150);
                        break;
                    case '⇅':
                        primaryArm.setPower(Range.clip(Double.parseDouble(extractString(instructions.get(i),
                                ',', 2)) + 0.15, -0.35,
                                Double.parseDouble(extractString(instructions.get(i), ',', 4))));

                        sleep(Long.parseLong(extractString(instructions.get(i), ',', 4))/100);

                        primaryArm.setPower(0);
                        break;
                    case '⇄':
                        secondaryArm.setPower(Range.clip(Double.parseDouble(extractString(instructions.get(i),
                                        ',', 2)), -1,
                                Double.parseDouble(extractString(instructions.get(i), ',', 4))));

                        sleep(Long.parseLong(extractString(instructions.get(i), ',', 4))/100);

                        secondaryArm.setPower(0);
                        break;
                }
            } else if (chars[0] == 'O' || chars[0] == 'o') {
                pincherServo.setPosition(0.57);
            } else if (chars[0] == 'C' || chars[0] == 'c') {
                pincherServo.setPosition(0.7);
            } else if (chars[0] == '↺') {
                leftSweeperServo.setPower(1.0); // spin forward
                rightSweeperServo.setPower(-1.0); // spin forward
            } else if (chars[0] == '↻') {
                leftSweeperServo.setPower(-1.0); // spin forward
                rightSweeperServo.setPower(1.0);
            }
        }
    }

    /**
     * extracts a string from between two separators, supposed to be used for extracting singular
     * elements in a string used for encrypting data.
     * @param string    the original string with information needing to be extracted.
     * @param separator the character separating sections of the string.
     * @param section   specifies which section in the string to extract, the section before the
     *                  first comma is section 1.
     */
    public String extractString(String string, char separator, int section) throws IndexOutOfBoundsException {
        if (section < 1) {
            throw new IndexOutOfBoundsException("The value of the parameter \"section\" cannot be less than 1.");
        }
        if (section > 1) {
            String temp = extractString(string, separator, section - 1) + ',';
            string = replaceString(string, temp, "");
        }
        char[] chars = string.toCharArray();
        StringBuilder output = new StringBuilder();
        for (char aChar : chars) {
            if (aChar == separator) {
                return output.toString();
            } else {
                output.append(aChar);
            }
        }
        return output.toString();
    }

    public String replaceString(String string, String replacing, String replacement) throws StringIndexOutOfBoundsException {
        char[] chars = string.toCharArray();
        char[] replaced = replacing.toCharArray();
        char[] newString = replacement.toCharArray();
        StringBuilder builder = new StringBuilder();

        if (contains(chars, replaced)) {
            for (int i = 0; i < chars.length; i++) {
                if (i == findStartingLocation(chars, replaced)) {
                    for (int j = 0; j < newString.length - 1; j++) {
                        builder.append(newString[j]);
                    }
                    i += (findEndingLocation(chars, replaced) - i);
                } else {
                    builder.append(chars[i]);
                }
            }
        } else {
            throw new StringIndexOutOfBoundsException("The string to be replaced is not contained" +
                    "within the starting string.");
        }

        return builder.toString();
    }

    /**
     * finds if a char array contains another char array where order and location matter.
     * @param chars1    the starting char array for the second char array to be found in
     * @param chars2    the char array to be found in chars1
     */
    public boolean contains(char[] chars1, char[] chars2) throws IndexOutOfBoundsException {
        if (chars1.length < chars2.length) {
            throw new IndexOutOfBoundsException("The chars to be searched for cannot be larger " +
                    "than the starting char set.");
        }

        List<Integer> charLocations = new ArrayList<>();
        for (int i = 0; i < chars1.length; i++) {
            if (chars1[i] == chars2[i]) {
                charLocations.add(i);
            }
        }

        return charLocations.get(charLocations.size() - 1) - charLocations.get(0) == chars2.length - 1;
    }

    /**
     * finds the location of where a char array starts containing another char array where order
     * and location matter.
     * @param chars1    the starting char array for the second char array to be found in
     * @param chars2    the char array to be found in chars1
     */
    public int findStartingLocation(char[] chars1, char[] chars2) throws IndexOutOfBoundsException {
        if (chars1.length < chars2.length) {
            throw new IndexOutOfBoundsException("The chars to be searched for cannot be larger " +
                    "than the starting char set.");
        }

        List<Integer> charLocations = new ArrayList<>();
        for (int i = 0; i < chars1.length; i++) {
            if (chars1[i] == chars2[i]) {
                charLocations.add(i);
            }
        }

        return charLocations.get(charLocations.size() - 1) - charLocations.get(0) ==
                chars2.length - 1 ? charLocations.get(0) : null;
    }

    /**
     * finds the location of where a char array stops containing another char array where order
     * and location matter.
     * @param chars1    the starting char array for the second char array to be found in
     * @param chars2    the char array to be found in chars1
     */
    public int findEndingLocation(char[] chars1, char[] chars2) throws IndexOutOfBoundsException {
        if (chars1.length < chars2.length) {
            throw new IndexOutOfBoundsException("The chars to be searched for cannot be larger " +
                    "than the starting char set.");
        }

        List<Integer> charLocations = new ArrayList<>();
        for (int i = 0; i < chars1.length; i++) {
            if (chars1[i] == chars2[i]) {
                charLocations.add(i);
            }
        }

        return charLocations.get(charLocations.size() - 1) - charLocations.get(0) ==
                chars2.length - 1 ? charLocations.get(charLocations.size() - 1) : null;
    }

    /**
     * calculates the powers for the motors
     * front left motor is 0 in the array, front right: 1, back left: 2, back right: 3
     * @param fb                forward backward - how far the robot moves forward and back
     * @param strafe            how far the robot moves side to side
     * @param turn              how far the robot turns
     * @param drivePowerMax     the maximum speed the robot can move at
     */
    public double[] calcDrivePowers(double fb, double strafe, double turn, double drivePowerMax) {
        double frontLeftPower = fb + strafe + turn;
        double backLeftPower = fb - strafe + turn;
        double backRightPower = fb - strafe - turn;
        double frontRightPower = fb + strafe - turn;

        double max = Math.max(1.0, Math.abs(frontLeftPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));
        max = Math.max(max, Math.abs(frontRightPower));

        if (max > 1) {
            return new double[]{frontLeftPower / max / drivePowerMax, frontRightPower / max / drivePowerMax,
                    backLeftPower / max / drivePowerMax, backRightPower / max / drivePowerMax};
        } else {
            return new double[]{frontLeftPower / drivePowerMax, frontRightPower / drivePowerMax,
                    backLeftPower / drivePowerMax, backRightPower / drivePowerMax};
        }
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction

            moveRobot(0, turnSpeed);
            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobotStraight(0,0, getHeading());
    }

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
     *  This function is useful for giving the robot a moment to stabilize its heading between movements.
     *
     * @param maxSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param distance   Length of time (in seconds) to hold the specified heading.
     */
    public void moveRobotStraight(double maxSpeed, double distance, double heading) {
        // Determine new target position, and pass to motor controller
        int moveCounts = (int) (distance * COUNTS_PER_INCH);
        leftFrontTarget = frontLeft.getCurrentPosition() + moveCounts;
        rightFrontTarget = frontRight.getCurrentPosition() + moveCounts;
        leftBackTarget = backLeft.getCurrentPosition() + moveCounts;
        rightBackTarget = backRight.getCurrentPosition() + moveCounts;

        // Set Target FIRST, then turn on RUN_TO_POSITION
        frontRight.setTargetPosition(leftFrontTarget);
        frontLeft.setTargetPosition(rightFrontTarget);
        backRight.setTargetPosition(leftBackTarget);
        backLeft.setTargetPosition(rightBackTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        maxSpeed = Math.abs(maxSpeed);
        frontLeft.setTargetPosition(leftFrontTarget);
        frontRight.setTargetPosition(rightFrontTarget);
        backLeft.setTargetPosition(leftBackTarget);
        backRight.setTargetPosition(rightFrontTarget);

        // keep looping while we are still active, and BOTH motors are running.
        while (opModeIsActive() &&
                (frontRight.isBusy() && frontLeft.isBusy())) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            if (turnSpeed < 1) {
                frontLeft.setPower(turnSpeed);
                backLeft.setPower(turnSpeed);
                frontRight.setPower(-turnSpeed);
                frontRight.setPower(-turnSpeed);

            }
            adjusted_P_DRIVE_GAIN = (Math.tan(turnSpeed) * ROBOT_WIDTH) / moveCounts;
            frontRight.setPower(1 / turnSpeed);
            backRight.setPower(1 / turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(true);
        }
    }
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobotStraight(0, 0, 0);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;

    }

    // **********  LOW Level driving functions.  ********************
    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1, 1);
    }
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos FL:BL:FR:BR",  "%7d:%7d:%7d:%7d",      leftFrontTarget,  leftBackTarget, rightFrontTarget, rightBackTarget);
            telemetry.addData("Actual Pos FL:BL:FR:BR",  "%7d:%7d:%7d:%7d",      frontLeft.getCurrentPosition(),  backLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backRight.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);

        telemetry.update();
    }

}


