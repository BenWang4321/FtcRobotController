package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="supra autonomous")

public class SupraAutonomous extends LinearOpMode {

    DcMotorEx frontLeft, backLeft, backRight, frontRight = null;
    ModernRoboticsI2cRangeSensor rangeSensor;
    double cameraDist;

    double gravity = 9.807;
    double ballHeightAtLauncher = 2;
    double ballWeight = 2;
    double launchAngle = 45;
    double requiredLaunchHeight = 1;

    DcMotor ejector1, ejector2 = null;
    IMU imu;

    YawPitchRollAngles robotOrientation;

    IMU.Parameters myIMUparameters;
    Orientation myRobotOrientation;

    double distancePerRotation = 2.35619449019;
    double speed;
    int preciseVelocity = 1000;
    int maxVelocity = 3000;
    int[] velocity = {0,0,0,0};
    double[] currentPosition = {0,0,0,0,0,0};
    double robotLength = 5; //cm
    double robotWidth = 5; //cm
    List<double[]> wheelDistFromRobot = new ArrayList<>();
    double[] robotDistFromCamera = {1,1,1};
    List<double[]> currentWheelPosition = new ArrayList<>();
    List<int[]> supraCheckpoints = new ArrayList<>();
    int currentCheckPoint = 0;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(myIMUparameters);
        supraCheckpoints.add(new int[]{0, 0, 0});

        /*forward*/         addCheckpoint(10000, 10000, 10000, 10000);
        /*backward*/        addCheckpoint(-10000, -10000, -10000, -10000);
        /*strafe left*/     addCheckpoint(-10000, 10000, 10000, -10000);
        /*strafe right*/    addCheckpoint(10000, -10000, -10000, 10000);
        /*forward left*/    addCheckpoint(10000, 0, 0, 10000);
        /*forward right*/   addCheckpoint(0, 10000, 10000, 0);
        /*backward left*/   addCheckpoint(-10000, 0, 0, -10000);
        /*backward right*/  addCheckpoint(0, -10000, -10000, 0);
        /*turn left*/       addCheckpoint(-10000, 10000, -10000, 10000);
        /*turn right*/      addCheckpoint(10000, -10000, -10000, 10000);


        char[] ballOrder = new char[3];
        boolean onBlue = true;

        wheelDistFromRobot.add(new double[]{1,1,1});
        wheelDistFromRobot.add(new double[]{1,1,1});
        wheelDistFromRobot.add(new double[]{1,1,1});
        wheelDistFromRobot.add(new double[]{1,1,1});
        currentWheelPosition.add(wheelDistFromRobot.get(0));
        currentWheelPosition.add(wheelDistFromRobot.get(1));
        currentWheelPosition.add(wheelDistFromRobot.get(2));
        currentWheelPosition.add(wheelDistFromRobot.get(3));

        initAprilTag();

        waitForStart();


        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMotorEnable();
        frontRight.setMotorEnable();
        backLeft.setMotorEnable();
        backRight.setMotorEnable();

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()) {

            myRobotOrientation = imu.getRobotOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.DEGREES
            );

// Then read or display the desired values (Java type float):
            float X_axis = myRobotOrientation.firstAngle;
            float Y_axis = myRobotOrientation.secondAngle;
            float Z_axis = myRobotOrientation.thirdAngle;
            for (AprilTagDetection detection : aprilTag.getDetections()) {
                switch (detection.id) {
                    case 21:
                        ballOrder[0] = 'G';
                        ballOrder[1] = 'P';
                        ballOrder[2] = 'P';
                        break;
                    case 22:
                        ballOrder[0] = 'P';
                        ballOrder[1] = 'G';
                        ballOrder[2] = 'P';
                        break;
                    case 23:
                        ballOrder[0] = 'P';
                        ballOrder[1] = 'P';
                        ballOrder[2] = 'G';
                        break;
                    default:
                        break;
                }
            }

            initiateCheckpoint();
            speed = frontLeft.getVelocity() * distancePerRotation;
            // Share the CPU

             //detects if the motor has arrived to position
            if (frontRight.getPower() < 0.1 && frontLeft.getPower() < 0.1 &&
                    backRight.getPower() < 0.1 && backLeft.getPower() < 0.1) {
                currentCheckPoint += 1;
            }

            telemetryAprilTag();
            telemetry.addData("speed", speed);
            telemetry.update();

        }

        // Save more CPU resources when camera is no longer needed.
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    @SuppressLint("DefaultLocale")
    private ArrayList<AprilTagDetection> telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (cm, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }


        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

        return aprilTag.getDetections();
    }   // end method telemetryAprilTag()

    /**
     * Adds a checkpoint for the motor on the robot.
     * Does all the calculations of the positions for you,
     * so you can just add how much you want the motor to change,
     * not where you want the motor to change to.
     * @param frontLeft     sets the next position for the front left motor
     * @param frontRight    sets the next position for the front right motor
     * @param backLeft      sets the next position for the back left motor
     * @param backRight     sets the next position for the back right motor
     */
    private void addCheckpoint(int frontLeft, int frontRight, int backLeft, int backRight) {
        int[] currentCheckpoint = supraCheckpoints.get(supraCheckpoints.size() - 1);
        supraCheckpoints.add(new int[]{frontLeft + currentCheckpoint[0], frontRight + currentCheckpoint[1],
                backLeft + currentCheckpoint[2], backRight + currentCheckpoint[3]});
    }

    private double calculateLaunchPower(double distFromGoal) {
        MathExtended math = new MathExtended();
        return Math.sqrt((gravity * distFromGoal) / 2 * (Math.pow(Math.cos(launchAngle), 2) * (ballHeightAtLauncher + distFromGoal * Math.tan(launchAngle) + 1))); //just a placeholder
    }
    private double[] calculateRobotPosition() {
        ArrayList<AprilTagDetection> aprilTagDistance = telemetryAprilTag();
        double distX = 0;
        double distY = 0;
        double distZ = 0;
        for (AprilTagDetection detection : aprilTagDistance) {
            if (detection.metadata != null) {
                distX = detection.ftcPose.x;
                distY = detection.ftcPose.y;
                distZ = detection.ftcPose.z;
            }
        }

        currentPosition = new double[]{distX + robotDistFromCamera[0], distY + robotDistFromCamera[1], distZ + robotDistFromCamera[2]};
        return new double[]{};
    }

    private void moveMotors(int frontRightAmount, int frontLeftAmount, int backRightAmount, int backLeftAmount) {
        frontRight.setTargetPosition(frontRightAmount);
        frontLeft.setTargetPosition(frontLeftAmount);
        backRight.setTargetPosition(backRightAmount);
        backLeft.setTargetPosition(backLeftAmount);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setVelocity(maxVelocity);
        frontLeft.setVelocity(maxVelocity);
        backRight.setVelocity(maxVelocity);
        backLeft.setVelocity(maxVelocity);
    }
    private void turnRobot(double angle) {
        moveMotors((int) (robotLength + robotWidth / (2 * angle / 360)), (int) (robotLength + robotWidth / (2 * angle / 360)), (int) (robotLength - robotWidth / (2 * angle / 360)), (int) (robotLength - robotWidth / (2 * angle / 360)));
    }
    private void convertPositionToMovement() {
        moveMotors(((int)Math.round(Math.sqrt(Math.pow(supraCheckpoints.get(currentCheckPoint)[0] - currentPosition[0], 2) + Math.pow(supraCheckpoints.get(currentCheckPoint)[0] - currentPosition[0], 2)))), ((int) Math.round(Math.sqrt(Math.pow(supraCheckpoints.get(currentCheckPoint)[0] - currentPosition[0], 2) + Math.pow(supraCheckpoints.get(currentCheckPoint)[0] - currentPosition[0], 2)))), ((int) Math.round(Math.sqrt(Math.pow(supraCheckpoints.get(currentCheckPoint)[0] - currentPosition[0], 2) + Math.pow(supraCheckpoints.get(currentCheckPoint)[0] - currentPosition[0], 2)))), ((int)Math.round(Math.sqrt(Math.pow(supraCheckpoints.get(currentCheckPoint)[0] - currentPosition[0], 2) + Math.pow(supraCheckpoints.get(currentCheckPoint)[0] - currentPosition[0], 2)))));
    }

    private void calcVelocity() {
        velocity[0] = (int) frontRight.getVelocity();
        velocity[1] = (int) frontLeft.getVelocity();
        velocity[2] = (int) backRight.getVelocity();
        velocity[3] = (int) backLeft.getVelocity();
    }
    private boolean preciseVelocity(double firstValue, double secondValue, double thirdValue) {
        boolean isntAtPos = true;
        for (int i = 0; i<=2; i++) {
            if (firstValue <= 0.1 * secondValue) {
                frontRight.setVelocity(preciseVelocity);
                frontLeft.setVelocity(preciseVelocity);
                backRight.setVelocity(preciseVelocity);
                backLeft.setVelocity(preciseVelocity);
            }
            if (firstValue <= 0.02 * secondValue) {
                isntAtPos = false;
            }
        }
        return isntAtPos;
    }
    private void initiateCheckpoint() {
        convertPositionToMovement();
        boolean isntAtPos = true;
        double angle = (Math.acos((supraCheckpoints.get(currentCheckPoint)[0] - currentPosition[0]) / (supraCheckpoints.get(currentCheckPoint)[1] - currentPosition[1])) - currentPosition[3]);
        turnRobot(angle - currentPosition[3]);
        while (isntAtPos) {
            calcVelocity();
            myRobotOrientation = imu.getRobotOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.XYZ,
                    AngleUnit.DEGREES
            );
            isntAtPos = preciseVelocity(angle - myRobotOrientation.firstAngle, 0.1, 0.02);
            }
        isntAtPos = true;
        convertPositionToMovement();
        while (isntAtPos) {
            calcVelocity();
            double[] position = calculateRobotPosition();
            for (int i = 0; i<=2; i++) {
                if (supraCheckpoints.get(currentCheckPoint)[0] - position[0] <= 0.02 * velocity[i]) {
                    isntAtPos = false;
                }
            }
        }
        currentCheckPoint++;
    }
}