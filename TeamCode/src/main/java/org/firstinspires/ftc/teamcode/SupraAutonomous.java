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
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

@Autonomous(name="supra autonomous")

public class SupraAutonomous extends LinearOpMode {
    //arena boundaries are 367.5, 367.5
    DcMotorEx frontLeft, backLeft, backRight, frontRight = null;
    ModernRoboticsI2cRangeSensor distanceSensor = null;
    DcMotorEx[] motorGroup;
    ModernRoboticsI2cRangeSensor rangeSensor;
    double cameraDist;
    DcMotor ejector1, ejector2 = null;
    IMU imu;
    IMU.Parameters myIMUparameters;
    int positionTolerance = 10;
    int precisionPositionTolerance = 50;
    double DISTANCE_PER_ROTATION = 23.5619449019;
    double FRICTION_COEFFICIENT = 0.638297872; //friction force / effective force applied
    double ROBOT_LENGTH = 38; //In Centimeters
    double speed;
    double[] currentPosition = new double[4];
    int[] motorRotations = new int[4];
    double currentRotation = 90;
    //front right, front left, back right, back left
    List<double[]> motorDistancesFromRobot = new ArrayList<>(List.of(new double[]{0,129.8, 45}, new double[]{0,129.8, 25, -45}, new double[]{0,129.8, -25, 45}, new double[]{0,129.8, 25, 45}));

    double cameraOrientationfromRobot = 0;
    int currentCheckpoint = 0;
    int maxVelocity = 3000;
    int currentMaxVelocity = 0;
    int preciseVelocity = 1000;

    HashMap<Integer, double[]> supraAprilTag = new HashMap<>();
    //24.225 from closest point y value
    public enum runMode {

    }
    List<double[]> supraCheckpoints = new ArrayList<>(List.of(new double[]{121.9,0,90}));
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;



    public void runOpMode() {

        supraAprilTag.put(20, new double[]{34.461,17.961});
        supraAprilTag.put(24, new double[]{333.039, 349.539});

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        ejector1 = hardwareMap.get(DcMotorEx.class, "ejector1");
        ejector2 = hardwareMap.get(DcMotorEx.class, "ejector2");
        imu = hardwareMap.get(IMU.class, "imu");
        //distanceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultrasonicSensor");
        motorGroup = new DcMotorEx[]{frontRight, frontLeft, backRight, backLeft};

        frontLeft.setTargetPositionTolerance(positionTolerance);
        backLeft.setTargetPositionTolerance(positionTolerance);
        frontRight.setTargetPositionTolerance(positionTolerance);
        backRight.setTargetPositionTolerance(positionTolerance);
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(myIMUparameters);

        initAprilTag();

        waitForStart();

        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        while (opModeIsActive()) {
            if (currentCheckpoint + 1 > supraCheckpoints.size()) {
                executeCheckpoint();
            }
            telemetryAprilTag();
            telemetry.addData("speed", speed);
            telemetry.update();
        }
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
        VisionPortal visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                Orientation rot = Orientation.getOrientation(detection.rawPose.R, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", rot.firstAngle, rot.secondAngle, rot.thirdAngle));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f (pov)", detection.robotPose.getOrientation().getYaw(),
                        detection.robotPose.getOrientation().getPitch(), detection.robotPose.getOrientation().getRoll()));

                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range,
                        detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Rotation %b", (90 - currentRotation) == 0));

                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }


        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
        telemetry.addLine(String.format("X %6.0f, Y %6.0f", currentPosition[0], currentPosition[1]));
        telemetry.addLine(String.format("Rotation %b", (90 - currentRotation) == 0));
        telemetry.addLine(String.format("Checkpoint %d, %d", currentCheckpoint, supraCheckpoints.size()));



    }   // end method telemetryAprilTag()

    /**
     * Adds a checkpoint for the motor on the robot.
     * Does all the calculations of the positions for you,
     * so you can just add how much you want the motor to change,
     * not where you want the motor to change to.
     */

    private void executeCheckpoint() {
        double[] objective = supraCheckpoints.get(currentCheckpoint);
        boolean IsAt = false;
        boolean interrupted = false;
        double orientationObjective = Math.asin(objective[1] - currentPosition[1] / objective[0] - currentPosition[0]);
        if (orientationObjective > 180) {
            orientationObjective -= 180;
        }
        double distance = Math.hypot(objective[0] - currentPosition[0], objective[1] - currentPosition[1]);
        double[] strafe = calculateStrafing(objective[0], objective[1], 0,0);
        double frontRightSpeed = (strafe[0] + strafe[1]) / 2;
        double frontLeftSpeed = (-strafe[0] + strafe[1]) / 2;
        double backRightSpeed = (strafe[0] + strafe[1]) / 2;
        double backLeftSpeed = (-strafe[0] + strafe[1]) / 2;
        double max = Math.max(frontRightSpeed, Math.max(frontLeftSpeed, Math.max(backLeftSpeed, Math.max(backRightSpeed, 1))));
        frontRightSpeed /= max;
        frontLeftSpeed /= max;
        backRightSpeed /= max;
        backLeftSpeed /= max;
        double strafePower = Math.max(frontRightSpeed, Math.max(frontLeftSpeed, Math.max(backRightSpeed, backLeftSpeed)));

        //The speed required for strafing is the distance moved multiplied by the max speed the motor can move
        double strafeSpeed = strafePower * distance;
        //The speed required for driving straight is the turn speed plus the distance. The turn speed  is the circumferance distance of the robot / 2. The circumferance is Pi * 2 * radii. The radii is a wheels distance from the center of the robot. Rotating speed multiplied to compensate for acceleration.
        double straightSpeed = (motorDistancesFromRobot.get(0)[1] * Math.PI * orientationObjective) * 1.5 + distance;
        //Compares the required speed for strafing and Driving straight
        currentMaxVelocity = maxVelocity;
        if (strafeSpeed > straightSpeed) {
            //Calculate if the robot has arrived at the objective through matching the robots position with the objective. Will also stop mid process if called upon. Will slow down the robot if it nears the target.
            while (!IsAt || !interrupted) {
                frontRight.setVelocity(currentMaxVelocity * frontRightSpeed);
                frontLeft.setVelocity(currentMaxVelocity * frontLeftSpeed);
                backRight.setVelocity(currentMaxVelocity * backRightSpeed);
                backLeft.setVelocity(currentMaxVelocity * backLeftSpeed);
                calculateRobotPosition();
                if (Math.abs(currentPosition[0] - objective[0]) < precisionPositionTolerance || Math.abs(currentPosition[1] - objective[1]) < precisionPositionTolerance) {
                    currentMaxVelocity = preciseVelocity;
                    setMaxVelocity(preciseVelocity);
                } else if (Math.abs(objective[0] - currentPosition[0]) < positionTolerance || Math.abs(objective[1] - currentPosition[1]) < positionTolerance) {
                    IsAt = true;
                } else {
                    setMaxVelocity(maxVelocity);
                }
            }
        } else {
            //rotate robot
            //direction false is left, true is right
            boolean direction = false;
            if (orientationObjective - currentRotation < 0) {
                direction = true;
            }
            if (Math.abs(orientationObjective - currentRotation) < 0 && direction == true) {
                frontRight.setVelocity(currentMaxVelocity);
                frontLeft.setVelocity(-currentMaxVelocity);
                backRight.setVelocity(currentMaxVelocity);
                backLeft.setVelocity(-currentMaxVelocity);
            } else {
                frontRight.setVelocity(-currentMaxVelocity);
                frontLeft.setVelocity(currentMaxVelocity);
                backRight.setVelocity(-currentMaxVelocity);
                backLeft.setVelocity(currentMaxVelocity);
            }
            while (!IsAt || !interrupted) {
                calculateRobotRotation();
                if (Math.abs(currentRotation - orientationObjective) < precisionPositionTolerance) {
                    setMaxVelocity(preciseVelocity);
                } else if (Math.abs(currentRotation - orientationObjective) < positionTolerance) {
                    IsAt = true;
                } else {
                    setMaxVelocity(maxVelocity);
                }
            }
            while (!IsAt || !interrupted) {
                calculateRobotRotation();
                calculateRobotRotation();
                if (Math.abs(currentRotation - orientationObjective) / 360 - ((int) Math.floor(Math.abs(currentRotation - orientationObjective))) < positionTolerance / 360) {
                    supraCheckpoints.add(currentCheckpoint, new double[] {0,0, orientationObjective + (currentRotation - orientationObjective)});
                    interrupted = true;
                } else {
                    for (DcMotorEx motor:motorGroup) {
                        if (motor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                            motor.setDirection(DcMotorSimple.Direction.REVERSE);
                        } else {
                            motor.setDirection(DcMotorSimple.Direction.FORWARD);
                        }
                    }
                }
                if (Math.abs(objective[0] - currentPosition[0]) < precisionPositionTolerance || Math.abs(objective[1] - currentPosition[1]) < precisionPositionTolerance) {
                    setMaxVelocity(preciseVelocity);
                } else if (Math.abs(objective[0] - currentPosition[0]) < positionTolerance || Math.abs(objective[1] - currentPosition[1]) < positionTolerance) {
                    IsAt = true;
                } else {
                    setMaxVelocity(maxVelocity);
                }
            }

        }
        //Check if orientation is correct for driving straight
        if (orientationObjective - currentRotation < preciseVelocity) {
            boolean direction = false;
            if (orientationObjective - currentRotation < 0) {
                direction = true;
            }
            if (Math.abs(orientationObjective - currentRotation) < 0 && direction == true) {
                frontRight.setVelocity(currentMaxVelocity);
                frontLeft.setVelocity(-currentMaxVelocity);
                backRight.setVelocity(currentMaxVelocity);
                backLeft.setVelocity(-currentMaxVelocity);
            } else {
                frontRight.setVelocity(-currentMaxVelocity);
                frontLeft.setVelocity(currentMaxVelocity);
                backRight.setVelocity(-currentMaxVelocity);
                backLeft.setVelocity(currentMaxVelocity);
            }
            while (!IsAt || !interrupted) {
                calculateRobotRotation();
                if (Math.abs(currentRotation - orientationObjective) < precisionPositionTolerance) {
                    setMaxVelocity(preciseVelocity);
                } else if (Math.abs(currentRotation - orientationObjective) < positionTolerance) {
                    IsAt = true;
                } else {
                    setMaxVelocity(maxVelocity);
                }
            }
        }
        stopAllMotors();
        if (IsAt == true) {
            currentCheckpoint += 1;
        }
    }
    private void runAllMotors() {
        for (DcMotorEx motor : motorGroup) {
            motor.setVelocity(currentMaxVelocity);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    private void stopAllMotors() {
        for (DcMotorEx motor : motorGroup) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    private void driveToAprilTag(AprilTagDetection tag, double abortDistance) {
        if (tag.metadata != null) {
            double distance = abortDistance - tag.ftcPose.range;
        }
    }

    private void turnToAprilTag(AprilTagDetection tag, double abortAngle) {
        if (tag.metadata != null) {
            double angle = abortAngle - tag.ftcPose.bearing;
        }
    }
    private void calculateRobotRotation() {
        Orientation robotOrientation;
        robotOrientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
        );
        currentRotation = robotOrientation.firstAngle;
    }
    private boolean calculateRobotPosition() {
        int[] newPositions = {frontRight.getCurrentPosition(), frontLeft.getCurrentPosition(), backRight.getCurrentPosition(), backLeft.getCurrentPosition()};
        int collectiveChange = newPositions[0] - motorRotations[0];
        int excessChange = 0;
        for (int i = 0; i<4; i++) {
            if (newPositions[i] - motorRotations[i] < collectiveChange) {
                collectiveChange = newPositions[i] - motorRotations[i];
            } else if (newPositions[i] - motorRotations[i] - collectiveChange > excessChange) {
                excessChange = newPositions[i] - motorRotations[i] - collectiveChange;
            }
        }
        motorRotations = newPositions;

        /*currentPosition[0] += Math.cos((double) (Math.round(currentRotation/))) * collectiveChange;
        currentPosition[2] += Math.sin((double) (Math.round(currentRotation))) * collectiveChange;
        currentRotation += Math.tan( motorDistancesFromRobot.get(0)[1] / motorDistancesFromRobot.get(0)[0]);
        motorRotations = positions;
        calculateRobotRotation();
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            currentPosition[0] = supraAprilTag.get(detection.id)[0] - detection.robotPose.getPosition().x;
            currentPosition[1] = supraAprilTag.get(detection.id)[1] - detection.robotPose.getPosition().y;
            currentRotation[0] = 0 - detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
        }

        //determines if current position doesn't align with current angle to determine if robot actual position doesnt align with assumed position
        if (Math.sin(currentRotation[0]) * distanceSensor.getDistance(DistanceUnit.CM) - 367.5 + currentPosition[0] > precisionPositionTolerance * 2 && Math.cos(currentRotation[0]) * distanceSensor.getDistance(DistanceUnit.CM) - 367.5 + currentPosition[1] > precisionPositionTolerance * 2) {

        }

         */
        boolean interrupted = false;
        AngularVelocity angleChange = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        if (Math.sin(DISTANCE_PER_ROTATION * (excessChange)) < angleChange.zRotationRate && distanceSensor.getDistance(DistanceUnit.CM) < 20) {
            interrupted = true;
        }
        return interrupted;
    }
    public void velocityLeavingLauncher(){
        double Distance_toGoal = rangeSensor.getDistance(DistanceUnit.CM);
        double V_leavingToDistance = Math.sqrt(((Distance_toGoal/100+0.1524)*9.81));
        double power = V_leavingToDistance / 3 * Math.PI;
        ejector1.setPower(power);
        ejector2.setPower(power);
    }
    //multiplies the distance by the robot's angle to calculate the slope distance for strafing. Calculates the x (forward) power each wheel must have. The y (right) power depends on each wheel as they have different angles
    public double[] calculateStrafing(double objective1, double objective2, double wheelAngleOffsetX, double wheelAngleOffsetY) {
        double[] objective = {(objective1 - (objective1 - currentPosition[0]) * Math.cos(currentRotation + wheelAngleOffsetX)) / (objective2-currentPosition[1] * Math.cos(currentRotation + wheelAngleOffsetX)), (objective2 - (objective1 - currentPosition[0]) * Math.cos(currentRotation + wheelAngleOffsetY)) / (objective2-currentPosition[1] * Math.cos(currentRotation + wheelAngleOffsetY))};
        return objective;
    }

    public void setMaxVelocity(int minVelocity) {
        currentMaxVelocity = (int) Math.min(minVelocity, Math.max(backRight.getVelocity(), Math.max(backLeft.getVelocity(), Math.max(frontLeft.getVelocity(), frontRight.getVelocity()))));
    }
}