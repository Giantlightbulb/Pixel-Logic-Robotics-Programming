package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="StateAutonomousCrater", group="Autonomous")

public class StateAutonomousCrater extends LinearOpMode {
    ArmHardwareOmni robot = new ArmHardwareOmni();

    public void runOpMode() {
        robot.init(this, hardwareMap, telemetry);
        int targetPosition;
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        // Left motion motor 2 positive, motor 4 positive
        // Forward motion motor 1 positive motor 3 positive
        //Gyro calibating
        robot.modernRoboticsI2cGyro.calibrate();

        while (!isStopRequested() && robot.modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(robot.timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        //Finalized gyro calibration.
        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();

        //Initiate Servos
        robot.mascot.setPosition(robot.basePosition);//mascot up
        robot.innerVac.setPower(0);
        robot.vacuum.setPower(0);

        // Start Button
        waitForStart();
        robot.timer.reset();

        robot.frontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Finalized startup

        //1. Lower Down and Clearance
        /*
        //Lifts the robot up to release latch tension
        runMotor(robot.verticalLift, 0.85, 0.3, "Lift Up");

        //Lowers the robot down
        runMotor(robot.verticalLift, -0.2, 1.7, "Lower Down");

        //2. Approach Sampling
        //Clears the latch
        runDriveTrain(robot.frontDrive, robot.backDrive, 0.25, 0.75, "Clearance");
        //Rotates to align
        */

        //rotateToTheta(1.0, 0, "Rotating");

        //Approaches the sampling field
        //runDriveTrain(robot.leftDrive, robot.rightDrive, 0.25, 3, "Sample Approach");
        robot.DriveForwardDistance(robot.leftDrive, robot.rightDrive, 0.35, 1727,6.0);


        robot.runDriveTrain(robot.frontDrive, robot.backDrive, 0.35, 2, "Approach Initial Sample");

        //3-4. Sample Positioning and Sampling
        boolean sampleFound = false; //Whether the sample is found
        double sampleClearance = 6; //Time to clear sampling field
        double remainingSampleClearance = 20; //Time remaining after sampling

        robot.frontDrive.setPower(-0.35);
        robot.backDrive.setPower(-0.35);
        while (opModeIsActive() && robot.timer.seconds() < sampleClearance && !robot.checkColor()) {
            telemetry.addData("Sampling", "%2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
            sampleFound = robot.checkColor();
        }
        remainingSampleClearance = sampleClearance - robot.timer.seconds();
        robot.timer.reset();
        if (sampleFound) {
            robot.runDriveTrain(robot.leftDrive, robot.rightDrive, 0.25, 1.5, "Sampling");
            robot.runDriveTrain(robot.leftDrive, robot.rightDrive, -0.25, 1.5, "Exiting Sampling");
            robot.runDriveTrain(robot.frontDrive, robot.backDrive, 0.25, remainingSampleClearance, "Clearing Sampling");
        }
        //Clear of sampling

        //5. Rotate 45 degrees
        robot.rotateTheta(2.0, -22.5, "Rotating");

        //6. Approach Field Wall
        robot.runDriveTrain(robot.frontDrive, robot.backDrive, 0.35, 3, "Approaching Field Wall");

        //7. Approach Depot
        robot.runDriveTrain(robot.leftDrive, robot.rightDrive, -0.35, 3, "Approaching Depot");

        //8. Drop Mascot
        robot.mascot.setPosition(robot.setPosition);

        //9. Approach Crater
        robot.runDriveTrain(robot.leftDrive, robot.rightDrive, 0.25, 3, "Approaching Crater");

        //10. Rotate robot
        robot.rotateTheta(2.0, -135, "Rotating");

        //11.Center Crater
        robot.runDriveTrain(robot.leftDrive, robot.rightDrive, 0.25, 3, "Centering Crater");

        //12. Extending arm over crater
        //Misnomer, runDriveTrain is used to run motor pair
        robot.runDriveTrain(robot.leftExtension, robot.rightExtension, 0.25, 1, "Extending");

        telemetry.addData("Autonomous Completed", "Please Wait");
        telemetry.update();
    }
}
