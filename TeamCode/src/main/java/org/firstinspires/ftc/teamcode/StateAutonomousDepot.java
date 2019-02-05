package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="DepotTest", group="Autonomous")

public class StateAutonomousDepot extends LinearOpMode {
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
        //Lifts the robot up to release latch tension
        robot.runMotor(robot.verticalLift, 0.85, 0.3, "Lift Up");

        //Lowers the robot down
        robot.runMotor(robot.verticalLift, -0.2, 1.7, "Lower Down");

        //2. Approach Sampling
        //Clears the latch
        robot.runDriveTrain(robot.frontDrive, robot.backDrive, 0.25, 0.75, "Clearance");
        //Rotates to align
        robot.rotateTheta(1.0, 0, "Rotating");
        //Approaches the sampling field
        robot.runDriveTrain(robot.leftDrive, robot.rightDrive, 0.25, 3, "Sample Approach");
        robot.rotateTheta(1.0, 0, "Rotating");
        robot.runDriveTrain(robot.frontDrive, robot.backDrive, -0.25, 3, "Approach Initial Sample");

        //3-4. Sample Positioning and Sampling
        boolean sampleFound = false; //Whether the sample is found
        double sampleClearance = 5.0; //Time to clear sampling field
        double remainingSampleClearance = 0; //Time remaining after sampling
        robot.rotateTheta(1.0, 0, "Rotating");
        robot.frontDrive.setPower(0.25);
        robot.backDrive.setPower(0.25);
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
        robot.rotateTheta(1.0, -135, "Rotating");

        //6. Approach Field Wall
        robot.runDriveTrain(robot.leftDrive, robot.rightDrive, -0.25, 3, "Approaching Field Wall");
        robot.DriveForwardDistance(robot.leftDrive, robot.rightDrive, 0.25, 10000, 10.0);


        //7. Approach Depot
        robot.runDriveTrain(robot.frontDrive, robot.backDrive, 0.25, 3, "Approaching Depot");

        //8. Drop Mascot
        robot.mascot.setPosition(robot.setPosition);

        //9. Approach Crater
        robot.runDriveTrain(robot.frontDrive, robot.backDrive, -0.25, 3, "Approaching Crater");

        //10. Rotate robot
        robot.rotateTheta(1.0, 45, "Rotating");

        //11. Extending arm over crater
        //Misnomer, runDriveTrain is used to run motor pair
        robot.runDriveTrain(robot.leftExtension, robot.rightExtension, 0.25, 1, "Extending");

        telemetry.addData("Autonomous Completed", "Please Wait");
        telemetry.update();
    }
}
