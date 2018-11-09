package org.firstinspires.ftc.teamcode;

//TeleOp and Hardware
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.SwitchableLight;



import android.graphics.Color;
@Autonomous(name="AutonomousCraterSide", group="Autonomous")

public class AutonomousCraterSide extends LinearOpMode {
    HardwareOmni robot = new HardwareOmni();
    public void runOpMode() {
        robot.init(hardwareMap);
        //Handles the current autonomous with drive by time
        telemetry.log().add("Gyro Calibrating. Do Not Move!");


        //Gyro calibrating
        robot.modernRoboticsI2cGyro.calibrate();

        while (!isStopRequested() && robot.modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(robot.timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        //Finalized gyro calibration.
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        waitForStart();
        robot.timer.reset();
        /*
        Motor 1 and 3: Left and Right Movement
        Motor 2 and 4: Forward and Backwards Movement
        Motor 5: Horizontal Extension
        Motor 6: Vertical Extension
        Motor 7: Chariot Rotation
        Motor 8: Whipper
         */
        /*
        //Lower the robot
        //Telescoping lift lower down
        robot.motor6.setPower(0.5);
        while (opModeIsActive() && (robot.timer.seconds() < 3.0)){
            telemetry.addData("Lower Down", "Lift: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.motor6.setPower(0);
        robot.timer.reset();

        //Sideways motion to clear hook
        double clearance = 3.0;
        robot.motor2.setPower(0.5);
        robot.motor4.setPower(-0.5);
        while (opModeIsActive() && (robot.timer.seconds() < clearance)){
            telemetry.addData("Clearance", "Left and Right: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.motor2.setPower(0);
        robot.motor4.setPower(0);
        robot.timer.reset();

        //Backoff from Lander
        robot.motor1.setPower(0.5);
        robot.motor3.setPower(-0.5);
        while (opModeIsActive() && (robot.timer.seconds() < 3.0)){
            telemetry.addData("Backoff", "Left and Right: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.motor1.setPower(0);
        robot.motor3.setPower(0);
        robot.timer.reset();

        //Recenter
        //Same time for the clearance
        robot.motor2.setPower(-0.5);
        robot.motor4.setPower(0.5);
        while (opModeIsActive() && (robot.timer.seconds() < clearance)){
            telemetry.addData("Recenter", "Left and Right: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.motor2.setPower(0);
        robot.motor4.setPower(0);
        robot.timer.reset();

        //Sample
        //Approach position for sampling
        //Diagonal movement to establish position
        //Back
        robot.motor1.setPower(0.5);
        robot.motor3.setPower(-0.5);
        //Right
        robot.motor2.setPower(0.5);
        robot.motor4.setPower(-0.5);
        while (opModeIsActive() && (robot.timer.seconds() < 3.0)) {
            telemetry.addData("Sample Position", "Left and Right: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.motor1.setPower(0);
        robot.motor3.setPower(0);
        //Movement parallel to sampling
        robot.motor2.setPower(-0.5);
        robot.motor4.setPower(0.5);
        robot.timer.reset();
        */

        //Need to add color condition
        double timeToFind = 0;
        int alpha = 0;
        int red = 0;
        int green = 0;
        int blue = 0;
        while (opModeIsActive() &&
                (robot.timer.seconds() < 10.0) &&
                !((Math.abs(alpha - 70) < 50 ) &&
                (Math.abs(red - 255) < 50 ) &&
                (Math.abs(green - 107) < 50 ) &&
                (Math.abs(blue) < 10 ))) {
            robot.colors = robot.colorSensor.getNormalizedColors();
            float max = Math.max(Math.max(Math.max(robot.colors.red, robot.colors.green), robot.colors.blue), robot.colors.alpha);
            robot.colors.red   /= max;
            robot.colors.green /= max;
            robot.colors.blue  /= max;
            int color = robot.colors.toColor();
            alpha = Color.alpha(color);
            red = Color.red(color);
            green = Color.green(color);
            blue = Color.blue(color);
            telemetry.addData("Sampling", "Sampling: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.addData("a", alpha);
            telemetry.addData("r", red);
            telemetry.addData("g", green);
            telemetry.addData("b", blue);
            //Amount of time taken to find the sample
            timeToFind = robot.timer.seconds();
            telemetry.update();
        }
        robot.timer.reset();
        robot.motor2.setPower(0);
        robot.motor4.setPower(0);


        //Retrieves mineral only if the mineral was correctly sampled/the correct color was found
        if (((Math.abs(alpha - 70) < 50 ) &&
                (Math.abs(red - 255) < 50 ) &&
                (Math.abs(green - 107) < 50 ) &&
                (Math.abs(blue) < 10 ))) {
            robot.motor2.setPower(0);
            robot.motor4.setPower(0);
            //Extension
            robot.motor5.setPower(1);
            //Whipper is maintained
            //robot.motor8.setPower(1);
            while (opModeIsActive() && (robot.timer.seconds() < 3.0)) {
                telemetry.addData("Retrieving mineral", "Retrieving: %2.5f S Elapsed", robot.timer.seconds());
                telemetry.update();
            }
            //Retraction
            robot.motor5.setPower(-1);
            while (opModeIsActive() && (robot.timer.seconds() < 3.0)) {
                telemetry.addData("Returning mineral", "Returning: %2.5f S Elapsed", robot.timer.seconds());
                telemetry.update();
            }
            robot.motor5.setPower(0);
        }

        //Return to original position
        robot.timer.reset();
        //Reversed motion
        robot.motor2.setPower(0.5);
        robot.motor4.setPower(-0.5);
        //Moves for the same length of time
        while (opModeIsActive() && (robot.timer.seconds() < timeToFind)){
            telemetry.addData("Returning to initial position", "Returning: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.motor2.setPower(0);
        robot.motor4.setPower(0);
        robot.timer.reset();

        //Depot
        //Straight line towards the depot
        robot.motor2.setPower(-0.5);
        robot.motor4.setPower(0.5);
        robot.motor1.setPower(-0.5);
        robot.motor3.setPower(0.5);
        while (opModeIsActive() && (robot.timer.seconds() < 3)){
            telemetry.addData("Approaching depot", "Approaching: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.motor1.setPower(0);
        robot.motor3.setPower(0);
        robot.timer.reset();
        //Diagonal towards wall
        while (opModeIsActive() && (robot.timer.seconds() < 3)){
            telemetry.addData("Approaching depot wall", "Approaching: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        double depotEnter = 3.0;
        //Diagonal towards depot
        robot.motor2.setPower(0.5);
        robot.motor4.setPower(-0.5);
        robot.timer.reset();
        while (opModeIsActive() && (robot.timer.seconds() < depotEnter)){
            telemetry.addData("Entering Depot", "Entering: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.motor2.setPower(0);
        robot.motor4.setPower(0);
        robot.timer.reset();
        //Mascot knockoff
        //Mineral Drop
        robot.motor5.setPower(0);
        robot.servo1.setPosition(90);
        robot.servo1.setPosition(0);
        //Return to wall
        robot.motor2.setPower(-0.5);
        robot.motor2.setPower(0.5);
        while (opModeIsActive() && (robot.timer.seconds() < depotEnter)){
            telemetry.addData("Exiting Depot", "Exiting: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.timer.reset();

        /* Could account for traffic but currently unfinished and unrealistic
        //Robot rotates 90 degrees to the wall
        robot.motor2.setPower(0.5);
        robot.motor4.setPower(0.5);
        robot.motor1.setPower(0.5);
        robot.motor3.setPower(0.5);
        robot.timer.reset();

        double initialAngle = robot.gyro.getAngularOrientation(robot.aRefInt, robot.aOrderXYZ, robot.aUnit).firstAngle;
        while (opModeIsActive() && (robot.timer.seconds() < 3.0) && (Math.abs(initialAngle -
                robot.gyro.getAngularOrientation(robot.aRefInt, robot.aOrderXYZ, robot.aUnit).firstAngle) < 90)){
            telemetry.addData("Rotating towards wall", "Rotating: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        double minimumSafeDistance = robot.rangeSensor.getDistance(DistanceUnit.CM);
        robot.timer.reset();
        initialAngle = robot.gyro.getAngularOrientation(robot.aRefInt, robot.aOrderXYZ, robot.aUnit).firstAngle;
        while (opModeIsActive() && (robot.timer.seconds() < 3.0) && (Math.abs(initialAngle -
        */

        //Park Crater
        //Movement towards crater
        robot.motor1.setPower(0.5);
        robot.motor3.setPower(-0.5);
        while (opModeIsActive() && (robot.timer.seconds() < 3.0)){
            telemetry.addData("Parking", "Parking: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.timer.reset();
        robot.motor2.setPower(0);
        robot.motor4.setPower(0);
        robot.motor1.setPower(0);
        robot.motor3.setPower(0);
        telemetry.addLine()
                .addData("Parked:", "Completed");
    }
}
