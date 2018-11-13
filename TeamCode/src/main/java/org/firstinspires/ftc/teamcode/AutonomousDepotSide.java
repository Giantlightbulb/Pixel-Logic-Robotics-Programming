package org.firstinspires.ftc.teamcode;

//TeleOp and Hardware
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.SwitchableLight;



import android.graphics.Color;
@Autonomous(name="AutonomousDepotSide", group="Autonomous")

public class AutonomousDepotSide extends LinearOpMode {
    HardwareOmni robot = new HardwareOmni();
    public void runOpMode() {
        robot.init(hardwareMap);
        //Handles the current autonomous with drive by time
        telemetry.log().add("Gyro Calibrating. Do Not Move!");


        //Approximate Speed at full power: 36 inches / 1.75 seconds
        // Left motion motor 2 positive, motor 4 negative
        // Forward motion motor 1 positive motor 3 negative


        //Gyro calibating
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
        Motor 8: NA
         */

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
        double final CLEARANCE = 3.0;
        robot.motor2.setPower(0.5); // go left 2 inches
        robot.motor4.setPower(-0.5);// assist left 2 inches
        while (opModeIsActive() && (robot.timer.seconds() < CLEARANCE)){
            telemetry.addData("Clearance", "Left and Right: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.motor2.setPower(0);
        robot.motor4.setPower(0);
        robot.timer.reset();

        //Leave Lander, GoTo Leftmost Sample (forward backward motion
        robot.motor1.setPower(0.5);
        robot.motor3.setPower(-0.5);
        while (opModeIsActive() && (robot.timer.seconds() < 0.6)){ // 19 inches forward on this vector
            telemetry.addData("Backoff", "Left and Right: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.motor1.setPower(0);
        robot.motor3.setPower(0);
        robot.timer.reset();

        //Leave Lander, GoTo Leftmost Sample (left/right motion) // 14 inches left on this vector
        robot.motor2.setPower(0.5);
        robot.motor4.setPower(-0.5);
        while (opModeIsActive() && (robot.timer.seconds() < 0.5)){
            telemetry.addData("Recenter", "Left and Right: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.motor2.setPower(0);
        robot.motor4.setPower(0);
        robot.timer.reset();
        // Should've approached sampling position.

        //BEGIN SAMPLING ==========================================================================

        //Moving right, parallel to sampling
        robot.motor2.setPower(-0.3);
        robot.motor4.setPower(0.3);
        robot.timer.reset();

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
        robot.motor2.setPower(0);
        robot.motor4.setPower(0);
        robot.timer.reset();

        //Retrieves mineral only if the mineral was correctly sampled
        if (found) {
            robot.motor2.setPower(0);
            robot.motor4.setPower(0);

            robot.motor1.setPower(-0.3);
            robot.motor3.setPower(0.3);
            robot.timer.reset();

            while (opModeIsActive() && (robot.timer.seconds() < 1.0)) {
                telemetry.addData("Backing up to acquire mineral(1 sec)", "Backing up: %2.5f S Elapsed", robot.timer.seconds());
                telemetry.update();
            }
            robot.motor1.setPower(0);
            robot.motor3.setPower(0);

            //Extension
            robot.motor5.setPower(1);
            robot.timer.reset();

            while (opModeIsActive() && (robot.timer.seconds() < 3.0)) {
                telemetry.addData("Prepping Chariot (3 sec)", "Prepping: %2.5f S Elapsed", robot.timer.seconds());
                telemetry.update();
            }

            robot.motor7.setPower(0.5);
            robot.timer.reset();
            while (opModeIsActive() && (robot.timer.seconds() < 1.0)) {
                telemetry.addData("Lowering chariot(1 sec)", "Lowering: %2.5f S Elapsed", robot.timer.seconds());
                telemetry.update();
            }
            robot.motor7.setPower(0);

            //TURN ON WHIPPER
            robot.servo3.setPower(1);


            robot.motor1.setPower(0.3);
            robot.motor3.setPower(-0.3);
            robot.timer.reset();

            while (opModeIsActive() && (robot.timer.seconds() < 1.0)) {
                telemetry.addData("Acquiring mineral(1 sec)", "Acquring: %2.5f S Elapsed", robot.timer.seconds());
                telemetry.update();
            }
            robot.motor1.setPower(0);
            robot.motor3.setPower(0);

            //Retraction
            robot.motor5.setPower(-1);
            while (opModeIsActive() && (robot.timer.seconds() < 3.0)) {
                telemetry.addData("Storing mineral(3 sec)", "Storing: %2.5f S Elapsed", robot.timer.seconds());
                telemetry.update();
            }
            robot.motor5.setPower(0);
        }

        //Return to original position
        robot.timer.reset();
        //Reversed motion
        robot.motor2.setPower(0.3);
        robot.motor4.setPower(-0.3);
        //Moves for the same length of time
        while (opModeIsActive() && (robot.timer.seconds() < timeToFind)){
            telemetry.addData("Returning to initial position (" + timeToFind + " sec)", "Returning: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.motor2.setPower(0);
        robot.motor4.setPower(0);
        robot.timer.reset();

        // BEGIN DEPOT ============================================================================

        // Move past Sampling Row to avoid knocking unobtanium
        robot.motor2.setPower(0.5);
        robot.motor4.setPower(-0.5);
        while (opModeIsActive() && (robot.timer.seconds() < 1)){ // move left 26 inches
            telemetry.addData("Moving past Sampling Row(1 sec) ", "Moving: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.motor1.setPower(0);
        robot.motor3.setPower(0);


        //Spin 45 degrees. Check with the gyro sensor
        robot.motor2.setPower(0.3);
        robot.motor4.setPower(0.3);
        robot.motor1.setPower(0.3);
        robot.motor3.setPower(0.3);
        //

        while (opModeIsActive() && (robot.timer.seconds() < 1)){ // 45 degrees clockwise
            telemetry.addData("Aligning with depot(1 sec)", "Aligning: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.motor1.setPower(0);
        robot.motor3.setPower(0);
        robot.motor2.setPower(0);
        robot.motor4.setPower(0);
        robot.timer.reset();


        robot.motor1.setPower(0.5);
        robot.motor3.setPower(0.5);
        //
        while (opModeIsActive() && (robot.timer.seconds() < 5)){ // 64 inches forward
            telemetry.addData("Approaching depot (5 sec)", "Approaching: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }

        robot.motor1.setPower(0);
        robot.motor3.setPower(0);
        robot.timer.reset();


        //Mascot knockoff
        robot.servo1.setPosition(90);
        robot.servo1.setPosition(0);
        //Mineral Dropoff
        robot.servo3.setPower(-1);
        robot.servo3.setPower(0);
        //Return to wall
        robot.motor2.setPower(-0.5);
        robot.motor4.setPower(0.5);
        robot.timer.reset();

        while (opModeIsActive() && (robot.timer.seconds() < 6.5)){ // 75+ inches to crater
            telemetry.addData("Approaching Crater (6.5 sec)", "Approaching: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.motor2.setPower(0);
        robot.motor4.setPower(0);
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


        robot.motor2.setPower(0);
        robot.motor4.setPower(0);
        robot.motor1.setPower(0);
        robot.motor3.setPower(0);
        telemetry.addLine()
                .addData("Parked:", "Completed");


    }
}

