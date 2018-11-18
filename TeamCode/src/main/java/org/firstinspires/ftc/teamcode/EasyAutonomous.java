//EASY AUTONOMOUS

package org.firstinspires.ftc.teamcode;

//TeleOp and Hardware
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.SwitchableLight;



import android.graphics.Color;
@Autonomous(name="EasyAutonomous", group="Autonomous")

public class EasyAutonomous extends LinearOpMode {
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
        Motor 8: NA
         */
        //Lower the robot
        //Telescoping lift lower down
        //  ----------------------------------current location

/*
        robot.motor6.setPower(-0.3);
        while (opModeIsActive() && (robot.timer.seconds() < 0.1)){
            telemetry.addData("Lower Down", "Lift: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.motor6.setPower(0);
        robot.servo2.setPosition(0.68);
        robot.timer.reset();

        while (opModeIsActive() && (robot.timer.seconds() < 0.2)){
            telemetry.addData("Short Pause", robot.timer.seconds());
            telemetry.update();
        }


        robot.motor6.setPower(0.2);
        robot.timer.reset();

        while (opModeIsActive() && (robot.timer.seconds() < 2.0)){
            telemetry.addData("Lower Down", "Lift: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.motor6.setPower(0);
        robot.timer.reset();
        */
        double CLEARANCE = 0.6;
        robot.motor2.setPower(0.2); // go left 2 inches
        robot.motor4.setPower(-0.2);// assist left 2 inches
        robot.timer.reset();

        while (opModeIsActive() && (robot.timer.seconds() < CLEARANCE)){
            telemetry.addData("Clearance", "Left and Right: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.motor2.setPower(0);
        robot.motor4.setPower(0);
        robot.timer.reset();

         // END LOWER DOWN AUTONOMOUS TEST SEQUENCE

        // Crater/Depot Dash ==================================================
        robot.motor1.setPower(0.4);
        robot.motor3.setPower(-0.4);
        while (opModeIsActive() && (robot.timer.seconds() < 2.6)) {
            telemetry.addData("Dashing", "Dashing %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.motor1.setPower(0);
        robot.motor3.setPower(0);

        robot.motor1.setPower(-0.2);
        robot.motor3.setPower(0.2);
        while (opModeIsActive() && (robot.timer.seconds() < 0.25)) {
            telemetry.addData("Dashing", "Dashing %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.motor1.setPower(0);
        robot.motor3.setPower(0);

        //Mascot knockoff
        robot.servo1.setPosition(0);

        telemetry.addLine()
                .addData("Parked:", "Completed");

    }
}

