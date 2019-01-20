package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="StackBasedAutonomous", group="Autonomous")

public class StackBasedAutonomous extends LinearOpMode {
    ArmHardwareOmni robot = new ArmHardwareOmni();

    public void runOpMode() {
        robot.init(hardwareMap);
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
        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        telemetry.update();

        //Initiate Servos
        robot.latch.setPosition(0.65); // latched
        robot.mascot.setPosition(0.8);//mascot up

        // Start Button
        waitForStart();
        robot.timer.reset();
        while(opModeIsActive()) {
            telemetry.addLine()
                    .addData("Compass1:", "null");
            telemetry.addLine()
                    .addData("Compass2:", "null");
            telemetry.addLine()
                    .addData("Gyro:", "null");
            telemetry.addLine()
                    .addData("Range Sensor:", "null");
            telemetry.addLine()
                    .addData("Color Sensor", "null");
            telemetry.addLine()
                    .addData("Compass1:", "null");
            telemetry.addLine()
                    .addData("Compass1:", "null");
            telemetry.update();
        }
        telemetry.clear();
    }
}
