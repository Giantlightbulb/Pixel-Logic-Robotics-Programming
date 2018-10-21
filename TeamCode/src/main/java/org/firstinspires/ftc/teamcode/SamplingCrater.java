package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SamplingCrater extends LinearOpMode {
    HardwareOmni robot = new HardwareOmni(); // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    static final double     SLOWER_SPEED = 0.2;
    static final int red =255;
    static final int green = 178;
    static final int blue = 0;
    boolean mineralNotSeen;
    boolean mineralSeen;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.motor1.setPower(FORWARD_SPEED);
        robot.motor3.setPower(FORWARD_SPEED);
        runtime.reset();
        robot.colors = robot.colorSensor.getNormalizedColors();
        while (opModeIsActive() && (mineralNotSeen = true)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.motor1.setPower(0);
        robot.motor3.setPower(0);
        while (mineralNotSeen){
            robot.motor1.setPower(SLOWER_SPEED);
            robot.motor3.setPower(SLOWER_SPEED);
            runtime.reset();
            while (opModeIsActive() && (mineralNotSeen = true)) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
        }
        robot.motor1.setPower(0);
        robot.motor3.setPower(0);
        if (Math.abs(robot.colors.red - red) < 10 &&
                Math.abs(robot.colors.green - green) < 10 &&
                Math.abs(robot.colors.blue - blue) < 10){
            pushGold();
            moveLeft(); // (sideways relative to heading)
            } else {
            moveRight();
            if (Math.abs(robot.colors.red - red) < 10 &&
                    Math.abs(robot.colors.green - green) < 10 &&
                    Math.abs(robot.colors.blue - blue) < 10) {
                pushGold();
                moveLeft();
                moveLeft();//(we need to go twice the distance)
            } else{
                    moveLeft();
                    moveLeft();
                }
                if (Math.abs(robot.colors.red - red) < 10 &&
                        Math.abs(robot.colors.green - green) < 10 &&
                        Math.abs(robot.colors.blue - blue) < 10) {
                   pushGold();
                } else {
                 //  rerun the program
                }
            }
        }


    public void moveLeft() {
        robot.motor2.setPower(SLOWER_SPEED);
        robot.motor4.setPower(SLOWER_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <1.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.motor2.setPower(0);
        robot.motor4.setPower(0);

    }
    public void moveRight() {
        robot.motor2.setPower(-SLOWER_SPEED);
        robot.motor4.setPower(-SLOWER_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <1.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.motor2.setPower(0);
        robot.motor4.setPower(0);

    }
    public void pushGold() {
        robot.motor1.setPower(SLOWER_SPEED);
        robot.motor3.setPower(SLOWER_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <1.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.motor1.setPower(SLOWER_SPEED);
        robot.motor3.setPower(SLOWER_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <1.0)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.motor1.setPower(0);
        robot.motor3.setPower(0);
    }
   }
