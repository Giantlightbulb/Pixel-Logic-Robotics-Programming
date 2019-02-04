package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import android.graphics.Color;

@Autonomous(name="StateAutonomousDepot", group="Autonomous")

public class StateAutonomousDepot extends LinearOpMode {
    ArmHardwareOmni robot = new ArmHardwareOmni();

    public void runOpMode() {
        robot.init(hardwareMap);
        int targetPosition;
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
        robot.mascot.setPosition(0.8);//mascot up
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
        //END INITIALIZATION---------------------------------------------------------------------

        robot.verticalLift.setPower(0.85); // lifts robot up
        while (opModeIsActive() && (robot.timer.seconds() < 0.3)) {
            telemetry.addData("Lower Down", "Lift: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.timer.reset();

        robot.verticalLift.setPower(-0.2); // lowers robot down
        while (opModeIsActive() && (robot.timer.seconds() < 1.7)){
            telemetry.addData("Lower Down", "Lift: %2.5f S Elapsed", robot.timer.seconds());
            telemetry.update();
        }
        robot.verticalLift.setPower(0);




        //Forward Motion (works) - approach Sampling, part 1
        DriveForwardDistance(robot.leftDrive, robot.rightDrive,0.3, 4000,10.0);

        sleep(500);

        sleep(500);

        //Sampling


        //Left Motion (hopefully) - approach Sampling, part 2


        /*robot.leftDrive.setPower(0.3);
        robot.rightDrive.setPower(0.3);
        targetPosition = 5000;
        while((robot.leftDrive.getCurrentPosition() < targetPosition) || (robot.rightDrive.getCurrentPosition() < targetPosition)){

        }
        */
        // end
    }
    public void DriveForwardDistance(DcMotor firstMotor, DcMotor secondMotor, double power, int distance, double timeOut){

        // Reset encoders
        firstMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        secondMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set Target Position b
        firstMotor.setTargetPosition(distance);
        secondMotor.setTargetPosition(distance);

        // Set to RUN_TO_POSITION mode
        firstMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        secondMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set Drive Power
        firstMotor.setPower(power);
        secondMotor.setPower(power);
        robot.timer.reset();

        while((opModeIsActive())&&(firstMotor.isBusy() || secondMotor.isBusy())){
            // wait
            telemetry.addData("Path1",  "Going to %7d ,  currently at %7d and %7d.", firstMotor.getTargetPosition(),  firstMotor.getCurrentPosition(), secondMotor.getCurrentPosition());
            telemetry.update();
        }

        firstMotor.setPower(0);
        secondMotor.setPower(0);

        firstMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        secondMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
