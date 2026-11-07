/* MIT License
 * Copyright (c) [2026] [Base 10 Assets, LLC]
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Sideways9", group = "StarterBot")
public class maybeSideways9 extends OpMode {

    // Declare OpMode members.
    private DcMotor left_front_drive = null;
    private DcMotor left_back_drive = null;
    private DcMotor right_front_drive = null;
    private DcMotor right_back_drive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;
    private IMU imu = null;
    
    public Gamepad voidpad;
    double mixup;

    public final int LAUNCHER_TARGET_VELOCITY = 300; 
    public final int LAUNCHER_MIN_VELOCITY = 100;    

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    double intakePower;

    // Field-centric variables
    boolean fieldCentric = true;
    boolean aButtonLastState = false;

    @Override
    public void init() {
        left_front_drive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
        left_back_drive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        right_back_drive  = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake            = hardwareMap.get(DcMotor.class, "intake");
        launcher          = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo     = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo   = hardwareMap.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo  = hardwareMap.get(CRServo.class, "rightIntakeServo");

        // Initialize the IMU for Field Centric
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        left_back_drive.setDirection(DcMotor.Direction.REVERSE);
        right_back_drive.setDirection(DcMotor.Direction.FORWARD);

        left_front_drive.setZeroPowerBehavior(BRAKE);
        right_front_drive.setZeroPowerBehavior(BRAKE);
        left_back_drive.setZeroPowerBehavior(BRAKE);
        right_back_drive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        // Flipped servo directions (Left is now REVERSE, Right is FORWARD)
        leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        imu.resetYaw();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up == true){
            mixup += 1;
        }else if (gamepad1.dpad_down == true){
            mixup -= 1;
        }
        if (mixup >= -100){
            voidpad = gamepad2;
            
        }else if (mixup<-100 || gamepad2.getGamepadId() == -1){
            voidpad = gamepad1;
        }
        // --- GAMEPAD 1: DRIVING ---
        
        if (gamepad1.a && !aButtonLastState) {
            fieldCentric = !fieldCentric;
        }
        aButtonLastState = gamepad1.a;

        if (gamepad1.y) {
            imu.resetYaw();
        }

        double forward = -gamepad1.left_stick_y;
        double strafe  = gamepad1.left_stick_x;
        double rotate  = gamepad1.right_stick_x;

        if (fieldCentric) {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

            mecanumDrive(rotY, rotX, rotate);
        } else {
            mecanumDrive(forward, strafe, rotate);
        }

        // --- GAMEPAD 2: SHOOTER & INTAKE ---
        
        intakePower = voidpad.left_trigger;
        launch();

        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);
        
        
        telemetry.addData("mixup movement away from tyranny",mixup);
        telemetry.addLine("mixup > -100 is co-op");
        telemetry.addData("Field Centric", fieldCentric ? "ON" : "OFF");
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
    }

    @Override
    public void stop() {
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        leftFrontPower  = Math.min(Math.max(forward + strafe + rotate, -1), 1);
        rightFrontPower = Math.min(Math.max(forward - strafe - rotate, -1), 1);
        leftBackPower   = Math.min(Math.max(forward - strafe + rotate, -1), 1);
        rightBackPower  = Math.min(Math.max(forward + strafe - rotate, -1), 1);

        left_front_drive.setPower(leftFrontPower);
        right_front_drive.setPower(rightFrontPower);
        left_back_drive.setPower(leftBackPower);
        right_back_drive.setPower(rightBackPower);
    }

    void launch() {
        
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY * voidpad.right_trigger);
        if (voidpad.left_bumper){
            intakePower = (-1) * intakePower;
        }
        // X  and B now ONLY controls windmillServo
        if (voidpad.x){
            windmillServo.setPower(1);
        } else if(voidpad.b){
            
                windmillServo.setPower(-1);
            
            
        } else {
            windmillServo.setPower(0);
        }
    }
}
