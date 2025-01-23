package frc.robot.subsystem;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.PreferenceKeys;

public class SwerveModule {

    @AutoLog
    public static class SwerveModuleIOInputs {
        public double driveVelocityRadPerSec = 0.0;
        public double drivePositionRad = 0.0;
        public double driveVoltsAppliedToMotor = 0.0;
        public double driveVoltsSuppliedToMotorController = 0.0;

        public Rotation2d turnMotorControllerPosition = new Rotation2d();
        public Rotation2d absoluteEncoderPosition = new Rotation2d();
        public double turnVoltsAppliedToMotor = 0.0;
        public double turnVoltsSuppliedToMotorController = 0.0;

        public double absolutEncoderRotationsSinceLastReset = 0.0;

    // public double[] odometryTimestamps = new double[] {};
    // public double[] odometryDrivePositionsRad = new double[] {};
    // public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    private final DutyCycleEncoder absoluteEncoder;

    private final TalonFX driveMotorController;
    private final VelocityVoltage driveVelocityInput = new VelocityVoltage(0);//default requency is 100Hz, every 10 ms
    private final DCMotorSim driveMotorSim;
    private TalonFXSimState driveMotorControllerSimState;

    private final TalonFX turnMotorController;
    private final PositionVoltage turnPositionInput;
    private final DCMotorSim turnMotorSim; 
    private TalonFXSimState turnMotorControllerSimState;
    
    // Inputs from drive motor
    private final StatusSignal<AngularVelocity> driveAngularVelocity;
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<Voltage> driveVoltsAppliedToMotor;
    private final StatusSignal<Voltage> driveVoltsSuppliedToMotorController;

    // Inputs from turn motor
    private final StatusSignal<Angle> turnPosition;
    private final StatusSignal<Voltage> turnVoltsAppliedToMotor;
    private final StatusSignal<Voltage> turnVoltsSuppliedToMotorController;


    public SwerveModule(int driveDeviceId, int turnDeviceId, int absoluteEncoderPort) {
        driveMotorController = new TalonFX(driveDeviceId);
        turnMotorController = new TalonFX(turnDeviceId);
        //The assumption is a full cycle is the wheel turning a full circle which is why 2*PI.  Therefore we expect 0 at 0
        //By doing this the desire is that we are directly getting the radian value when we call the get method
        absoluteEncoder = new DutyCycleEncoder(new DigitalInput(absoluteEncoderPort), 2*Math.PI, 0);
        turnPositionInput = new PositionVoltage(absoluteEncoder.get()); //set position to what absolute encoder indicates

        driveAngularVelocity = driveMotorController.getVelocity();
        drivePosition = driveMotorController.getPosition();
        driveVoltsAppliedToMotor = driveMotorController.getMotorVoltage();
        driveVoltsSuppliedToMotorController = driveMotorController.getSupplyVoltage();
        
        turnPosition = turnMotorController.getPosition();
        turnVoltsAppliedToMotor = turnMotorController.getMotorVoltage();
        turnVoltsSuppliedToMotorController = turnMotorController.getSupplyVoltage();

        if(RobotBase.isSimulation()) {
            driveMotorControllerSimState = driveMotorController.getSimState();
            driveMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getFalcon500(1), 0.001, 10.0
                ),
                DCMotor.getFalcon500(1)
            );

            turnMotorControllerSimState = turnMotorController.getSimState();
            turnMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getFalcon500(1), 0.001, 10.0
                ),
                DCMotor.getFalcon500(1)
            );
        } else {
            driveMotorSim = null;
            turnMotorSim = null;
        }
    }

    public void updateInputs(SwerveModuleIOInputs inputs){
        BaseStatusSignal.refreshAll(driveAngularVelocity, drivePosition, driveVoltsAppliedToMotor, driveVoltsSuppliedToMotorController, turnPosition, turnVoltsAppliedToMotor, turnVoltsSuppliedToMotorController);

        inputs.driveVelocityRadPerSec=Units.rotationsToRadians(driveAngularVelocity.getValueAsDouble());
        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVoltsAppliedToMotor=driveVoltsAppliedToMotor.getValueAsDouble();
        inputs.driveVoltsSuppliedToMotorController=driveVoltsSuppliedToMotorController.getValueAsDouble();
        
        //this is the module's angle measured from the motor controller's onboard relative encoder
        inputs.turnMotorControllerPosition=Rotation2d.fromRotations(turnPosition.getValueAsDouble());
        inputs.turnVoltsAppliedToMotor=turnVoltsAppliedToMotor.getValueAsDouble();
        inputs.turnVoltsSuppliedToMotorController=turnVoltsSuppliedToMotorController.getValueAsDouble();

        //to represent the module angle we will assume the absolute encoder is the source of true
        inputs.absoluteEncoderPosition = getAbsoluteEncoderPosition(); //this handles both real and simulation values
    }

    public void setModuleState(SwerveModuleState desiredSwerveModuleStates) {
        //figures out if it only needs to do a smaller angle change and run the motor in the reverse direction
        desiredSwerveModuleStates.optimize(getAbsoluteEncoderPosition());

        //omega (angular velocity in radians per second) = velocity/radius
        double desiredSpeedOfTheWheelInAngularVelocity = desiredSwerveModuleStates.speedMetersPerSecond/Preferences.getDouble(PreferenceKeys.WHEEL_RADIUS_METERS, DriveConstants.WHEEL_RADIUS_DEFAULT_VALUE);
        Rotation2d desiredAngleOfTheWheel = desiredSwerveModuleStates.angle;

        turnMotorController.setControl(turnPositionInput.withPosition(desiredAngleOfTheWheel.getRotations()));//expect rotations
        driveMotorController.setControl(driveVelocityInput.withVelocity(Units.radiansToRotations(desiredSpeedOfTheWheelInAngularVelocity)));//expect rotations per second

        if(RobotBase.isSimulation()) {
            simulateModule((desiredSpeedOfTheWheelInAngularVelocity), desiredAngleOfTheWheel.getRadians());
        }
    }

    private void simulateModule(double desiredSpeedOfTheWheelInAngularVelocity, double desiredAngleOfTheWheelInRadians) {
        driveMotorSim.setAngularVelocity(desiredSpeedOfTheWheelInAngularVelocity);//expect radians per second
        driveMotorSim.update(.001);//This seems to be the sim update frequency that seems to be the closest to reality.  Not sure why.
        //the driveMotorSim seems to half the angular velocity, not sure why - simulation is accurate up to half the angular velocity
        driveMotorControllerSimState.setRotorVelocity(driveMotorSim.getAngularVelocity());

        turnMotorSim.setAngle(desiredAngleOfTheWheelInRadians);//expect radians
        turnMotorSim.update(.001);
        turnMotorControllerSimState.setRawRotorPosition(turnMotorSim.getAngularPosition());
    }

    //needed to simulate gyro and for odometry
    public SwerveModulePosition getPosition(SwerveModuleIOInputs inputs) {
        return new SwerveModulePosition(inputs.drivePositionRad * Preferences.getDouble(PreferenceKeys.WHEEL_RADIUS_METERS, DriveConstants.WHEEL_RADIUS_DEFAULT_VALUE), inputs.absoluteEncoderPosition);
    }

    private Rotation2d getAbsoluteEncoderPosition() {
        Rotation2d result;
        if(RobotBase.isSimulation()) {
            result = new Rotation2d(turnMotorSim.getAngularPositionRad());
        } else {
            result = new Rotation2d(absoluteEncoder.get());
        }
        return result;
    }
}
