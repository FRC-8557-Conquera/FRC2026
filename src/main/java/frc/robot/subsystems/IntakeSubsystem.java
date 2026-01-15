package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkMax intakeLeft;
  private final SparkMax intakeRight;
  private final SparkMax intakeAci;

  public IntakeSubsystem() {
    intakeLeft = new SparkMax(Constants.Intake.intakeFollower1, MotorType.kBrushless);
    intakeRight = new SparkMax(Constants.Intake.intakeFollower2, MotorType.kBrushless);
    intakeAci = new SparkMax(Constants.Intake.intakeRoller, MotorType.kBrushless);

    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(40);

    SparkMaxConfig angleConfig = new SparkMaxConfig();
    angleConfig.idleMode(SparkBaseConfig.IdleMode.kBrake).smartCurrentLimit(30);

    intakeLeft.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeRight.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeAci.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intakeIn() {
    intakeLeft.set(0.8);
    intakeRight.set(-0.8);
  }

  public void intakeOut() {
    intakeLeft.set(-0.6);
    intakeRight.set(0.6);
  }

  public void stopRoller() {
    intakeLeft.stopMotor();
    intakeRight.stopMotor();
  }

  public void angleUp() {
    intakeAci.set(0.4);
  }

  public void angleDown() {
    intakeAci.set(-0.4);
  }

  public void stopAngle() {
    intakeAci.stopMotor();
  }
}
