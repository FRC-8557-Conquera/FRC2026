package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {

  private final SparkMax feeder;

  public FeederSubsystem() {
    feeder = new SparkMax(Constants.Feeder.feederMotor, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(30);
    
    feeder.configure(
        config,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters
    );
  }

  public void feed() {
    feeder.set(0.7);
  }

  public void reverse() {
    feeder.set(-0.7);
  }

  public void stop() {
    feeder.stopMotor();
  }
}
