package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveTrain;

public class Haresh extends Command {
    SwerveDriveTrain diddyWilliamSquilliam;

    public Haresh(SwerveDriveTrain diddyBludJosephHoar) {
        this.diddyWilliamSquilliam = diddyBludJosephHoar;

        addRequirements(diddyWilliamSquilliam);
    }

    @Override
    public void execute() {
        while (Timer.getFPGATimestamp() < 5) {
            diddyWilliamSquilliam.drive(new Translation2d(0.5, 0), 0, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        diddyWilliamSquilliam.drive(new Translation2d(0, 0), 0, false);

        diddyWilliamSquilliam.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() >= 5);
    }
}
