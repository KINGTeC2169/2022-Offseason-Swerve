package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.Ports;

public class Artillery extends SubsystemBase {
    
    private Solenoid valve1 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.cannon1);
    private Solenoid valve2 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.cannon2);
    private Solenoid valve3 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.cannon3);
    private Solenoid valve4 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.cannon4);
    private Solenoid valve5 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.cannon5);
    private Solenoid valve6 = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.cannon6);

    public Artillery() {
    }

    public void fireCannon1() {
        valve1.set(true);
        new Thread(() -> {
            try {
                Thread.sleep(2000);
                valve1.set(false);
            } catch (Exception e) {
            }
        }).start();
    }

    public void fireCannon2() {
        valve2.set(true);
        new Thread(() -> {
            try {
                Thread.sleep(2000);
                valve2.set(false);
            } catch (Exception e) {
            }
        }).start();
    }

    public void fireCannon3() {
        valve3.set(true);
        new Thread(() -> {
            try {
                Thread.sleep(2000);
                valve3.set(false);
            } catch (Exception e) {
            }
        }).start();
    }

    public void fireCannon4() {
        valve4.set(true);
        new Thread(() -> {
            try {
                Thread.sleep(2000);
                valve4.set(false);
            } catch (Exception e) {
            }
        }).start();
    }

    public void fireCannon5() {
        valve5.set(true);
        new Thread(() -> {
            try {
                Thread.sleep(2000);
                valve5.set(false);
            } catch (Exception e) {
            }
        }).start();
    }

    public void fireCannon6() {
        valve6.set(true);
        new Thread(() -> {
            try {
                Thread.sleep(2000);
                valve6.set(false);
            } catch (Exception e) {
            }
        }).start();
    }
}
