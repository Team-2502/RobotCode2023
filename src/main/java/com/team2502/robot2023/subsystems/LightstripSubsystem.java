package com.team2502.robot2023.subsystems;

import java.util.Optional;

import com.team2502.robot2023.Constants;
import com.team2502.robot2023.Constants.Subsystems.Leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightstripSubsystem extends SubsystemBase {

    public enum Animations {
        OFF((b,f)->{
            b.setRGB(0,0,0,0);
        });

        public final Animation animation;
        Animations(Animation animation) {
            this.animation = animation;
        }
    };

    @FunctionalInterface
    public interface Animation {
        void tick(
                AddressableLEDBuffer buffer,
                double frame
                );
    }

    Animation animation;
    AddressableLED strip;
    AddressableLEDBuffer buffer;
    Timer cancelTimer; // stops temporary animations
    Timer frameTimer; // counts frames
    int frameCount;
    Optional<Double> timeout;

    public LightstripSubsystem() {
        strip = new AddressableLED(Leds.PORT);
        strip.setLength(Leds.LED_COUNT);
        buffer = new AddressableLEDBuffer(Leds.LED_COUNT);

        strip.setData(buffer);
        strip.start();

        animation = Animations.OFF.animation;
        timeout = Optional.empty();
    }

    @Override
    public void periodic() {
        if (timeout.isPresent() && cancelTimer.hasElapsed(timeout.get())) {
            selectAnimation();
        }
        if (frameTimer.advanceIfElapsed(1/Leds.FRAME_RATE)) {
            frameCount += 1;
            animation.tick(buffer, frameCount);
        }
    }

    public void selectAnimation() {
        setAnimation(Animations.OFF.animation);
    }

    public void setAnimation(Animation animation, Optional<Double> timeout) {
        this.animation = animation;
        this.timeout = timeout;
        frameCount = 0;
        cancelTimer.reset();
        frameTimer.reset();
        if (timeout.isPresent()) cancelTimer.start();
    }

    public void setAnimation(Animation animation, double timeout) {
        setAnimation(animation, timeout);
    }

    public void setAnimation(Animation animation) {
        setAnimation(animation, Optional.empty());
    }
}
