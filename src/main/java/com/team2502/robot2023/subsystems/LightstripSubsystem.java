package com.team2502.robot2023.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.Optional;

import com.team2502.robot2023.Constants;
import com.team2502.robot2023.Constants.Subsystems.Leds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightstripSubsystem extends SubsystemBase {

    public static class Animations {
        static Animation off = ((s,f)->{
            s.buffer.setRGB(0,0,0,0);
            return false;
        });
        Animation request_cube = ((s,f)->{
            s.fillColor(Rotation2d.fromDegrees(-90),Rotation2d.fromDegrees(90),Color.kPurple);
            return false;
        });
        Animation request_cone = ((s,f)->{
            s.fillColor(Rotation2d.fromDegrees(-90),Rotation2d.fromDegrees(90),Color.kYellow);
            return false;
        });
    };

    @FunctionalInterface
    public interface Animation {
        boolean tick(Lightstrip strip, double frame);
    }

    class Lightstrip {
        AddressableLED strip;
        public AddressableLEDBuffer buffer;

        public Lightstrip(int port, int count) {
            strip = new AddressableLED(port);
            strip.setLength(count);
            buffer = new AddressableLEDBuffer(count);

            strip.setData(buffer);
            strip.start();
        }

        public int getID(Rotation2d angle) {
            return (int) ((angle.getDegrees()%360)/90)*(Leds.LED_LEFT-Leds.LED_AHEAD) + Leds.LED_AHEAD;
        }

        public Rotation2d getAngle(int id) {
            return Rotation2d.fromDegrees((id-Leds.LED_AHEAD)/(Leds.LED_LEFT-Leds.LED_AHEAD) * 90);
        }

        /** fill angle range with a solid color
         *
         * @param from robot-centric angle to start range (counterclockwise)
         * @param to robot-centric angle to end range (must be greater than from)
         * @param color color to set range to
         */
        public void fillColor(Rotation2d from, Rotation2d to, Color color) {
            // assumes led strip goes counterclockwise
            for (int i = getID(from); i < getID(to); i++) {
                buffer.setLED(i, color);
            }
        }
    }

    class ScheduledAnimation implements Comparable<ScheduledAnimation> {
        Animation animation;
        int frameCount;
        int order;

        ScheduledAnimation(Animation animation, int order) {
            super();
            this.animation = animation;
            this.order = order;
            frameCount = 0;
        }

        boolean tick(Lightstrip lightstrip) {
            frameCount += 1;
            return animation.tick(lightstrip, frameCount);
        }

        public int getOrder() {
            return order;
        }

        @Override
        public int compareTo(ScheduledAnimation animation) {
            return ((ScheduledAnimation) animation).getOrder() - this.order;
        }

    }

    Lightstrip strip;
    ArrayList<ScheduledAnimation> animations;
    Timer frameTimer;

    public LightstripSubsystem() {
        strip = new Lightstrip(Leds.PORT,Leds.LED_COUNT);

        animations = new ArrayList<>(2);

        animations.add(new ScheduledAnimation(Animations.off, 0));
    }

    @Override
    public void periodic() {
        if (!frameTimer.advanceIfElapsed(1/Leds.FRAME_RATE)) {
            return;
        }
        Collections.sort(animations);
        Iterator<ScheduledAnimation> i = animations.iterator();
        while (i.hasNext()) {
            ScheduledAnimation animation = i.next();
            if (!animation.tick(strip)) {
                animations.remove(animation);
            }
        }
    }

    public ScheduledAnimation scheduleAnimation(Animation animation, int order) {
        ScheduledAnimation scheduledAnimation = new ScheduledAnimation((Animation) animation, order);
        animations.add(scheduledAnimation);
        return scheduledAnimation;
    }

    public void cancelAnimation(ScheduledAnimation animation) {
        animations.remove(animation);
    }

}
