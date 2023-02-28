package com.team2502.robot2023.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.Optional;
import java.util.function.Function;

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

    /** Class for static animations not tied to a subsystem 
     * 
     * {@link Animation}s that need to accept data inputs should be defined elsewhere
     * */
    public static final class Animations {
        public static final Animation off = ((s,f)->{
            s.buffer.setRGB(0,0,0,0);
            return false;
        });
        public static final Animation request_cube = ((s,f)->{
            s.fillColor(Rotation2d.fromDegrees(-90),Rotation2d.fromDegrees(90),Color.kPurple);
            return false;
        });
        public static final Animation request_cone = ((s,f)->{
            s.fillColor(Rotation2d.fromDegrees(-90),Rotation2d.fromDegrees(90),Color.kYellow);
            return false;
        });
        public static final Animation orbit_demo = ((s,f)->{
            Rotation2d angle = Rotation2d.fromDegrees(f*3);
            Rotation2d offset = Rotation2d.fromDegrees(30);
            s.fillColor(angle.minus(offset),angle.plus(offset),Color.kWhite);
            return false;
        });
    };

    @FunctionalInterface
    public interface Animation {
        /** preform one tick of an animation
         *
         * @param strip led strip to set colors on
         * @param frame execution count, starts at 1
         */
        boolean tick(Lightstrip strip, double frame);
    }

    public class Lightstrip {
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

        /** fill angle range based using given function
         *
         * @param from robot-centric angle to start range (counterclockwise)
         * @param to robot-centric angle to end range (must be greater than from)
         * @param mapFunction rotation to color transform
         */
        public void fillMap(Rotation2d from, Rotation2d to, Function<Rotation2d,Color> mapFunction) {
            for (int i = getID(from); i < getID(to); i++) {
                buffer.setLED(i, mapFunction.apply(getAngle(i)));
            }
        }
    }

    public class ScheduledAnimation implements Comparable<ScheduledAnimation> {
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
            return animation.getOrder() - this.order;
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
