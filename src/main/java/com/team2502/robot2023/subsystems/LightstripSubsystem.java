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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightstripSubsystem extends SubsystemBase {

    /** Class for static animations not tied to a subsystem 
     * 
     * {@link Animation}s that need to accept data inputs should be defined elsewhere
     * */
    public static final class Animations {
        public static final Animation off = ((s,f)->{
            for (int i = 0; i < Leds.LED_COUNT; i++) {
                s.buffer.setRGB(i,55,0,0);
            }
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
        public static final Animation orbit_demo_simple = ((s,f)->{
            s.buffer.setRGB((int) (f/8)%Leds.LED_COUNT,255,148,0);
            return false;
        });
        public static final Animation orbit_demo = ((s,f)->{
            Rotation2d angle = Rotation2d.fromDegrees(f*Leds.FRAME_TIME*90);
            Rotation2d offset = Rotation2d.fromDegrees(30);
            s.fillColor(angle.minus(offset),angle.plus(offset),Color.kPeru);
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

        public void flush() {
            strip.setData(buffer);
            strip.start();
        }

        /** compute led ID for the given robot-centric angle
         *
         * assumes that the LED strip is installed such that LED zero 
         * is straight ahead, and further LEDs go counterclockwise
         *
         * @param angle counterclockwise robot-centric angle
         * @return led ID closest to that angle
         */
        public int getID(Rotation2d angle) {
            return (int) (((angle.getDegrees()+360)%360)/90) // quadrants ccw
                *(Leds.LED_LEFT-Leds.LED_AHEAD) // multiply by quadrant led count
                + Leds.LED_AHEAD;
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
            fillMap(from,to,(c)->{return color;});
        }

        /** fill angle range based using given function
         *
         * @param from robot-centric angle to start range (counterclockwise)
         * @param to robot-centric angle to end range (must be greater than from)
         * @param mapFunction rotation to color transform
         */
        public void fillMap(Rotation2d from, Rotation2d to, Function<Rotation2d,Color> mapFunction) {
            for (int i = getID(from); i != getID(to) % Leds.LED_COUNT; i++) {
                i %= Leds.LED_COUNT;
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
            return this.order - animation.getOrder();
        }

    }

    Lightstrip strip;
    ArrayList<ScheduledAnimation> animations;
    Timer frameTimer;

    public LightstripSubsystem() {
        strip = new Lightstrip(Leds.PORT,Leds.LED_COUNT);
        frameTimer = new Timer();
        frameTimer.start();

        animations = new ArrayList<ScheduledAnimation>(2);

        animations.add(new ScheduledAnimation(Animations.orbit_demo_simple, 2));
        animations.add(new ScheduledAnimation(Animations.orbit_demo_simple, -2));
    }

    @Override
    public void periodic() {
        if (!frameTimer.advanceIfElapsed(1/Leds.FRAME_RATE)) {
            return;
        }
        Collections.sort(animations);
        Iterator<ScheduledAnimation> i = animations.iterator();

        SmartDashboard.putNumber("ani count",animations.size());
        while (i.hasNext()) {
            ScheduledAnimation animation = i.next();
            animation.tick(strip);
            strip.buffer.setRGB(5,255,148,0);
            strip.flush();
//            if (animation.tick(strip)) {
 //               animations.remove(animation);
  //          }
        }
        strip.buffer.setRGB(5,255,148,0);
        strip.flush();
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
