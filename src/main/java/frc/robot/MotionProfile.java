package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class MotionProfile {

  /** {@summary} Class describing the motion constraints for the profile to obey */
  public static class MotionConstraints {
    public double maxJerk;
    public double maxAccel;
    public double maxVelocity;
    public double velocityTolerance;

    /**
     * {@summary} constructor for MotionConstraints
     *
     * @param maxJerk max Jerk (derivative of Accel)
     * @param maxAccel max Acceleration to be used in the profile
     * @param maxVelocity max Velocity to be used in the profile
     * @param velocityTolerance max initial velocity to be ignored (allowing us to use S-curve
     *     profile)
     */
    public MotionConstraints(
        double maxJerk, double maxAccel, double maxVelocity, double velocityTolerance) {
      this.maxJerk = maxJerk;
      this.maxAccel = maxAccel;
      this.maxVelocity = maxVelocity;
      this.velocityTolerance = velocityTolerance;
    }
  }

  public static class State {
    public double velocity;
    public double acceleration;
    public double position;

    /**
     * {@summary} constructor for MotionProfile State
     *
     * @param position the position represented by the state
     * @param velocity the velocity represented by the state
     * @param acceleration the acceleration represented by the state
     */
    public State(double position, double velocity, double acceleration) {
      this.acceleration = acceleration;
      this.velocity = velocity;
      this.position = position;
    }

    /**
     * {@summary} constructor for MotionProfile State. Acceleration will be set to 0
     *
     * @param position the position represented by the state
     * @param velocity the velocity represented by the state
     */
    public State(double position, double velocity) {
      this(position, velocity, 0);
    }

    public State(double position) {
      this(position, 0, 0);
    }
  }

  // Timing of the end of various phases, measured in seconds from start of profile
  private class Timings {
    double endRampUpAccel; // done ramping up acceleration, accel is now at max
    double endMaxAccel; // done with max accel phase, ramp down accel
    double endRampDownAccel; // done ramping accel down, now at constant velocity
    double endMaxVelocity; // done with constant velocity phase, start decellerating
    double endRampUpDeccel; // done ramping up decell, now at max decel
    double endMaxDeccel; // done with max decel phase, ramping down to 0
    double endRampDownDeccel; // profile complete
  }

  private class TransitionStates {
    State state1, state2, state3, state4, state5, state6;

    public TransitionStates() {
      double a, v, p, t;
      t = timings.endRampUpAccel;
      a = constraints.maxJerk * timings.endRampUpAccel;
      v = constraints.maxJerk * Math.pow(t, 2) / 2;
      p = initial.position + constraints.maxJerk * Math.pow(t, 3) / 6;
      state1 = new State(p, v, a);

      t = timings.endMaxAccel - timings.endRampUpAccel;
      a = state1.acceleration;
      v = state1.velocity + a * t;
      p = state1.position + state1.velocity * t + a * Math.pow(t, 2) / 2;
      state2 = new State(p, v, a);

      t = timings.endRampDownAccel - timings.endMaxAccel;
      a = state2.acceleration - constraints.maxJerk * t;
      v = state2.velocity + state2.acceleration * t - constraints.maxJerk * Math.pow(t, 2) / 2;
      p =
          state2.position
              + state2.velocity * t
              + state2.acceleration * Math.pow(t, 2) / 2
              - constraints.maxJerk * Math.pow(t, 3) / 6;
      state3 = new State(p, v, a);

      t = timings.endMaxVelocity - timings.endRampDownAccel;
      a = 0;
      v = state3.velocity;
      p = state3.position + state3.velocity * t;
      state4 = new State(p, v, a);

      t = timings.endRampUpDeccel - timings.endMaxVelocity;
      a = -state1.acceleration;
      v = state4.velocity - constraints.maxJerk * Math.pow(t, 2) / 2;
      p = state4.position + state4.velocity * t - constraints.maxJerk * Math.pow(t, 3) / 6;
      state5 = new State(p, v, a);

      t = timings.endMaxDeccel - timings.endRampUpDeccel;
      a = state5.acceleration;
      v = state5.velocity + a * t;
      p = state5.position + state5.velocity * t + state5.acceleration * Math.pow(t, 2) / 2;
      state6 = new State(p, v, a);
    }
  }

  // Enum representing types of profiles. Auto will choose SCurve if initial velocity is low enough,
  // otherwise trapezoid
  public enum ProfileType {
    TRAPEZOID,
    SCURVE,
    AUTO
  };

  private int direction;
  private MotionConstraints constraints;
  private State goal, initial;
  private Timings timings;
  private Timer timer;

  double lastTime = 0;
  private TransitionStates states;
  private ProfileType type;

  public MotionProfile(State goal, State current, MotionConstraints constraints, ProfileType type) {
    this.constraints = constraints;

    // It's easier to always calculate the profile the same way, so if the goal is less than
    // current, flip them and we'll flip the result later
    direction = shouldFlipProfile(current, goal) ? -1 : 1;
    this.goal = direct(goal);
    this.initial = direct(current);
    timings = new Timings();
    timer = new Timer();

    // Choose a profile type and calculate it. This will populate the Timings object and
    // TransitionStates object as necessary
    if (current.velocity > constraints.velocityTolerance
        || goal.velocity != 0
        || type == ProfileType.TRAPEZOID) {
      this.type = ProfileType.TRAPEZOID;
      calculateTrapezoid();
    } else {
      this.type = ProfileType.SCURVE;
      calculateSCurve();
    }
  }

  public State sample() {
    // Start timer, this no-ops if the timer is already running so will start it on the first call
    // to sample()
    timer.start();
    State retval = new State(0, 0, 0);

    // If we're following an S-curve use equations described in paper linked in calculateSCurve
    if (type == ProfileType.SCURVE) {
      double time = timer.get();
      if (time < timings.endRampUpAccel) {
        // ramp up accel, Jerk = Max
        retval.acceleration = Math.min(time * constraints.maxJerk, constraints.maxAccel);
        retval.velocity = constraints.maxJerk * Math.pow(time, 2) / 2;
        retval.position = initial.position + constraints.maxJerk * Math.pow(time, 3) / 6;
      } else if (time < timings.endMaxAccel) {
        // constant accel, Jerk = 0
        retval.acceleration = states.state1.acceleration;
        double t = time - timings.endRampUpAccel;
        retval.velocity = states.state1.velocity + constraints.maxAccel * t;
        retval.position =
            states.state1.position
                + states.state1.velocity * t
                + constraints.maxAccel * Math.pow(t, 2) / 2;
      } else if (time < timings.endRampDownAccel) {
        // ramp down accel, Jerk = -Max
        double t = time - timings.endMaxAccel;
        retval.acceleration = Math.max(states.state2.acceleration - t * constraints.maxJerk, 0);
        retval.velocity =
            states.state2.velocity
                + states.state2.acceleration * t
                - constraints.maxJerk * Math.pow(t, 2) / 2;
        retval.position =
            states.state2.position
                + states.state2.velocity * t
                + states.state2.acceleration * Math.pow(t, 2) / 2
                - constraints.maxJerk * Math.pow(t, 3) / 6;
      } else if (time < timings.endMaxVelocity) {
        // max vel cruise
        double t = time - timings.endRampDownAccel;
        retval.acceleration = 0;
        retval.velocity = states.state3.velocity;
        retval.position = states.state3.position + states.state3.velocity * t;
      } else if (time < timings.endRampUpDeccel) {
        // ramp up decceleration. Jerk = -Max
        double t = time - timings.endMaxVelocity;
        retval.acceleration = -constraints.maxJerk * t;
        retval.velocity = states.state4.velocity - constraints.maxJerk * Math.pow(t, 2) / 2;
        ;
        retval.position =
            states.state4.position
                + states.state4.velocity * t
                - constraints.maxJerk * Math.pow(t, 3) / 6;
      } else if (time < timings.endMaxDeccel) {
        // max decceleration. Jerk = 0
        double t = time - timings.endRampUpDeccel;
        retval.acceleration = states.state5.acceleration;
        retval.velocity = states.state5.velocity + states.state5.acceleration * t;
        retval.position =
            states.state5.position
                + states.state5.velocity * t
                + states.state5.acceleration * Math.pow(t, 2) / 2;
      } else if (time < timings.endRampDownDeccel) {
        // ramping down deccel. Jerk = Max
        double t = time - timings.endMaxDeccel;
        retval.acceleration = states.state6.acceleration + constraints.maxJerk * t;
        retval.velocity =
            states.state6.velocity
                + states.state6.acceleration * t
                + constraints.maxJerk * Math.pow(t, 2) / 2;
        retval.position =
            states.state6.position
                + states.state6.velocity * t
                + states.state6.acceleration * Math.pow(t, 2) / 2
                + constraints.maxJerk * Math.pow(t, 3) / 6;
      } else {
        // profile complete, just return goal
        return direct(goal);
      }
      return direct(retval);
    }
    // Otherwise we're following a trapezoid, implementation stolen from WPILib class
    else {
      retval.velocity = initial.velocity;
      retval.position = initial.position;
      double t = timer.get();
      if (t < timings.endMaxAccel) {
        retval.velocity += t * constraints.maxAccel;
        retval.position += (initial.velocity + t * constraints.maxAccel / 2.0) * t;
        retval.acceleration = constraints.maxAccel;
      } else if (t < timings.endMaxVelocity) {
        retval.acceleration = 0;
        retval.velocity = constraints.maxVelocity;
        retval.position +=
            (initial.velocity + timings.endMaxAccel * constraints.maxAccel / 2.0)
                    * timings.endMaxAccel
                + constraints.maxVelocity * (t - timings.endMaxAccel);
      } else if (t <= timings.endMaxDeccel) {
        retval.acceleration = -constraints.maxAccel;
        retval.velocity = goal.velocity + (timings.endMaxDeccel - t) * constraints.maxAccel;
        double timeLeft = timings.endMaxDeccel - t;
        retval.position =
            goal.position - (goal.velocity + timeLeft * constraints.maxAccel / 2.0) * timeLeft;
      } else {
        retval = goal;
      }

      return direct(retval);
    }
  }

  private void calculateTrapezoid() {
    if (initial.velocity > constraints.maxVelocity) {
      initial.velocity = constraints.maxVelocity;
    }

    // Deal with a possibly truncated motion profile (with nonzero initial or
    // final velocity) by calculating the parameters as if the profile began and
    // ended at zero velocity
    double cutoffBegin = initial.velocity / constraints.maxAccel;
    double cutoffDistBegin = cutoffBegin * cutoffBegin * constraints.maxAccel / 2.0;

    double cutoffEnd = goal.velocity / constraints.maxAccel;
    double cutoffDistEnd = cutoffEnd * cutoffEnd * constraints.maxAccel / 2.0;

    // Now we can calculate the parameters as if it was a full trapezoid instead
    // of a truncated one

    double fullTrapezoidDist = cutoffDistBegin + (goal.position - initial.position) + cutoffDistEnd;
    double accelerationTime = constraints.maxVelocity / constraints.maxAccel;

    double fullSpeedDist =
        fullTrapezoidDist - accelerationTime * accelerationTime * constraints.maxAccel;

    // Handle the case where the profile never reaches full speed
    if (fullSpeedDist < 0) {
      accelerationTime = Math.sqrt(fullTrapezoidDist / constraints.maxAccel);
      fullSpeedDist = 0;
    }
    timings.endRampUpAccel = 0;
    timings.endMaxAccel = accelerationTime - cutoffBegin;
    timings.endRampDownAccel = timings.endMaxAccel;
    timings.endMaxVelocity = timings.endMaxAccel + fullSpeedDist / constraints.maxVelocity;
    timings.endRampUpDeccel = timings.endMaxVelocity;
    timings.endMaxDeccel = timings.endMaxVelocity + accelerationTime - cutoffEnd;
    timings.endRampDownDeccel = timings.endMaxDeccel;
  }

  private void calculateSCurve() {
    // calculation algorithm based on
    // https://www.researchgate.net/publication/348383330_Mathematics_for_Real-Time_S-Curve_Profile_Generator
    double va = Math.pow(constraints.maxAccel, 2) / constraints.maxJerk;
    double sa = 2 * Math.pow(constraints.maxAccel, 3) / Math.pow(constraints.maxJerk, 2);
    double sv, tj, ta, tv;
    double s = goal.position - initial.position;
    if (constraints.maxVelocity * constraints.maxJerk < Math.pow(constraints.maxAccel, 2)) {
      sv = 2 * constraints.maxVelocity * Math.sqrt(constraints.maxVelocity / constraints.maxJerk);
    } else {
      sv =
          constraints.maxVelocity
              * ((constraints.maxVelocity / constraints.maxAccel)
                  + (constraints.maxAccel / constraints.maxJerk));
    }
    if ((constraints.maxVelocity < va && s > sa)
        || (constraints.maxVelocity < va && s < sa && s > sv)) {
      // Type A or C.1, eqs 72,75,76
      tj = Math.sqrt(constraints.maxVelocity / constraints.maxJerk);
      tv = s / constraints.maxVelocity;
      ta = tj;
    } else if ((constraints.maxVelocity > va && s < sa)
        || (constraints.maxVelocity < va && s < sa)) {
      // Type B or C.2, eqs 66,67,68
      tj = Math.pow(s / (2 * constraints.maxJerk), 1 / 3.);
      ta = tj;
      tv = 2 * tj;
    } else if (s > sv) {
      // Type D.1 eqs 77,78,79
      tj = constraints.maxAccel / constraints.maxJerk;
      ta = constraints.maxVelocity / constraints.maxAccel;
      tv = s / constraints.maxVelocity;
    } else {
      // Type D.2 eqs 80,88,89
      tj = constraints.maxAccel / constraints.maxJerk;
      ta =
          .5
              * (Math.sqrt(
                      (4 * s * Math.pow(constraints.maxJerk, 2) + Math.pow(constraints.maxAccel, 3))
                          / (constraints.maxAccel * Math.pow(constraints.maxJerk, 2)))
                  - constraints.maxAccel / constraints.maxJerk);
      tv = ta + tj;
    }

    timings.endRampUpAccel = tj;
    timings.endMaxAccel = ta;
    timings.endRampDownAccel = tj + ta;
    timings.endMaxVelocity = tv;
    timings.endRampUpDeccel = tv + tj;
    timings.endMaxDeccel = tv + ta;
    timings.endRampDownDeccel = tv + ta + tj;
    states = new TransitionStates();
  }

  /**
   * Returns true if the profile inverted.
   *
   * <p>The profile is inverted if goal position is less than the initial position.
   *
   * @param initial The initial state (usually the current state).
   * @param goal The desired state when the profile is complete.
   */
  private static boolean shouldFlipProfile(State initial, State goal) {
    return initial.position > goal.position;
  }

  // Flip the sign of the velocity and position if the profile is inverted
  private State direct(State in) {
    State result = new State(in.position, in.velocity, in.acceleration);
    result.position = result.position * direction;
    result.velocity = result.velocity * direction;
    result.acceleration = result.acceleration * direction;
    return result;
  }

  public boolean isFinished() {
    return timer.get() > 0 && timer.get() > timings.endRampDownDeccel;
  }
}
