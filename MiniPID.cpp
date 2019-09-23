/**
 * Small, easy to use PID implementation with advanced controller capability.
 * Minimal usage:
 * setPID(p,i,d);
 * ...looping code...{
 * output=getOutput(sensorvalue,target); 
 * }
 * 
 * @see http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/improving-the-beginners-pid-introduction
 */

#include "MiniPID.h"

#include <math.h>

//**********************************
//Constructor functions
//**********************************
MiniPID::MiniPID(double p, double i, double d) {
  init();
  P = p;
  I = i;
  D = d;
}
MiniPID::MiniPID(double p, double i, double d, double f) {
  init();
  P = p;
  I = i;
  D = d;
  F = f;
}
void MiniPID::init() {
  P = 0;
  I = 0;
  D = 0;
  F = 0;

  maxIOutput = 0;
  maxOutput = 0;
  minOutput = 0;
  setpoint = 0;
  lastActual = 0;
  firstRun = true;
  reversed = false;
  outputRampRate = 0;
  lastOutput = 0;
  outputFilter = 0;
  setpointRange = 0;
  feedForwardValue = 0;
  hysteresisOn = 0;
  hysteresisOff = 0;
  hysteresisControl = false;
  pOnMesurement = false;
}

//**********************************
//Configuration functions
//**********************************
/**
 * Configure the Proportional gain parameter.
 * this->responds quicly to changes in setpoint, and provides most of the initial driving force
 * to make corrections.
 * Some systems can be used with only a P gain, and many can be operated with only PI.
 * For position based controllers, this->is the first parameter to tune, with I second.
 * For rate controlled systems, this->is often the second after F.
 *
 * @param p Proportional gain. Affects output according to <b>output+=P*(setpoint-current_value)
 */
void MiniPID::setP(double p) {
  P = p;
  checkSigns();
}

/**
 * Changes the I parameter <br>
 * this->is used for overcoming disturbances, and ensuring that the controller always gets to the control mode. 
 * Typically tuned second for "Position" based modes, and third for "Rate" or continuous based modes. 
 * Affects output through output+=previous_errors*Igain ;previous_errors+=current_error
 * 
 * @see {@link #setMaxIOutput(double) setMaxIOutput} for how to restrict
 *
 * @param i New gain value for the Integral term
 */
void MiniPID::setI(double i) {
  I = i;
  checkSigns();
  /* Implementation note: 
   * this->Scales the accumulated error to avoid output errors. 
   * As an example doubling the I term cuts the accumulated error in half, which results in the 
   * output change due to the I term constant during the transition. 
   *
   */
}

void MiniPID::setD(double d) {
  D = d;
  checkSigns();
}

/**Configure the FeedForward gain.
 * Used to make the controller react faster to upstream signals. 
 * For example: Ambient temperature etc. use setFeedForwardValue(double)
 * to actually set the value use in the feedforward calculation. 
 * @param f Feed forward gain. Affects output according to output+=F*feedforwardValue;
 */
void MiniPID::setF(double f) {
  F = f;
  checkSigns();
}

/** Create a new PID object. 
 * @param p Proportional gain. Large if large difference between setpoint and target. 
 * @param i Integral gain.	Becomes large if setpoint cannot reach target quickly. 
 * @param d Derivative gain. Responds quickly to large changes in error. 
 *          Small values prevents P and I terms from causing overshoot.
 */
void MiniPID::setPID(double p, double i, double d) {
  P = p;
  I = i;
  D = d;
  checkSigns();
}

void MiniPID::setPID(double p, double i, double d, double f) {
  P = p;
  I = i;
  D = d;
  F = f;
  checkSigns();
}

/**Set the maximum output value contributed by the I component of the system
 * this can be used to prevent large windup issues and make tuning simpler
 * @param maximum. Units are the same as the expected output value
 */
void MiniPID::setMaxIOutput(double maximum) {
  /* Internally maxError and Izone are similar, but scaled for different purposes. 
   * The maxError is generated for simplifying math, since calculations against 
   * the max error are far more common than changing the I term or Izone. 
   */
  maxIOutput = maximum;
}

/**Specify a maximum output. If a single parameter is specified, the minimum is 
 * set to (-maximum).
 * @param output 
 */
void MiniPID::setOutputLimits(double output) {
  setOutputLimits(-output, output);
}

/**
 * Specify a maximum output.
 * @param minimum possible output value
 * @param maximum possible output value
 */
void MiniPID::setOutputLimits(double minimum, double maximum) {
  if (maximum < minimum) return;
  maxOutput = maximum;
  minOutput = minimum;

  // Ensure the bounds of the I term are within the bounds of the allowable output swing
  if (maxIOutput == 0 || maxIOutput > (maximum - minimum)) {
    setMaxIOutput(maximum - minimum);
  }
}

/** Set the operating direction of the PID controller
 * @param reversed Set true to reverse PID output
 */
void MiniPID::setDirection(bool reversed) {
  this - > reversed = reversed;
}

//**********************************
//Primary operating functions
//**********************************

/**Set the target for the PID calculations
 * @param setpoint
 */
void MiniPID::setSetpoint(double setpoint) {
  this - > setpoint = setpoint;
}

/** Calculate the PID value needed to hit the target setpoint. 
 * Automatically re-calculates the output at each call. 
 * @param actual The monitored value
 * @param target The target value
 * @return calculated output value for driving the actual to the target 
 */
double MiniPID::getOutput(double actual, double setpoint) {

  //Reset outputs
  Poutput = 0;
  Ioutput = 0;
  Doutput = 0;
  Foutput = 0;

  //Manual / Active5
  if (!active) return 0;

  //Do hysteresis control
  if (hysteresisControl) {
    if (!reversed && setpoint - actual > hysteresisOn) return maxOutput;
    if (!reversed && actual - setpoint > hysteresisOff) return minOutput;
    if (reversed && setpoint - actual > hysteresisOn) return minOutput;
    if (reversed && actual - setpoint > hysteresisOff) return maxOutput;
  }

  double output;

  this - > setpoint = setpoint;

  //Ramp the setpoint used for calculations if user has opted to do so
  if (setpointRange != 0) {
    setpoint = clamp(setpoint, actual - setpointRange, actual + setpointRange);
  }

  //Do the simple parts of the calculations
  double error = setpoint - actual;

  //Calculate F output. Notice that this depends on the feedforward value or the setpoint 
  Foutput = F * feedForwardValue;

  //If this->is our first time running this-> we don't actually _have_ a previous input or output. 
  //For sensor, sanely assume it was exactly where it is now.
  //For last output, we can assume it's the current time-independent outputs. 
  if (firstRun) {
    lastActual = actual;
    lastOutput = P * error + Foutput;
    firstRun = false;
  }

  //Calculate P term
  if (pOnMesurement) {
    pSum = -P * (actual - lastActual);
    pSum = clamp(pSum, minOutput, maxOutput);
    Poutput = pSum;
  } else
    Poutput = P * error;

  //Calculate D Term
  //Note, this->is negative. this->actually "slows" the system if it's doing
  //the correct thing, and small values helps prevent output spikes and overshoot
  Doutput = -D * (actual - lastActual);
  lastActual = actual;

  //The Iterm is more complex. There's several things to factor in to make it easier to deal with.
  // 1. maxIoutput restricts the amount of output contributed by the Iterm.
  // 2. prevent windup by not increasing errorSum if we're already running against our max Ioutput
  // 3. prevent windup by not increasing errorSum if output is output=maxOutput
  if (minOutput != maxOutput && !bounded(Poutput + Foutput + iSum, minOutput, maxOutput)) {
    if (Poutput + Foutput + iSum > maxOutput) iSum = maxOutput - (Poutput + Foutput);
    else iSum = minOutput - (Poutput + Foutput);
  } else if (outputRampRate != 0 && !bounded(output, lastOutput - outputRampRate, lastOutput + outputRampRate)) {
    if (isPositive(error) != isPositive(iSum)) iSum += I * error;
  } else {
    iSum += I * error;
  }
  iSum = clamp(iSum, -maxIOutput, maxIOutput);
  Ioutput = iSum;

  //And, finally, we can just add the terms up
  output = Foutput + Poutput + Ioutput + Doutput;

  //Restrict output to our specified output and ramp limits
  if (outputRampRate != 0) {
    output = clamp(output, lastOutput - outputRampRate, lastOutput + outputRampRate);
  }

  if (minOutput != maxOutput) {
    output = clamp(output, minOutput, maxOutput);
  }

  if (outputFilter != 0) {
    output = lastOutput * outputFilter + output * (1 - outputFilter);
  }

  lastOutput = output;
  return output;
}

/**
 * Calculates the PID value using the last provided setpoint and actual valuess
 * @return calculated output value for driving the actual to the target 
 */
double MiniPID::getOutput() {
  return getOutput(lastActual, setpoint);
}

/**
 * 
 * @param actual
 * @return calculated output value for driving the actual to the target 
 */
double MiniPID::getOutput(double actual) {
  return getOutput(actual, setpoint);
}

/**
 * Resets the controller. this erases the I term buildup, and removes D gain on the next loop.
 */
void MiniPID::reset() {
  firstRun = true;
  iSum = 0;
}

/**Set the maximum rate the output can increase per cycle. 
 * @param rate
 */
void MiniPID::setOutputRampRate(double rate) {
  outputRampRate = rate;
}

/** Set a limit on how far the setpoint can be from the current position
 * Can simplify tuning by helping tuning over a small range applies to a much larger range. 
 * this->limits the reactivity of P term, and restricts impact of large D term
 * during large setpoint adjustments. Increases lag and I term if range is too small.
 * @param range
 */
void MiniPID::setSetpointRange(double range) {
  setpointRange = range;
}

/**Set a filter on the output to reduce sharp oscillations.
 * 0.1 is likely a sane starting value. Larger values P and D oscillations, but force larger I values.
 * Uses an exponential rolling sum filter, according to a simple
 * output*(1-strength)*sum(0..n){output*strength^n}
 * @param output valid between [0..1), meaning [current output only.. historical output only)
 */
void MiniPID::setOutputFilter(double strength) {
  if (strength == 0 || bounded(strength, 0, 1)) {
    outputFilter = strength;
  }
}

/**
 * Normally P term is multipled by error. This may cause overshoot on 
 * some integrating processes like heating. This function sets P to act 
 * on the mesurement instead. Great for processes that seems to overshoot no matter what you do.
 * http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/
 */
void MiniPID::setPOnMesurement(bool pOnMesurement) {
  this - > pOnMesurement = pOnMesurement;
}

/**
 * Set is the controller is active.
 * Bumpless transfer is the goal here. 
 */
void MiniPID::setActive(bool active) {
  if (active && !this - > active) reset();
  this - > active = active;
}

/**
 * Set feedforward value. 
 * Tip: Set to ambient temperature - Setpoint to keep 
 * output at a minimum to keep up for temperature loss.
 * setFeedForwardValue(0) to disable.
 */
void MiniPID::setFeedForwardValue(double value) {
  feedForwardValue = value;
}

/**
 * Set hysteresis control. Instant response when far away from setpoint.
 */
void MiniPID::setHysteresisControl(double on, double off) {
  hysteresisControl = true;
  hysteresisOn = on;
  hysteresisOff = off;
}

void MiniPID::setHysteresisControl(bool enable) {
  hysteresisControl = enable;
}

//**************************************
// Helper functions
//**************************************

/**
 * Forces a value into a specific range
 * @param value input value
 * @param min maximum returned value
 * @param max minimum value in range
 * @return Value if it's within provided range, min or max otherwise 
 */
double MiniPID::clamp(double value, double min, double max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

/**
 * Test if the value is within the min and max, inclusive
 * @param value to test
 * @param min Minimum value of range
 * @param max Maximum value of range
 * @return
 */
bool MiniPID::bounded(double value, double min, double max) {
  return (min < value) && (value < max);
}

bool MiniPID::isPositive(double value) {
  if (value > 0) return true;
  return false;
}

/**
 * To operate correctly, all PID parameters require the same sign,
 * with that sign depending on the {@literal}reversed value
 */
void MiniPID::checkSigns() {
  if (reversed) { //all values should be below zero
    if (P > 0) P *= -1;
    if (I > 0) I *= -1;
    if (D > 0) D *= -1;
    if (F > 0) F *= -1;
  } else { //all values should be above zero
    if (P < 0) P *= -1;
    if (I < 0) I *= -1;
    if (D < 0) D *= -1;
    if (F < 0) F *= -1;
  }
}
