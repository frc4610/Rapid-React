package frc.robot.utils.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;

import java.util.TreeMap;

// This is TimeInterpolatableBuffer but modified

public class InterpolatingTreeMap<T> extends TreeMap<Double, T> {
  private final double m_historySize;
  private final InterpolateFunction<T> m_interpolatingFunc;

  private InterpolatingTreeMap(
      InterpolateFunction<T> interpolateFunction, double historySizeSeconds) {
    this.m_historySize = historySizeSeconds;
    this.m_interpolatingFunc = interpolateFunction;
  }

  /**
   * Create a new InterpolatingTreeMap.
   *
   * @param interpolateFunction The function used to interpolate between values.
   * @param historySizeSeconds  The history size of the buffer.
   * @param <T>                 The type of data to store in the buffer.
   * @return The new InterpolatingTreeMap.
   */
  public static <T> InterpolatingTreeMap<T> createBuffer(
      InterpolateFunction<T> interpolateFunction, double historySizeSeconds) {
    return new InterpolatingTreeMap<>(interpolateFunction, historySizeSeconds);
  }

  /**
   * Create a new InterpolatingTreeMap that stores a given subclass of
   * {@link Interpolatable}.
   *
   * @param historySizeSeconds The history size of the buffer.
   * @param <T>                The type of {@link Interpolatable} to store in the
   *                           buffer.
   * @return The new InterpolatingTreeMap.
   */
  public static <T extends Interpolatable<T>> InterpolatingTreeMap<T> createBuffer(
      double historySizeSeconds) {
    return new InterpolatingTreeMap<>(Interpolatable::interpolate, historySizeSeconds);
  }

  /**
   * Create a new InterpolatingTreeMap to store Double values.
   *
   * @param historySizeSeconds The history size of the buffer.
   * @return The new InterpolatingTreeMap.
   */
  public static InterpolatingTreeMap<Double> createDoubleBuffer(double historySizeSeconds) {
    return new InterpolatingTreeMap<>(MathUtil::interpolate, historySizeSeconds);
  }

  /**
   * Add a sample to the buffer.
   *
   * @param timeSeconds The timestamp of the sample.
   * @param sample      The sample object.
   */
  public void addSample(double timeSeconds, T sample) {
    cleanUp(timeSeconds);
    super.put(timeSeconds, sample);
  }

  /**
   * Removes samples older than our current history size.
   *
   * @param time The current timestamp.
   */
  private void cleanUp(double time) {
    while (!super.isEmpty()) {
      var entry = super.firstEntry();
      if (time - entry.getKey() >= m_historySize) {
        super.remove(entry.getKey());
      } else {
        return;
      }
    }
  }

  /** Clear all old samples. */
  public void clear() {
    super.clear();
  }

  /**
   * Sample the buffer at the given time. If the buffer is empty, this will return
   * null.
   *
   * @param timeSeconds The time at which to sample.
   * @return The interpolated value at that timestamp. Might be null.
   */
  @SuppressWarnings("UnnecessaryParentheses")
  public T getSample(double timeSeconds) {
    if (super.isEmpty()) {
      return null;
    }

    // Special case for when the requested time is the same as a sample
    var nowEntry = super.get(timeSeconds);
    if (nowEntry != null) {
      return nowEntry;
    }

    var topBound = super.ceilingEntry(timeSeconds);
    var bottomBound = super.floorEntry(timeSeconds);

    // Return null if neither sample exists, and the opposite bound if the other is
    // null
    if (topBound == null && bottomBound == null) {
      return null;
    } else if (topBound == null) {
      return bottomBound.getValue();
    } else if (bottomBound == null) {
      return topBound.getValue();
    } else {
      // Otherwise, interpolate. Because T is between [0, 1], we want the ratio of
      // (the difference
      // between the current time and bottom bound) and (the difference between top
      // and bottom
      // bounds).
      return m_interpolatingFunc.interpolate(
          bottomBound.getValue(),
          topBound.getValue(),
          ((timeSeconds - bottomBound.getKey()) / (topBound.getKey() - bottomBound.getKey())));
    }
  }

  public interface InterpolateFunction<T> {
    /**
     * Return the interpolated value. This object is assumed to be the starting
     * position, or lower
     * bound.
     *
     * @param start The lower bound, or start.
     * @param end   The upper bound, or end.
     * @param t     How far between the lower and upper bound we are. This should be
     *              bounded in [0, 1].
     * @return The interpolated value.
     */
    @SuppressWarnings("ParameterName")
    T interpolate(T start, T end, double t);
  }
}