I2S library change log
======================

3.0.0_demo
----------

  * CHANGE: Frame master hardcoded to 16kHz

3.0.0
-----

  * REMOVED: Combined I2S and TDM master

2.4.0
-----

  * ADDED: Frame-based I2S slave implementation.
  * CHANGE: AN00162 now uses frame-based I2S master component.

2.3.0
-----

  * ADDED: Configuration option for slave bit clock polarity. This allows
    supporting masters that toggle word clock and data on rising edge of bit
    clock.

2.2.0
-----

  * ADDED: Frame-based I2S master using the new i2s_frame_callback_if. This
    reduces the overhead of an interface call per sample.
  * CHANGE: Reduce number of LR clock ticks needed to synchronise.
  * RESOLVED: Documentation now correctly documents the valid values for FSYNC.
  * RESOLVED: The I2S slave will now lock correctly in both I2S and
    LEFT_JUSTFIED modes. Previously there was a bug that meant LEFT_JUSTFIED
    would not work.

2.1.3
-----

  * CHANGE: Slave mode now includes sync error detection and correction e.g.
    when bit-clock is interrupted

2.1.2
-----

  * RESOLVED: .project file fixes such that example(s) import into xTIMEComposer
    correctly

2.1.1
-----

  * CHANGE: Update to source code license and copyright

2.1.0
-----

  * CHANGE: Input or output ports can now be null, for use when input or
    output-only is required
  * CHANGE: Software license changed to new license

2.0.1
-----

  * CHANGE: Performance improvement to TDM to allow 32x32 operation
  * RESOLVED: Bug fix to initialisation callback timing that could cause I2S
    lock up

2.0.0
-----

  * CHANGE: Major update to API from previous I2S components

  * Changes to dependencies:

    - lib_logging: Added dependency 2.0.0

    - lib_xassert: Added dependency 2.0.0

