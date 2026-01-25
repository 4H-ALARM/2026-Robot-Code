// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import org.littletonrobotics.junction.Logger;

/**
 * The ToggleHandler class provides a simple mechanism to toggle a boolean state. It can be used to
 * manage on/off states or any other binary state in a system.
 */
public class ToggleHandler {
  // Variable to store the current state (true or false)
  private boolean state;
  // Variable to store the key to post
  private String key;

  /**
   * Constructor to initialize the ToggleHandler. The state is initialized to false by default.
   *
   * @param akitkey
   */
  public ToggleHandler(String akitkey) {
    this.state = false; // Initial state is set to false (off)
    this.key = akitkey; // key for advantagekit logging
    Logger.recordOutput("Toggles/" + this.key, state);
  }

  /**
   * Method to toggle the state. If the current state is true, it becomes false; if it's false, it
   * becomes true.
   */
  public void toggle() {
    state = !state; // Inverts the current state
    Logger.recordOutput("Toggles/" + this.key, state);
  }

  /**
   * Method to get the current state of the toggle.
   *
   * @return The current state (true or false).
   */
  public boolean get() {
    return state; // Returns the current state
  }
}
