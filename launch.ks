///////////////////////
// ASCENT PARAMETERS //
///////////////////////

// height of the target orbit in meters
set orbitHeight to 100*1000.

// target orbit inclination
// (positive goes to north, 0 means equatorial)
set orbitInclination to 0.

// precision of the target orbit that we need relative to the height
// (if this is too low, it gets hard for the script)
set orbitPrecision to 0.01.

// height of the athmosphere of the body we launch from in meters
set atmoHeight to 70*1000.

// pitch over by a few degrees early to avoid dropping boosters
// directly on the launchpad (value = desired pitch in degrees,
// speed = pitch increment per main loop step)
set initialPitchValue to 5.
set initialPitchIncr to 0.25.

// height and speed where the initial pitch maneuver is performed
// (must both be met to trigger pitching)
set initialHeight to 500.
set initialSpeed to 100.

// seconds to apoapsis when the gravity turn will be started and
// which will be aimed for during the rest of the turn, the time
// increment per main loop step and the max pitch as well as the
// pitch increment per main loop step
set startApoSecs to 30.
set finalApoSecs to 60.
set apoSecsIncr to 0.2.
set gravityMaxPitch to 90.
set gravityPitchIncr to 0.3.

// start throttling down when the apoapsis has reached this fraction
// of the target orbit height but keep at least min throttle during
// the final phase of the gravity turn
set throttleDownGravityStart to 0.5.
set minGravityThrottle to 0.1.

// seconds before reaching apoapsis when the circularization burn
// is started
set circStartEta to 5.

// max pitch off prograde to keep height during the circularization burn
// and pitch increment/decrement per main loop step
set circMaxPitch to 20.
set circPitchIncr to 0.1.

// throttle limit for the circularization burn
set maxCircThrottle to 0.8.

// start throttling down when the periapsis has reached this fraction
// of the target orbit height but keep at least min throttle during the
// circularization burn
set throttleDownCircStart to 0.5.
set minCircThrottle to 0.1.

// fairing deployment altitude
set deploymentAlt to 60*1000.

// set the waiting time between staging attempts
set stagingWaitTime to 2.

// the time step of the main loop (in seconds)
set timeStep to 0.2.


//////////////////////////
// FUNCTION DEFINITIONS //
//////////////////////////

// checks the staging of the rocket by checking if thrust has decreased or is 0
// (called during every iteration of the main loop)
set currentThrust to 0.0.
set autoStaging to false.
function checkStaging {
  if autoStaging = false {
    return.
  }
  if maxthrust < currentThrust or maxthrust < 0.1 {
    printInfo("staging").
    stage.
    wait stagingWaitTime.
  }
  set currentThrust to maxthrust.
}

// deploys the fairings if the fairing deployment altitude has been reached
set fairingsDeployed to false.
function checkFairings {
  if altitude >= deploymentAlt and fairingsDeployed = false {
    printInfo("deploying fairings").
    set fairingsDeployed to true.
    ag1 on.
  }
}

// prints information to the screen and keeps track of the line where the
// last info was printed for a nice display
set infoLine to 8.
function printInfo {
  parameter text.
  print round(missiontime, 1) at (0, infoLine).
  print text at (8, infoLine).
  set infoLine to infoLine + 1.
}

// proceeds to the next phase of the launch sequence, printing the according
// information to the screen
function nextPhase {
  parameter phase.
  printInfo(phase).
  print("                                        ") at (0, 2).
  print("current phase: " + phase) at (0, 2).
  set launchPhase to phase.
}


/////////////////////////////////////////////
// MAIN LOOP CONTAINING THE BUSINESS LOGIC //
/////////////////////////////////////////////

clearscreen.

set launchPhase to "countdown".
set loopActive to true.

set throttleSetpoint to 1.0.
set pitchSetpoint to 0.0.
set headingSetpoint to 90 - orbitInclination.

set apoSecsSetpoint to startApoSecs.
set lastApoSecs to startApoSecs.

// main loop
until loopActive = false {

  // set a timer to measure the computation time of the loop
  set cycleStart to time:seconds.

  checkStaging().
  checkFairings().

  if launchPhase = "countdown" {
    lock throttle to throttleSetpoint.
    lock steering to heading(headingSetpoint, 90 - pitchSetpoint).
    nextPhase("ignition").
  }

  if launchPhase = "ignition" {
    if verticalspeed < 0.1 {
      stage.
      wait stagingWaitTime/2.
    } else {
      set autoStaging to true.
      nextPhase("liftoff").
    }
  }

  if launchPhase = "liftoff" {
    if altitude >= initialHeight and verticalspeed >= initialSpeed {
      nextPhase("pitching over").
    }
  }

  if launchPhase = "pitching over" {
    if pitchSetpoint < initialPitchValue - initialPitchIncr/2 {
      set pitchSetpoint to pitchSetpoint + initialPitchIncr.
    } else {
      nextPhase("pitched ascent").
    }
  }

  if launchPhase = "pitched ascent" {
    if eta:apoapsis >= startApoSecs {
      nextPhase("gravity turn").
    }
  }

  if launchPhase = "gravity turn" {

    print("tta:    " + round(eta:apoapsis, 1) + "   ") at (0, 4).
    print("target: " + round(apoSecsSetpoint, 1) + "   ") at (0, 5).
    print("pitch:  " + round(pitchSetpoint, 1) + "   ") at (0, 6).

    // if the time to apoapsis grows faster than our setpoint plus the
    // defined increment, we pitch down, if the time to apoapsis shrinks
    // below our setpoint, we pitch up
    if eta:apoapsis > apoSecsSetpoint
        and pitchSetpoint < gravityMaxPitch - gravityPitchIncr/2
        and eta:apoapsis > lastApoSecs {
      set pitchSetpoint to pitchSetpoint + gravityPitchIncr.
      print("down") at (20, 6).
    } else if eta:apoapsis < apoSecsSetpoint
        and pitchSetpoint > initialPitchValue + gravityPitchIncr/2
        and eta:apoapsis < lastApoSecs {
      set pitchSetpoint to pitchSetpoint-gravityPitchIncr.
      print("up  ") at (20, 6).
    } else {
      print("keep") at (20, 6).
    }

    // increase our apoapsis setpoint
    if apoSecsSetpoint < finalApoSecs {
      set apoSecsSetpoint to apoSecsSetpoint + apoSecsIncr/2.
    }

    // throttle down if the apoapsis approaches our target as long as time to
    // apoapsis is growing
    if apoapsis / orbitHeight > throttleDownGravityStart
        and throttleSetpoint > minGravityThrottle
        and eta:apoapsis > lastApoSecs {
      set throttleSetpoint to 1 -
          (apoapsis / orbitHeight - throttleDownGravityStart) /
          throttleDownGravityStart * (1 - minGravityThrottle).
    }

    set lastApoSecs to eta:apoapsis.

    if apoapsis > orbitHeight {
      print("                    ") at (0, 4).
      print("                    ") at (0, 5).
      print("                    ") at (0, 6).
      print("    ") at (20, 6).
      set throttleSetpoint to 0.0.
      set pitchSetpoint to 90.0.
      nextPhase("keeping up apoapsis").
    }

  }

  if launchPhase = "keeping up apoapsis" {
    if apoapsis < orbitHeight {
      set throttleSetpoint to throttleSetpoint + 0.01.
    }
    if apoapsis > orbitHeight {
      set throttleSetpoint to 0.
    }
    if altitude > atmoHeight {
      nextPhase("coasting to apoapsis").
    }
  }

  if launchPhase = "coasting to apoapsis" {
    if eta:apoapsis < 600 {
      if warp >= 5 {
        set warp to 4.
      }
    }
    if eta:apoapsis < 300 {
      if warp >= 4 {
        set warp to 3.
      }
    }
    if eta:apoapsis < 60 {
      if warp >=2 {
        set warp to 1.
      }
    }
    if eta:apoapsis < 30 {
      set warp to 0.
    }
    if eta:apoapsis <= circStartEta {
      set lastCircAlt to altitude.
      set throttleSetpoint to maxCircThrottle.
      nextPhase("circularization burn").
    }
  }

  if launchPhase = "circularization burn" {

    // adjust our pitch in order to keep the current altitude
    if altitude > lastCircAlt
        and pitchSetpoint < 90 + circMaxPitch - circPitchIncr/2 {
      if pitchSetpoint < 90 {
        set pitchSetpoint to 90.
      } else {
        set pitchSetpoint to pitchSetpoint + circPitchIncr.
      }
    }
    if altitude < lastCircAlt
        and pitchSetpoint > 90 - circMaxPitch + circPitchIncr/2 {
      if pitchSetpoint > 90 {
        set pitchSetpoint to 90.
      } else {
        set pitchSetpoint to pitchSetpoint - circPitchIncr.
      }
    }

    set lastCircAlt to altitude.

    // throttle down if the periapsis approaches our target
    if periapsis / orbitHeight > throttleDownCircStart
        and throttleSetpoint > minCircThrottle {
      set throttleSetpoint to 1 -
          (periapsis / orbitHeight - throttleDownCircStart) /
          throttleDownCircStart * (1 - minCircThrottle).
    }

    // the orbit is good, we are done
    if periapsis >= orbitHeight - orbitPrecision * orbitHeight {
      set throttleSetpoint to 0.0.
      set loopActive to false.
      nextPhase("launch completed").
    }

  }

  // if we were faster than the desired timeStep, we wait
  if time:seconds - cycleStart < timeStep {
    wait timeStep - (time:seconds - cycleStart).
  }

}

unlock throttle.
unlock steering.
set pilotmainthrottle to 0.0.
sas on.
