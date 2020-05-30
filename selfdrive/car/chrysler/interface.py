#!/usr/bin/env python3

import time
from common.basedir import BASEDIR
import logging
import logging.handlers

from cereal import car
from selfdrive.controls.lib.drive_helpers import EventTypes as ET, create_event
from selfdrive.car.chrysler.values import Ecu, ECU_FINGERPRINT, CAR, FINGERPRINTS
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, is_ecu_disconnected, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.config import Conversions as CV

ButtonType = car.CarState.ButtonEvent.Type

# define Logger class to implement logging functionality
class CarInterfaceLogger(CarInterfaceBase):
  def __init__(self, name, CP, CarController, CarState):
    CarInterfaceBase.__init__(self, CP, CarController, CarState)
    self.name = name
    self.logger = logging.getLogger(name)
    h = logging.handlers.RotatingFileHandler(BASEDIR+"/"+str(name)+'.log', 'a', 10*1024*1024, 5) 
    f = logging.Formatter('%(asctime)s %(processName)-10s %(name)s %(levelname)-8s %(message)s')
    h.setFormatter(f)
    self.logger.addHandler(h)
    self.logger.setLevel(logging.CRITICAL) # set to logging.DEBUG to enable logging
    # self.logger.setLevel(logging.INFO) # set to logging.CRITICAL to disable logging
    self.delayLog = time.time()

class CarInterface(CarInterfaceLogger):
  def __init__(self, CP, CarController, CarState):
    CarInterfaceLogger.__init__(self, "ChryslerCarInterface", CP, CarController, CarState)

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 3.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=[]):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint, has_relay)
    ret.carName = "chrysler"
    ret.safetyModel = car.CarParams.SafetyModel.chrysler

    # Chrysler port is a community feature, since we don't own one to test
    ret.communityFeature = True

    # use ACC to control the speed
    ret.enableACCAccelControl = True

    # Speed conversion:              20, 45 mph
    ret.wheelbase = 3.089  # in meters for Pacifica Hybrid 2017
    ret.steerRatio = 16.2 # Pacifica Hybrid 2017
    ret.mass = 2858. + STD_CARGO_KG  # kg curb weight Pacifica Hybrid 2017
    ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [[9., 20.], [9., 20.]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.15,0.30], [0.03,0.05]]
    ret.lateralTuning.pid.kf = 0.00006   # full torque for 10 deg at 80mph means 0.00007818594
    ret.steerActuatorDelay = 0.1
    ret.steerRateCost = 0.7
    ret.steerLimitTimer = 0.4

    if candidate in (CAR.JEEP_CHEROKEE, CAR.JEEP_CHEROKEE_2019):
      ret.wheelbase = 2.91  # in meters
      ret.steerRatio = 12.7
      ret.steerActuatorDelay = 0.2  # in seconds

    ret.centerToFront = ret.wheelbase * 0.44

    ret.minSteerSpeed = 3.8  # m/s
    if candidate in (CAR.PACIFICA_2019_HYBRID, CAR.PACIFICA_2020, CAR.JEEP_CHEROKEE_2019):
      # TODO allow 2019 cars to steer down to 13 m/s if already engaged.
      ret.minSteerSpeed = 17.5  # m/s 17 on the way up, 13 on the way down once engaged.

    # starting with reasonable value for civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront)

    ret.enableCamera = is_ecu_disconnected(fingerprint[0], FINGERPRINTS, ECU_FINGERPRINT, candidate, Ecu.fwdCamera) or has_relay
    print("ECU Camera Simulated: {0}".format(ret.enableCamera))

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # ******************* do can recv *******************
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam)

    ret.canValid = self.cp.can_valid and self.cp_cam.can_valid

    # speeds
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # accel/decel button presses
    buttonEvents = []
    if self.CS.accelCruiseButtonChanged:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.accelCruise
      be.pressed = self.CS.accelCruiseButton
      buttonEvents.append(be)
    if self.CS.decelCruiseButtonChanged:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.decelCruise
      be.pressed = self.CS.decelCruiseButton
      buttonEvents.append(be)
    if self.CS.resumeCruiseButtonChanged:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.resumeCruise
      be.pressed = self.CS.resumeCruiseButton
      buttonEvents.append(be)
    ret.buttonEvents = buttonEvents

    # events
    events = self.create_common_events(ret, extra_gears=[car.CarState.GearShifter.low], gas_resume_speed=2.)

    if ret.vEgo < self.CP.minSteerSpeed:
      events.append(create_event('belowSteerSpeed', [ET.WARNING]))

    ret.events = events

    # copy back carState packet to CS
    self.CS.out = ret.as_reader()

    return self.CS.out

  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c):

    if (self.CS.frame == -1 or self.CS.buttonCounter == -1):
      return [] # if we haven't seen a frame 220 or 23b, then do not update.

    if time.time() - self.delayLog > 5 or self.CS.out.buttonEvents:
      self.delayLog = time.time()
      self.logger.info("******************************")
      self.logger.debug("CS: %s" % str(self.CS.out))
      self.logger.debug("******************************")
      self.logger.debug("CC: %s" % str(c))
      self.logger.debug("******************************")
      self.logger.info("Cruise enabled  : %s" % str(self.CS.out.cruiseState.enabled))
      self.logger.info("Cruise Set Speed: %s" % str(self.CS.out.cruiseState.speed * CV.MS_TO_MPH))
      self.logger.info("Current Speed   : %s" % str(self.CS.out.vEgo * CV.MS_TO_MPH))
      self.logger.info("Target Speed    : %s" % str(c.cruiseControl.targetSpeed * CV.MS_TO_MPH))      
    can_sends = self.CC.update(c.enabled, self.CS, c.actuators, c.cruiseControl.cancel, c.hudControl.visualAlert, self.CS.out.cruiseState.speed, c.cruiseControl.targetSpeed)

    return can_sends