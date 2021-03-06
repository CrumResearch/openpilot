from cereal import car
from common.numpy_fast import clip, interp
from selfdrive.config import Conversions as CV

# kph
V_CRUISE_MAX = 144
V_CRUISE_MIN = 8
V_CRUISE_DELTA = 8
V_CRUISE_ENABLE_MIN = 40


class MPC_COST_LAT:
  PATH = 1.0
  LANE = 3.0
  HEADING = 1.0
  STEER_RATE = 1.0


class MPC_COST_LONG:
  TTC = 5.0
  DISTANCE = 0.1
  ACCELERATION = 10.0
  JERK = 20.0


class EventTypes:
  ENABLE = 'enable'
  PRE_ENABLE = 'preEnable'
  NO_ENTRY = 'noEntry'
  WARNING = 'warning'
  USER_DISABLE = 'userDisable'
  SOFT_DISABLE = 'softDisable'
  IMMEDIATE_DISABLE = 'immediateDisable'
  PERMANENT = 'permanent'


def create_event(name, types):
  event = car.CarEvent.new_message()
  event.name = name
  for t in types:
    setattr(event, t, True)
  return event


def get_events(events, types):
  out = []
  for e in events:
    for t in types:
      if getattr(e, t):
        out.append(e.name)
  return out


def rate_limit(new_value, last_value, dw_step, up_step):
  return clip(new_value, last_value + dw_step, last_value + up_step)


def get_steer_max(CP, v_ego):
  return interp(v_ego, CP.steerMaxBP, CP.steerMaxV)


def update_v_cruise(v_cruise_kph, buttonEvents, enabled, enableACCAccelControl):
  cruiseMin = V_CRUISE_MIN if enableACCAccelControl else 32

  # handle button presses. TODO: this should be in state_control, but a decelCruise press
  # would have the effect of both enabling and changing speed is checked after the state transition
  for b in buttonEvents:
    if enabled and not b.pressed:
      if b.type == "accelCruise":
        v_cruise_kph += V_CRUISE_DELTA - (v_cruise_kph % V_CRUISE_DELTA)
      elif b.type == "decelCruise":
        v_cruise_kph -= V_CRUISE_DELTA - ((V_CRUISE_DELTA - v_cruise_kph) % V_CRUISE_DELTA)
      v_cruise_kph = clip(v_cruise_kph, cruiseMin, V_CRUISE_MAX)

  return v_cruise_kph


def initialize_v_cruise(v_ego, buttonEvents, v_cruise_last, enableACCAccelControl):
  # 250kph or above probably means we never had a set speed
  if v_cruise_last < 250:
    for b in buttonEvents:
      if enableACCAccelControl: # Resume from current speed
        if b.type == "resumeCruise":
          return v_cruise_last  

      elif b.type == "accelCruise":
        return v_cruise_last

  cruiseMinEnable = V_CRUISE_ENABLE_MIN if enableACCAccelControl else 32
  return int(round(clip(v_ego * CV.MS_TO_KPH, cruiseMinEnable, V_CRUISE_MAX)))
