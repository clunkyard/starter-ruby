# A robot on the core binding supplied by the arena.
# Tracking, protocol parsing, geometry and act serialization live in the core.
# Clunkyard robot API 2.0.0: one core under every language, the same
# answers everywhere. Facts (a call shows its arguments):
# match: mode ruleset control robots is_3d tick_rate max_ticks
#   robot_id(slot)
# clock: tick seconds remaining_ticks
# arena: size gravity terrain_kind ground_height(x y)
# me: health position velocity heading pitch roll angular_velocity
#   attitude grounded altitude applied
# battery: charge capacity regen sustainable_drive compute
#   drive_cost(fraction)
# drive: hover_lift
# gun: aim_now cooldown_ticks spread max_power damage_scale
#   can_fire(power reserve) shot_speed(power) shot_cost(power)
#   reload_ticks(power) shot_damage(power) power_for_speed(speed)
#   power_for_flight_time(range seconds) lead(contact power)
#   lead_point(position velocity power)
# scanner: last proximity_range reach(arc) arc_for_reach(range)
# sensors: contacts nearest cues contact(slot) position_of(contact)
# tracker: cue nearest_bearing_rate velocity(slot) position(slot)
#   last_seen(slot)
# nav: zone zone.margin(point) zone.contains(point margin)
#   zone.distance_from(point) storm storm.margin(point)
#   storm.contains(point margin) storm.distance_from(point) hill
#   hill.margin(point) hill.contains(point margin)
#   hill.distance_from(point) pickups nearest_pickup(kind)
#   field_distance(a b)
# score: me best armour
# angles: normalize(degrees) angle_diff(target current)
#   bearing_to(from to) elevation_to(from to) direction(heading pitch)
#   distance(a b) clamp(value low high)
# raw: number(path) text(path) has(path) flag(path) count(path) line()
# Controls: drive.set(throttle turn pitch_turn)
#   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
#   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
#   tracker.configure(smoothing forget_ticks cue_ticks)
#   status.set(text value) raw.extra(json)
# Every control argument is optional; a second call keeps what the first
# set. Absent facts are your language's none. Names are snake_case here;
# your language cases them its own way. Angles are degrees from the +x
# axis; pitch is positive up.
# Manual: https://clunkyard.com/manual#sdk
require 'clunkyard'
CY = Clunkyard

class Starter < CY::Robot
  def steer(towards,current)
    angles.clamp(angles.angle_diff(towards,current) / 45)
  end
  def on_start
    @free = match.is_3d && arena.gravity.z.zero?
  end
  def on_turn
    position = me.position
    drive.forces(brake: 0.2)
    target, cue = sensors.nearest, tracker.cue
    if target
      throttle = target.range > 350 ? 0.7 : target.range < 200 ? -0.4 : 0.2
      drive.set(throttle: throttle, turn: steer(target.bearing,me.heading), pitch_turn: steer(target.elevation,me.pitch))
      scanner.set(heading: target.bearing, arc: 20, pitch: target.elevation)
      power = gun.power_for_flight_time(target.range,1)
      aim = gun.lead(target,power)
      heading, pitch = aim ? [aim.heading,aim.pitch] : [target.bearing,target.elevation]
      gun.aim(heading,pitch)
      now = gun.aim_now
      gun.fire(power) if angles.angle_diff(heading,now.heading).abs < 3 && (pitch-now.pitch).abs < 3 && gun.can_fire(power,10)
      status.set('TRACK',target.range)
    elsif cue
      bearing, distance = angles.bearing_to(position,cue.position), angles.distance(position,cue.position)
      elevation = @free ? angles.elevation_to(position,cue.position) : 0
      error, tick = [cue.bearing_error,30].min, clock.tick
      across = (tick % 5 - 2) / 2.0 * error
      up = @free ? ((tick / 5) % 3 - 1) * error / 2 : 0
      drive.set(throttle: 0.7, turn: steer(bearing,me.heading), pitch_turn: steer(elevation,me.pitch))
      scanner.set(heading: bearing+across, arc: scanner.arc_for_reach(distance+100), pitch: elevation+up)
      status.set('CUE',distance)
    else
      drive.set(throttle: 0.5, turn: 0.33, pitch_turn: steer(0,me.pitch))
      scanner.set(heading: angles.normalize(scanner.last.heading+36), arc: 40, pitch: @free ? ((clock.tick / 10) % 3 - 1) * 35 : 0)
      status.set('SEARCH')
    end
    drive.set(throttle: drive.get('throttle')*0.5) if battery.charge < battery.capacity / 4
    storm, zone = nav.storm, nav.zone
    if storm && (storm.margin(position) || -1e9) > -60
      away = angles.bearing_to(storm.center,position)
      drive.set(throttle: 0.8, turn: angles.clamp(angles.angle_diff(away,me.heading)/30))
    end
    if zone && (zone.margin(position) || 1e9) < 60
      climb = zone.shape=='sphere' ? angles.elevation_to(position,zone.center) : 0
      drive.set(throttle: 0.8, turn: angles.clamp(angles.angle_diff(angles.bearing_to(position,zone.center),me.heading)/30), pitch_turn: angles.clamp((climb-me.pitch)/30))
    end
    work = raw.number('state.calibration.work')
    status.set('WORK',calibration_work(work.to_i,clock.tick)) if work && work.positive?
  end
end

# The calibration kernel, xorshift-sqrt v1: work rounds of a 64-bit xorshift
# and a square-root sum.
def calibration_work(work, tick)
  mask = 0xFFFFFFFFFFFFFFFF
  x = 88_172_645_463_325_252 ^ tick
  total = 0.0
  work.times do
    x ^= (x << 13) & mask
    x ^= x >> 7
    x ^= (x << 17) & mask
    total += Math.sqrt(x >> 11)
  end
  (total.floor + (x >> 40)) % 1_000_000
end

CY.run(Starter.new) if $PROGRAM_NAME == __FILE__
