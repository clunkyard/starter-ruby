# Narrow the scan around a contact and lead shots from recent sightings.
# Clunkyard guide: https://clunkyard.com/sdk/ruby
require 'clunkyard'
CY = Clunkyard

SEARCH_ARC = 40  # degrees: sweeps the circle in half a second
HOLD_WIDTH = 60  # units across at the target: it stays in the beam from tick to tick
MIN_ARC = 6      # degrees: the narrowest the beam holds
WINDOW = 12      # ticks of sightings the aim averages
SIGHTINGS = 8    # narrow sightings in the window before a shot
LOOK_EVERY = 40  # ticks between looks around the target, for a nearer robot
REACQUIRE = 10   # ticks to look again where a lost target was

class Starter < CY::Robot
  def steer(towards,current)
    angles.clamp(angles.angle_diff(towards,current) / 45)
  end
  def on_start
    @free = match.is_3d && arena.gravity.z.zero?
    @dodges, @dodge_end, @slot = 0, 0, 0
  end

  # A straight line through the narrowest of the last WINDOW ticks' sightings:
  # where the robot is now, how fast it moves, and how many sightings it rests
  # on. Each sighting is off by its bearing_error; the line averages that out.
  def fit(slot,tick)
    recent = tracker.history(slot).select { |s| s.tick > tick - WINDOW }
    return nil if recent.size < 2
    best = recent.map(&:bearing_error).min
    seen = recent.select { |s| s.bearing_error <= 1.5 * best }
    n = seen.size
    return nil if n < 2
    t0 = seen.sum(&:tick).to_f / n
    spread = seen.sum { |s| (s.tick - t0)**2 }
    position, velocity = {}, {}
    %i[x y z].each do |axis|
      mean = seen.sum { |s| s.position.public_send(axis) } / n
      slope = seen.sum { |s| (s.tick - t0) * (s.position.public_send(axis) - mean) } / spread
      position[axis] = mean + slope * (tick - t0)
      velocity[axis] = slope * match.tick_rate
    end
    [CY::Vec3.new(**position), CY::Vec3.new(**velocity), n]
  end

  def on_turn
    position, tick = me.position, clock.tick
    drive.forces(brake: 0.2)
    target, cue = sensors.nearest, tracker.cue
    last_seen = @slot != 0 ? tracker.last_seen(@slot) : nil
    if target
      @slot = target.slot
      throttle = target.range > 350 ? 0.7 : target.range < 200 ? -0.4 : 0.2
      drive.set(throttle: throttle, turn: steer(target.bearing,me.heading), pitch_turn: steer(target.elevation,me.pitch))
      # Halve the beam each tick until it is HOLD_WIDTH across at the target:
      # narrow enough for a precise bearing, wide enough to keep it in.
      hold = [MIN_ARC, 2 * Math.atan2(HOLD_WIDTH / 2.0, target.range) * 180 / Math::PI].max
      # In a crowd a narrow beam sees one robot. Every LOOK_EVERY ticks,
      # widen it for a tick: a nearer robot it finds becomes the target.
      arc = tick % LOOK_EVERY == 0 ? SEARCH_ARC : [hold, [SEARCH_ARC, scanner.last.arc / 2].min].max
      scanner.set(heading: target.bearing, arc: arc, pitch: target.elevation)
      power = gun.power_for_flight_time(target.range,1)
      # An exact contact: gun.lead is the intercept if the target holds
      # course. A noisy one: lead the fitted line, and fire only once it
      # rests on enough sightings.
      ready = true
      if target.bearing_error > 0
        line = fit(target.slot,tick)
        aim = line && gun.lead_point(line[0],line[1],power)
        ready = !line.nil? && line[2] >= SIGHTINGS
      else
        aim = gun.lead(target,power)
      end
      heading, pitch = aim ? [aim.heading,aim.pitch] : [target.bearing,target.elevation]
      gun.aim(heading,pitch)
      now = gun.aim_now
      gun.fire(power) if ready && angles.angle_diff(heading,now.heading).abs < 3 && (pitch-now.pitch).abs < 3 && gun.can_fire(power,10)
      status.set(ready ? 'TRACK' : 'NARROW',target.range)
    elsif last_seen && tick - last_seen <= REACQUIRE
      # Lost it: look again where it was last seen, widening the beam, before searching.
      where = tracker.position(@slot)
      bearing = angles.bearing_to(position,where)
      elevation = @free ? angles.elevation_to(position,where) : 0
      drive.set(throttle: 0.5, turn: steer(bearing,me.heading), pitch_turn: steer(elevation,me.pitch))
      scanner.set(heading: bearing, arc: [SEARCH_ARC, scanner.last.arc * 2].min, pitch: elevation)
      status.set('REACQUIRE')
    elsif cue
      bearing, distance = angles.bearing_to(position,cue.position), angles.distance(position,cue.position)
      elevation = @free ? angles.elevation_to(position,cue.position) : 0
      error = [cue.bearing_error,30].min
      across = (tick % 5 - 2) / 2.0 * error
      up = @free ? ((tick / 5) % 3 - 1) * error / 2 : 0
      drive.set(throttle: 0.7, turn: steer(bearing,me.heading), pitch_turn: steer(elevation,me.pitch))
      scanner.set(heading: bearing+across, arc: scanner.arc_for_reach(distance+100), pitch: elevation+up)
      status.set('CUE',distance)
    else
      drive.set(throttle: 0.5, turn: 0.33, pitch_turn: steer(0,me.pitch))
      scanner.set(heading: angles.normalize(scanner.last.heading+36), arc: SEARCH_ARC, pitch: @free ? ((tick / 10) % 3 - 1) * 35 : 0)
      status.set('SEARCH')
    end
    # A simple dodge: vary it, or predict the shooter.
    if tick >= @dodge_end && sensors.cues.any? { |c| c.source == 'muzzle' }
      @dodges += 1
      @dodge_end = tick + 10
    end
    if tick < @dodge_end
      # Off the line of fire: left, then right next time (turn is positive left, strafe right).
      left = @dodges.odd?
      if match.control == 'forces'
        drive.set(throttle: 0.5)
        drive.forces(strafe: left ? -1 : 1)
      else
        drive.set(throttle: 1, turn: left ? 1 : -1)
      end
      status.set(left ? 'dodge:left' : 'dodge:right',@dodges)
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
  end
end

CY.run(Starter.new) if $PROGRAM_NAME == __FILE__
