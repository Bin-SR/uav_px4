"""Time-parametrized NED paths. No ROS dependency or expanded lap waypoint lists."""
import bisect
import math


def check_point(point, origin):
    if (not all(math.isfinite(v) for v in point)
            or not 0.5 <= origin[2] - point[2] <= 10
            or math.hypot(point[0] - origin[0], point[1] - origin[1]) > 20 + 1e-8):
        raise ValueError('路径超出起飞点半径20米或高度0.5到10米范围')


class Profile:
    """Trapezoidal scalar velocity: bounded acceleration, zero endpoint velocity."""
    def __init__(self, distance, speed, acceleration):
        self.distance = distance
        self.speed = min(speed, math.sqrt(max(0.0, distance * acceleration)))
        self.acceleration = acceleration
        self.ramp = self.speed / acceleration
        self.cruise = max(0.0, distance / self.speed - self.ramp) if self.speed else 0.0
        self.duration = 2 * self.ramp + self.cruise

    def sample(self, t):
        if t <= 0 or not self.distance:
            return 0.0, 0.0, 0.0
        if t >= self.duration:
            return self.distance, 0.0, 0.0
        if t < self.ramp:
            return 0.5 * self.acceleration * t*t, self.acceleration*t, self.acceleration
        if t < self.ramp + self.cruise:
            return (0.5 * self.speed * self.ramp + self.speed * (t-self.ramp), self.speed, 0.0)
        remaining = self.duration - t
        return (self.distance - 0.5*self.acceleration*remaining**2,
                self.acceleration*remaining, -self.acceleration)


class Line:
    def __init__(self, start, end, speed=1.0, acceleration=2.0):
        self.start, self.end = list(start), list(end)
        distance = math.dist(start, end)
        self.direction = [(b-a)/distance if distance else 0.0 for a, b in zip(start, end)]
        self.profile = Profile(distance, speed, acceleration)
        self.duration = self.profile.duration

    def sample(self, t):
        s, v, a = self.profile.sample(t)
        return ([x+d*s for x, d in zip(self.start, self.direction)],
                [d*v for d in self.direction], [d*a for d in self.direction])


class Circuit:
    """Approach followed by repeated square/polygon or analytic circular path."""
    def __init__(self, step, start, origin, acceleration=2.0):
        center = step.get('center', 'takeoff')
        base = origin if center == 'takeoff' else start if center == 'current' else None
        if base is None:
            self.center = [origin[0] + center[0], origin[1] + center[1]]
        else:
            self.center = list(base[:2])
        self.z = origin[2] - step['height']
        self.kind = step['action']
        self.heading = step.get('heading', 'center')
        self.laps = step.get('laps', 1)
        self.sign = 1 if step.get('clockwise', True) else -1
        self.rotation = math.radians(step.get('rotation_deg', 0.0))
        self.speed = step['speed']
        cx, cy = self.center
        if self.kind == 'circle':
            self.radius = step['radius']
            if math.hypot(cx-origin[0], cy-origin[1]) + self.radius > 20 + 1e-8:
                raise ValueError('圆周超出水平半径20米范围')
            # Reserve acceleration for both tangential and centripetal terms.
            self.speed = min(self.speed, math.sqrt(acceleration*self.radius/2))
            self.progress = Profile(2*math.pi*self.radius*self.laps, self.speed, acceleration/2)
            self.body_duration = self.progress.duration
            first = [cx+self.radius*math.cos(self.rotation),
                     cy+self.radius*math.sin(self.rotation), self.z]
        else:
            sides = 4 if self.kind == 'square' else step['sides']
            radius = step['side']/(2*math.sin(math.pi/sides))
            angle = self.rotation + (math.pi/4 if self.kind == 'square' else 0)
            vertices = [[cx+radius*math.cos(angle+self.sign*2*math.pi*i/sides),
                         cy+radius*math.sin(angle+self.sign*2*math.pi*i/sides), self.z]
                        for i in range(sides)]
            for point in vertices:
                check_point(point, origin)
            self.edges = [Line(vertices[i], vertices[(i+1) % sides], self.speed, acceleration)
                          for i in range(sides)]
            self.ends = []
            total = 0.0
            for edge in self.edges:
                total += edge.duration
                self.ends.append(total)
            self.lap_duration = total
            self.body_duration = total*self.laps
            first = vertices[0]
        check_point(first, origin)
        self.approach = Line(start, first, min(step['speed'], 1.0), acceleration)
        self.end = first
        self.duration = self.approach.duration + self.body_duration
        if not math.isfinite(self.duration):
            raise ValueError('轨迹时长超出数值范围')

    def sample(self, t):
        if t < self.approach.duration:
            return self.approach.sample(t)
        t -= self.approach.duration
        if t >= self.body_duration:
            return list(self.end), [0.0]*3, [0.0]*3
        if self.kind == 'circle':
            distance, speed, acceleration = self.progress.sample(t)
            angle = self.rotation + self.sign*distance/self.radius
            c, s = math.cos(angle), math.sin(angle)
            omega = self.sign*speed/self.radius
            alpha = self.sign*acceleration/self.radius
            return ([self.center[0]+self.radius*c, self.center[1]+self.radius*s, self.z],
                    [-self.radius*s*omega, self.radius*c*omega, 0.0],
                    [-self.radius*c*omega**2-self.radius*s*alpha,
                     -self.radius*s*omega**2+self.radius*c*alpha, 0.0])
        lap_time = t % self.lap_duration
        index = min(bisect.bisect_right(self.ends, lap_time), len(self.edges)-1)
        return self.edges[index].sample(lap_time - (self.ends[index-1] if index else 0))


def bearing(position, center, fallback):
    dx, dy = center[0]-position[0], center[1]-position[1]
    return math.atan2(dy, dx) if math.hypot(dx, dy) > 0.1 else fallback


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Waypoints:
    """Absolute waypoints relative to takeoff: [north, east, height] in metres."""
    def __init__(self, step, start, origin, acceleration=2.0):
        points = [[origin[0]+p[0], origin[1]+p[1], origin[2]-p[2]] for p in step['points']]
        for point in points:
            check_point(point, origin)
        self.edges, self.ends = [], []
        total = 0.0
        for point in points:
            edge = Line(start, point, step['speed'], acceleration)
            self.edges.append(edge)
            total += edge.duration
            self.ends.append(total)
            start = point
        self.duration, self.end = total, points[-1]
        self.heading = step.get('heading', 'hold')
        center = step.get('center', 'takeoff')
        self.center = list(origin[:2]) if center == 'takeoff' else [origin[i]+center[i] for i in range(2)]

    def sample(self, t):
        if t >= self.duration:
            return list(self.end), [0.0]*3, [0.0]*3
        index = min(bisect.bisect_right(self.ends, t), len(self.edges)-1)
        return self.edges[index].sample(t - (self.ends[index-1] if index else 0))
