
import math
import statistics
import config

class ObstacleDetector:
    def __init__(self):
        self.obstacle_limit = config.OBSTACLE_DISTANCE
        self.wall_limit = config.WALL_DISTANCE
        self.emergency_limit = config.EMERGENCY_DISTANCE

    def sector_points(self, points, start, end):
        return [
            p for p in points
            if start <= p["angle"] <= end
        ]

    def nearest_distance(self, points):
        if len(points) < config.LIDAR_MIN_POINTS:
            return None

        distances = sorted(
            p["distance"] for p in points
        )

        return statistics.median(distances[:5])

    def wall_candidate(self, points):
        """
        Mencari kelompok titik yang mendekati
        garis lurus panjang menggunakan PCA 2D.

        Hasil hanya kandidat dinding.
        """
        if len(points) < 8:
            return False

        xs = [p["x"] for p in points]
        ys = [p["y"] for p in points]

        mx = statistics.mean(xs)
        my = statistics.mean(ys)

        sxx = statistics.mean(
            (x - mx) ** 2 for x in xs
        )
        syy = statistics.mean(
            (y - my) ** 2 for y in ys
        )
        sxy = statistics.mean(
            (x - mx) * (y - my)
            for x, y in zip(xs, ys)
        )

        theta = 0.5 * math.atan2(
            2 * sxy, sxx - syy
        )

        ux = math.cos(theta)
        uy = math.sin(theta)

        projections = []
        errors = []

        for x, y in zip(xs, ys):
            dx = x - mx
            dy = y - my

            projections.append(dx * ux + dy * uy)
            errors.append(abs(-dx * uy + dy * ux))

        length = max(projections) - min(projections)
        error = statistics.median(errors)

        return (
            length >= config.WALL_MIN_LENGTH_CM
            and error <= config.WALL_LINE_TOLERANCE_CM
        )

    def analyze(self, points):
        front = self.sector_points(points, -30, 30)
        left = self.sector_points(points, 30, 90)
        right = self.sector_points(points, -90, -30)

        distances = {
            "front": self.nearest_distance(front),
            "left": self.nearest_distance(left),
            "right": self.nearest_distance(right)
        }

        if any(v is None for v in distances.values()):
            return {
                "status": "SENSOR_UNCERTAIN",
                "action": "STOP",
                "distances": distances
            }

        f = distances["front"]
        l = distances["left"]
        r = distances["right"]

        if f < self.emergency_limit:
            action = "STOP"
        elif f < self.obstacle_limit:
            if l >= self.obstacle_limit and l >= r:
                action = "AVOID_LEFT"
            elif r >= self.obstacle_limit:
                action = "AVOID_RIGHT"
            else:
                action = "STOP"
        else:
            action = "FORWARD"

        # Klasifikasi hanya untuk informasi,
        # bukan dasar tunggal untuk menggerakkan motor.
        is_wall = (
            f < self.obstacle_limit
            and self.wall_candidate(front)
        )

        if is_wall:
            status = "WALL_CANDIDATE"
            action = "WALL_STOP"
        elif f < self.obstacle_limit:
            status = "OBSTACLE"
        else:
            status = "CLEAR"

        return {
            "status": status,
            "action": action,
            "distances": distances
        }
