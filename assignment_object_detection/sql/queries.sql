-- 1. Nearest object of a requested class to a map coordinate.
SELECT
    object_id,
    class_name,
    map_x,
    map_y,
    map_z,
    sqrt(power(map_x - $2, 2) + power(map_y - $3, 2)) AS distance
FROM semantic_objects
WHERE class_name = $1
ORDER BY distance
LIMIT 1;

-- 2. All objects within a rectangular map region.
SELECT
    object_id,
    class_name,
    map_x,
    map_y,
    map_z,
    observation_count
FROM semantic_objects
WHERE map_x BETWEEN $1 AND $2
  AND map_y BETWEEN $3 AND $4
ORDER BY class_name, object_id;

-- 3. Histogram of object counts per class.
SELECT
    class_name,
    count(*) AS object_count
FROM semantic_objects
GROUP BY class_name
ORDER BY object_count DESC, class_name;

-- 4. All robot observation poses associated with an object.
SELECT
    observation_id,
    seen_at_ns,
    map_x,
    map_y,
    map_yaw,
    camera_x,
    camera_y,
    camera_z,
    bearing_rad,
    range_m,
    confidence
FROM object_observations
WHERE object_id = $1
ORDER BY seen_at_ns;
