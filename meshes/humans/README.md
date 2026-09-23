# Synthetic mesh fixture

`synthetic_person.obj` is the original rectangular fixture retained for tests.

`low_poly_person.obj` is an original, stylized standing person with a separate
head, torso, arms, hands, legs, and feet. It is used by
`config/scenarios/sequential_objects_lidar.yaml`. Regenerate it with
`python meshes/humans/generate_low_poly_person.py` from the package directory.
Its unscaled bounds are about 0.98 m wide and 1.81 m high; the sequential
scenario uses `[1, 1, 1]` scale.
Both OBJ files use meters, stand on Z=0, and are dedicated to the public domain
(CC0). The low-poly mesh is a visible test object, not a production human model.
