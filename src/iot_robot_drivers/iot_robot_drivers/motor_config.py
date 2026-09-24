"""The motor CAN IDs and directions from motors.yaml, the only place they are set.

src/iot_robot_bringup/config/motors.yaml has one entry per joint, keyed by URDF joint
name:

    motors:
      wheel_joint_left:
        can_id: 12
        direction: 1
      gizmo_yaw_joint:
        can_id: 10
        direction: 1
        zero_on_start: true

robot.launch.py validates it with this module before it builds the URDF (which reads
the same file with xacro.load_yaml), and cubemars_tool takes its default CAN IDs and
the joint names it prints from it. `cubemars_tool identify --write` changes the can_id
values through write_can_ids, which keeps the comments.
"""

import os
import re
import tempfile
from dataclasses import dataclass

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

PACKAGE = "iot_robot_bringup"
WHEEL_JOINTS = ("wheel_joint_left", "wheel_joint_right")
GIZMO_JOINTS = ("gizmo_yaw_joint", "gizmo_pitch_joint")
JOINTS = WHEEL_JOINTS + GIZMO_JOINTS
# The servo protocol carries the motor ID in the low 8 bits of the extended frame ID
# (CubeMars AK Series Driver Manual, section 5.1), and CubeMars documents no narrower
# range for the Upper Computer's CAN ID field. 0 and 255 are left out: the drives are
# VESC-derived, and VESC firmware treats 255 as the broadcast ID
MIN_CAN_ID = 1
MAX_CAN_ID = 254
KEYS = ("can_id", "direction", "zero_on_start")


class MotorConfigError(ValueError):
    """motors.yaml is missing or invalid. The message names the file and the problem."""


@dataclass(frozen=True)
class Motor:
    joint: str
    can_id: int
    # 1 or -1: whether a positive motor command moves the joint in its positive URDF
    # direction
    direction: int
    # Set a temporary origin when the robot software starts (gizmo joints only)
    zero_on_start: bool = False

    @property
    def is_gizmo(self):
        return self.joint in GIZMO_JOINTS


@dataclass(frozen=True)
class MotorConfig:
    path: str
    # In file order
    motors: tuple

    def get(self, joint):
        """The Motor for a joint name, or None if the file does not list it."""
        return next((motor for motor in self.motors if motor.joint == joint), None)

    @property
    def wheels(self):
        return [self.get(joint) for joint in WHEEL_JOINTS]

    @property
    def gizmo(self):
        """The gizmo motors the file lists (none, one or both)."""
        return [motor for joint in GIZMO_JOINTS if (motor := self.get(joint))]

    def in_use(self, gizmo_mode):
        """The motors robot.launch.py drives: the wheels, plus the gizmo in can mode."""
        if gizmo_mode != "can":
            return self.wheels
        missing = [joint for joint in GIZMO_JOINTS if self.get(joint) is None]
        if missing:
            raise MotorConfigError(
                f"{self.path}: gizmo_mode can drives the gizmo motors, but the file has "
                f"no entry for {', '.join(missing)}. Add it, or use gizmo_mode:=fixed.")
        return self.wheels + self.gizmo

    def names(self):
        """{can_id: joint name} for every motor in the file."""
        return {motor.can_id: motor.joint for motor in self.motors}

    def joint_of(self, can_id):
        return self.names().get(can_id)


def default_path():
    """motors.yaml in the installed iot_robot_bringup share directory."""
    try:
        share = get_package_share_directory(PACKAGE)
    except (PackageNotFoundError, ValueError):
        raise MotorConfigError(
            f"Cannot find the {PACKAGE} package, which holds the default motors.yaml. "
            "Build the workspace with the environment you run in (`pixi run -e robot "
            "build` on the robot) so that pixi's activation sources install/, or name "
            "the file explicitly (cubemars_tool --motors PATH, robot.launch.py "
            "motors:=PATH).") from None
    return os.path.join(share, "config", "motors.yaml")


def load(path=None, require_gizmo=False):
    """Read and validate motors.yaml; path None means default_path().

    require_gizmo also demands entries for both gizmo joints, as gizmo_mode can does.
    Raises MotorConfigError listing every problem found.
    """
    path = path or default_path()
    try:
        with open(path) as stream:
            text = stream.read()
    except OSError as err:
        raise MotorConfigError(f"Cannot read the motor configuration {path}: "
                               f"{err.strerror}") from None
    return parse(text, path, require_gizmo)


def parse(text, path="motors.yaml", require_gizmo=False):
    """Validate the text of a motors.yaml; path is only used in messages."""
    try:
        data = yaml.load(text, Loader=_UniqueKeyLoader)
    except yaml.YAMLError as err:
        raise MotorConfigError(f"{path} is not valid YAML: {err}") from None

    if not isinstance(data, dict) or not isinstance(data.get("motors"), dict):
        raise MotorConfigError(
            f"{path}: expected a 'motors:' mapping with one entry per joint "
            f"({', '.join(JOINTS)}).")
    errors = [f"unknown top-level key '{key}' (only 'motors' is allowed)"
              for key in data if key != "motors"]

    motors = []
    for joint, entry in data["motors"].items():
        if joint not in JOINTS:
            errors.append(f"unknown joint '{joint}'; the joints are {', '.join(JOINTS)}")
            continue
        motor = _parse_motor(joint, entry, errors)
        if motor is not None:
            motors.append(motor)

    required = JOINTS if require_gizmo else WHEEL_JOINTS
    listed = set(data["motors"])
    errors += [f"no entry for {joint}" for joint in required if joint not in listed]

    by_id = {}
    for motor in motors:
        if motor.can_id in by_id:
            errors.append(f"CAN ID {motor.can_id} is given to both {by_id[motor.can_id]} "
                          f"and {motor.joint}; every motor needs its own ID")
        else:
            by_id[motor.can_id] = motor.joint

    if errors:
        raise MotorConfigError(
            f"Invalid motor configuration {path}:\n" + "\n".join(
                f"  - {error}" for error in errors))
    return MotorConfig(path=path, motors=tuple(motors))


def _parse_motor(joint, entry, errors):
    if not isinstance(entry, dict):
        errors.append(f"{joint}: expected a mapping with can_id and direction")
        return None
    count = len(errors)
    errors += [f"{joint}: unknown key '{key}' (allowed: {', '.join(KEYS)})"
               for key in entry if key not in KEYS]

    can_id = entry.get("can_id")
    if can_id is None:
        errors.append(f"{joint}: can_id is missing")
    elif not _is_int(can_id) or not MIN_CAN_ID <= can_id <= MAX_CAN_ID:
        errors.append(f"{joint}: can_id must be an integer from {MIN_CAN_ID} to "
                      f"{MAX_CAN_ID}, got {can_id!r}")

    direction = entry.get("direction")
    if direction is None:
        errors.append(f"{joint}: direction is missing (1 or -1)")
    elif not _is_int(direction) or direction not in (1, -1):
        errors.append(f"{joint}: direction must be 1 or -1, got {direction!r}")

    zero_on_start = entry.get("zero_on_start", False)
    if not isinstance(zero_on_start, bool):
        errors.append(f"{joint}: zero_on_start must be true or false, got "
                      f"{zero_on_start!r}")
    elif zero_on_start and joint not in GIZMO_JOINTS:
        errors.append(f"{joint}: zero_on_start applies to the gizmo joints only")

    if len(errors) > count:
        return None
    return Motor(joint=joint, can_id=can_id, direction=direction,
                 zero_on_start=zero_on_start)


def _is_int(value):
    # YAML true/false load as bool, which is a subclass of int
    return isinstance(value, int) and not isinstance(value, bool)


class _UniqueKeyLoader(yaml.SafeLoader):
    """SafeLoader that rejects a key given twice in one mapping.

    Plain YAML keeps the last value, so a copied joint block whose name was not changed
    would silently replace the first one.
    """


def _construct_mapping(loader, node, deep=False):
    seen = set()
    for key_node, _ in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in seen:
            raise yaml.constructor.ConstructorError(
                None, None, f"'{key}' appears twice in the same mapping",
                key_node.start_mark)
        seen.add(key)
    return loader.construct_mapping(node, deep=deep)


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping)


# Writing -----------------------------------------------------------------------------

_JOINT_LINE = re.compile(r"^(?P<indent> *)(?P<key>[A-Za-z0-9_]+)\s*:\s*(#.*)?$")
_CAN_ID_LINE = re.compile(r"^(?P<head> +can_id\s*:\s*)(?P<value>[^\s#]+)(?P<tail>.*)$",
                          re.DOTALL)


def write_can_ids(path, can_ids):
    """Set the can_id of some joints in motors.yaml, changing nothing else.

    can_ids maps joint name to the new CAN ID. Only those numbers are rewritten, in
    place: comments, key order and the other keys stay as they are. The result is
    validated before it replaces the file, so an assignment that leaves two joints on
    one ID is refused with MotorConfigError and the file is not touched.

    A symlink is followed, so the share-directory copy installed by
    `colcon build --symlink-install` updates the source file. Returns the path written.
    """
    real_path = os.path.realpath(path)
    try:
        with open(real_path) as stream:
            lines = stream.read().splitlines(keepends=True)
    except OSError as err:
        raise MotorConfigError(f"Cannot read {real_path}: {err.strerror}") from None

    in_motors = False
    joint_indent = None
    joint = None
    for index, line in enumerate(lines):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        indent = len(line) - len(line.lstrip(" "))
        if indent == 0:
            in_motors = stripped.split(":", 1)[0].strip() == "motors"
            joint = None
            continue
        if not in_motors:
            continue
        # The first line under motors: sets the indentation of the joint names
        if joint_indent is None:
            joint_indent = indent
        if indent == joint_indent:
            joint_match = _JOINT_LINE.match(line)
            joint = joint_match.group("key") if joint_match else None
            continue
        can_id_match = _CAN_ID_LINE.match(line)
        if joint in can_ids and can_id_match:
            lines[index] = (can_id_match.group("head") + str(int(can_ids[joint]))
                            + can_id_match.group("tail"))

    text = "".join(lines)
    config = parse(text, real_path)
    unchanged = [joint for joint, can_id in can_ids.items()
                 if (motor := config.get(joint)) is None or motor.can_id != can_id]
    if unchanged:
        raise MotorConfigError(
            f"Could not find the can_id line of {', '.join(unchanged)} in {real_path}; "
            "write it in block style ('can_id: <n>' on its own line) or edit it by hand.")

    # Write a sibling file and rename it over the original, so an interrupted write
    # never leaves half a file
    directory = os.path.dirname(real_path)
    fd, temp_path = tempfile.mkstemp(prefix=".motors-", suffix=".yaml", dir=directory)
    try:
        with os.fdopen(fd, "w") as stream:
            stream.write(text)
        os.chmod(temp_path, os.stat(real_path).st_mode & 0o7777)
        os.replace(temp_path, real_path)
    except BaseException:
        if os.path.exists(temp_path):
            os.unlink(temp_path)
        raise
    return real_path
