"""motors.yaml loading, validation and the comment-preserving CAN ID rewrite."""

import os
import textwrap

import pytest

from iot_robot_drivers import motor_config
from iot_robot_drivers.motor_config import Motor, MotorConfigError, load, parse, write_can_ids

REPO_MOTORS = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "iot_robot_bringup", "config",
    "motors.yaml")

GOOD = textwrap.dedent("""\
    # Header comment
    motors:
      wheel_joint_left:
        can_id: 10   # left
        direction: 1
      wheel_joint_right:
        can_id: 11
        direction: -1
      gizmo_yaw_joint:
        can_id: 12
        direction: 1
        zero_on_start: true
      gizmo_pitch_joint:
        # pitch comment
        can_id: 13
        direction: 1
        zero_on_start: true
    """)

WHEELS_ONLY = textwrap.dedent("""\
    motors:
      wheel_joint_left: {can_id: 1, direction: 1}
      wheel_joint_right: {can_id: 2, direction: -1}
    """)


def errors_of(text, **kwargs):
    with pytest.raises(MotorConfigError) as err:
        parse(text, "test.yaml", **kwargs)
    return str(err.value)


class TestLoad:
    def test_the_repository_file_is_valid_for_every_gizmo_mode(self):
        config = load(REPO_MOTORS, require_gizmo=True)
        assert [motor.joint for motor in config.motors] == list(motor_config.JOINTS)
        assert len({motor.can_id for motor in config.motors}) == 4
        assert all(motor.zero_on_start for motor in config.gizmo)

    def test_motors_in_file_order_with_helpers(self):
        config = parse(GOOD, "test.yaml")
        assert config.wheels == [Motor("wheel_joint_left", 10, 1),
                                 Motor("wheel_joint_right", 11, -1)]
        assert config.gizmo == [Motor("gizmo_yaw_joint", 12, 1, True),
                                Motor("gizmo_pitch_joint", 13, 1, True)]
        assert config.names() == {10: "wheel_joint_left", 11: "wheel_joint_right",
                                  12: "gizmo_yaw_joint", 13: "gizmo_pitch_joint"}
        assert config.joint_of(13) == "gizmo_pitch_joint"
        assert config.joint_of(99) is None
        assert config.gizmo[0].is_gizmo and not config.wheels[0].is_gizmo

    def test_motors_in_use_depend_on_the_gizmo_mode(self):
        config = parse(GOOD, "test.yaml")
        assert [m.can_id for m in config.in_use("can")] == [10, 11, 12, 13]
        assert [m.can_id for m in config.in_use("fixed")] == [10, 11]

    def test_gizmo_joints_are_only_required_when_used(self):
        config = parse(WHEELS_ONLY, "test.yaml")
        assert config.gizmo == []
        assert [m.can_id for m in config.in_use("fixed")] == [1, 2]
        with pytest.raises(MotorConfigError, match="gizmo_mode:=fixed"):
            config.in_use("can")
        message = errors_of(WHEELS_ONLY, require_gizmo=True)
        assert "no entry for gizmo_yaw_joint" in message
        assert "no entry for gizmo_pitch_joint" in message

    def test_missing_file(self, tmp_path):
        with pytest.raises(MotorConfigError, match="Cannot read the motor configuration"):
            load(str(tmp_path / "nope.yaml"))

    def test_default_path_is_in_the_bringup_share_directory(self, monkeypatch):
        monkeypatch.setattr(motor_config, "get_package_share_directory",
                            lambda package: f"/opt/share/{package}")
        assert motor_config.default_path() == \
            "/opt/share/iot_robot_bringup/config/motors.yaml"

    def test_default_path_explains_a_missing_package(self, monkeypatch):
        def not_found(package):
            raise motor_config.PackageNotFoundError(package)
        monkeypatch.setattr(motor_config, "get_package_share_directory", not_found)
        with pytest.raises(MotorConfigError) as err:
            load()
        assert "Cannot find the iot_robot_bringup package" in str(err.value)
        assert "--motors PATH" in str(err.value) and "motors:=PATH" in str(err.value)


class TestValidation:
    def test_duplicate_ids_name_both_joints(self):
        message = errors_of(GOOD.replace("can_id: 11", "can_id: 10"))
        assert "CAN ID 10 is given to both wheel_joint_left and wheel_joint_right" in message

    @pytest.mark.parametrize("value", ["0", "255", "300", "-1", "ten", "true", "10.5"])
    def test_can_id_range_and_type(self, value):
        message = errors_of(GOOD.replace("can_id: 12", f"can_id: {value}"))
        assert "gizmo_yaw_joint: can_id must be an integer from 1 to 254" in message

    def test_hexadecimal_ids_are_integers(self):
        assert parse(GOOD.replace("can_id: 12", "can_id: 0x7F"), "t").joint_of(127) \
            == "gizmo_yaw_joint"

    @pytest.mark.parametrize("value", ["0", "2", "-2", "true", "1.0", "left"])
    def test_direction_is_plus_or_minus_one(self, value):
        message = errors_of(GOOD.replace("direction: -1", f"direction: {value}"))
        assert "wheel_joint_right: direction must be 1 or -1" in message

    def test_missing_keys(self):
        message = errors_of(GOOD.replace("    can_id: 11\n", "").replace(
            "    direction: 1\n  wheel_joint_right", "  wheel_joint_right"))
        assert "wheel_joint_right: can_id is missing" in message
        assert "wheel_joint_left: direction is missing (1 or -1)" in message

    def test_unknown_keys_and_joints(self):
        message = errors_of(GOOD.replace("direction: -1", "direction: -1\n    speed: 3")
                            + "  arm_joint:\n    can_id: 20\n    direction: 1\nextra: 1\n")
        assert "wheel_joint_right: unknown key 'speed'" in message
        assert "unknown joint 'arm_joint'" in message
        assert "unknown top-level key 'extra'" in message

    def test_zero_on_start_is_for_the_gizmo_only(self):
        message = errors_of(GOOD.replace("direction: -1", "direction: -1\n    "
                                         "zero_on_start: true"))
        assert "wheel_joint_right: zero_on_start applies to the gizmo joints only" in message
        message = errors_of(GOOD.replace("zero_on_start: true", "zero_on_start: yes please",
                                         1))
        assert "zero_on_start must be true or false" in message

    def test_missing_wheel(self):
        text = GOOD.replace("  wheel_joint_left:\n    can_id: 10   # left\n    "
                            "direction: 1\n", "")
        assert "no entry for wheel_joint_left" in errors_of(text)

    def test_every_problem_is_listed_at_once(self):
        message = errors_of(GOOD.replace("can_id: 13", "can_id: 999").replace(
            "direction: -1", "direction: 5"))
        assert "gizmo_pitch_joint: can_id" in message and "wheel_joint_right: direction" \
            in message

    def test_a_joint_listed_twice_is_an_error(self):
        message = errors_of(GOOD + "  wheel_joint_left:\n    can_id: 20\n    direction: 1\n")
        assert "'wheel_joint_left' appears twice" in message

    @pytest.mark.parametrize("text", ["", "motors: []\n", "- 1\n", "motors:\n"])
    def test_wrong_structure(self, text):
        assert "expected a 'motors:' mapping" in errors_of(text)

    def test_invalid_yaml(self):
        assert "is not valid YAML" in errors_of("motors: {wheel_joint_left: [\n")


class TestWriteCanIds:
    def test_only_the_numbers_change(self, tmp_path):
        path = tmp_path / "motors.yaml"
        path.write_text(GOOD)
        written = write_can_ids(str(path), {"wheel_joint_left": 11,
                                            "wheel_joint_right": 10})
        assert written == str(path)
        expected = GOOD.replace("can_id: 10   # left", "can_id: X   # left").replace(
            "can_id: 11", "can_id: 10").replace("can_id: X", "can_id: 11")
        assert path.read_text() == expected

    def test_keeps_the_file_mode(self, tmp_path):
        path = tmp_path / "motors.yaml"
        path.write_text(GOOD)
        path.chmod(0o664)
        write_can_ids(str(path), {"gizmo_pitch_joint": 20})
        assert path.stat().st_mode & 0o777 == 0o664
        assert "can_id: 20" in path.read_text()

    def test_a_symlink_updates_its_target(self, tmp_path):
        source = tmp_path / "src" / "motors.yaml"
        source.parent.mkdir()
        source.write_text(GOOD)
        link = tmp_path / "share" / "motors.yaml"
        link.parent.mkdir()
        link.symlink_to(source)
        assert write_can_ids(str(link), {"gizmo_yaw_joint": 30}) == str(source)
        assert link.is_symlink()
        assert load(str(source)).get("gizmo_yaw_joint").can_id == 30

    def test_refuses_an_assignment_that_duplicates_an_id(self, tmp_path):
        path = tmp_path / "motors.yaml"
        path.write_text(GOOD)
        with pytest.raises(MotorConfigError, match="CAN ID 11 is given to both"):
            write_can_ids(str(path), {"wheel_joint_left": 11})
        assert path.read_text() == GOOD
        assert [p.name for p in tmp_path.iterdir()] == ["motors.yaml"]

    def test_flow_style_entries_are_reported(self, tmp_path):
        path = tmp_path / "motors.yaml"
        path.write_text(WHEELS_ONLY)
        with pytest.raises(MotorConfigError, match="Could not find the can_id line of "
                                                   "wheel_joint_left"):
            write_can_ids(str(path), {"wheel_joint_left": 5})
        assert path.read_text() == WHEELS_ONLY
