"""Select the supported Gazebo release without duplicating model resources."""
import atexit
import os
from pathlib import Path
import tempfile


def is_fortress():
    distro = os.environ.get('ROS_DISTRO')
    if distro not in ('humble', 'jazzy'):
        raise RuntimeError('Source ROS Humble (Fortress) or Jazzy (Harmonic) first')
    return distro == 'humble'


def gazebo_arguments(arguments):
    return {'gz_version': '6' if is_fortress() else '8', 'gz_args': arguments}


def gazebo_text(text):
    if not is_fortress():
        return text
    return (text.replace('gz-sim-', 'ignition-gazebo-')
            .replace('gz::sim::', 'ignition::gazebo::')
            .replace('gz.msgs.', 'ignition.msgs.'))


def prepare_share(share):
    """Build a temporary Fortress view; symlink unchanged/heavy assets.

    The launch process owns the view until exit. Source-share overrides use the
    same path, so they cannot accidentally load Harmonic plugins on Humble.
    """
    share = Path(share).resolve()
    if not is_fortress():
        return share
    directory = tempfile.TemporaryDirectory(prefix='sentry_fortress_')
    atexit.register(directory.cleanup)
    target = Path(directory.name)
    for source in share.iterdir():
        if source.name not in ('resource', 'config'):
            (target / source.name).symlink_to(source, target_is_directory=source.is_dir())
            continue
        for item in source.rglob('*'):
            destination = target / item.relative_to(share)
            if item.is_dir():
                destination.mkdir(parents=True, exist_ok=True)
            elif item.suffix in ('.sdf', '.yaml'):
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_text(gazebo_text(item.read_text()))
            else:
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.symlink_to(item)
    return target


def resource_paths(share):
    """Prepend the selected view ahead of any source paths set by a shell."""
    key = 'IGN_GAZEBO_RESOURCE_PATH' if is_fortress() else 'GZ_SIM_RESOURCE_PATH'
    entries = [str(Path(share) / 'resource/models'), str(Path(share) / 'resource/worlds')]
    if os.environ.get(key):
        entries.append(os.environ[key])
    return key, os.pathsep.join(entries)
