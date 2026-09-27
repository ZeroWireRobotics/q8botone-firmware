'''
Written by yufeng.wu0902@gmail.com

PlatformIO extra script, shared by robot/ and controller/ (see
extra_scripts in their platformio.ini): stamps the firmware version into the
build as Q8BOT_FW_VERSION, which main.cpp appends to FIRMWARE_ID.

The version comes from `git describe`, so a git tag is the only place a
version is ever written down. Tag a release vX.Y.Z and builds report:

    0.1.0                     the tagged commit itself, no local changes
    0.1.0-3-gabc1234          3 commits past v0.1.0
    0.1.0-3-gabc1234-dirty    ...plus uncommitted changes (anywhere in this repo)
    0.0.0-gabc1234            no release tag reachable yet
    unknown                   not built from a git checkout

Robot and controller share this repo, so they share one version line.

Only the project's own sources get the define (projenv, not env), so a new
version recompiles src/ but not the Arduino core or the libraries.
'''

import subprocess

Import('env', 'projenv')


def _git(*args):
    return subprocess.check_output(('git',) + args, cwd=env.subst('$PROJECT_DIR'),
                                   stderr=subprocess.DEVNULL, text=True).strip()


def firmware_version():
    try:
        return _git('describe', '--tags', '--match', 'v[0-9]*', '--dirty')[1:]
    except (OSError, subprocess.CalledProcessError):
        pass
    try:
        return '0.0.0-g' + _git('describe', '--always', '--dirty')
    except (OSError, subprocess.CalledProcessError):
        return 'unknown'


version = firmware_version()
print(f'Q8BOT_FW_VERSION = {version}')
projenv.Append(CPPDEFINES=[('Q8BOT_FW_VERSION', env.StringifyMacro(version))])
