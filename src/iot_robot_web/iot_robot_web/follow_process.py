"""Run a follow behaviour (`ros2 launch ...`) as a child process that can always be stopped."""

import asyncio
import collections
import ctypes
import os
import signal

PR_SET_PDEATHSIG = 1  # from <linux/prctl.h>

try:
    _prctl = ctypes.CDLL(None, use_errno=True).prctl
except (OSError, AttributeError):  # not Linux
    _prctl = None


def _stop_with_parent(parent_pid):
    """Return a preexec_fn that makes the child get SIGINT when this process dies.

    Without it a crashed or SIGKILLed web node would leave the follow launch running,
    still driving the robot, with nothing left to stop it. SIGINT makes the launch stop
    its nodes the way Ctrl+C would. The kernel signals when the thread that forked the
    child exits; that is the asyncio thread, which lives as long as the process.
    """
    if _prctl is None:
        return None

    def preexec():
        # Runs between fork and exec, so it only makes system calls
        _prctl(PR_SET_PDEATHSIG, int(signal.SIGINT))
        if os.getppid() != parent_pid:
            os._exit(1)  # the parent died before prctl took effect

    return preexec


class FollowProcess:
    """One `ros2 launch` child in its own session (process group).

    The launch process is stopped the way a terminal would stop it: SIGINT to the launch
    process only, which then shuts its nodes down in order. Anything still alive after the
    timeout, including nodes the launch left behind, is killed with the whole group. If
    this process dies without stopping it, the kernel sends the launch SIGINT.
    """

    def __init__(self, argv, logger):
        self.argv = argv
        self.logger = logger
        self.process = None
        self.returncode = None
        self.stopping = False
        # The last lines of output, to explain a failure
        self.output = collections.deque(maxlen=40)
        self._reader = None

    async def start(self):
        self.logger.info(f"Starting: {' '.join(self.argv)}")
        self.process = await asyncio.create_subprocess_exec(
            *self.argv,
            stdin=asyncio.subprocess.DEVNULL,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
            # A new session keeps Ctrl+C in the terminal from reaching the child directly,
            # and gives it a process group that can be killed as a whole
            start_new_session=True,
            preexec_fn=_stop_with_parent(os.getpid()),
        )
        self._reader = asyncio.create_task(self._read_output())

    @property
    def running(self):
        return self.process is not None and self.process.returncode is None

    @property
    def last_line(self):
        return self.output[-1] if self.output else ""

    async def _read_output(self):
        # The pipe must be drained or the child blocks once its buffer fills
        async for raw in self.process.stdout:
            line = raw.decode(errors="replace").rstrip()
            if line:
                self.output.append(line)
                self.logger.debug(line)
        self.returncode = await self.process.wait()
        if not self.stopping and self.returncode != 0:
            self.logger.error(
                f"{' '.join(self.argv[:4])} exited with code {self.returncode}:\n"
                + "\n".join(list(self.output)[-10:]))

    async def stop(self, timeout):
        """SIGINT the launch process, wait up to `timeout` s, then SIGKILL the group."""
        if self.process is None:
            return
        self.stopping = True
        if self.process.returncode is None:
            try:
                self.process.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                await asyncio.wait_for(self.process.wait(), timeout)
            except asyncio.TimeoutError:
                self.logger.warning(
                    f"The follow launch ignored SIGINT for {timeout:.0f} s, killing it")
        # Also catches nodes that outlived their launch process
        self.kill()
        await self.process.wait()
        if self._reader is not None:
            try:
                await asyncio.wait_for(self._reader, 1.0)
            except (asyncio.TimeoutError, asyncio.CancelledError):
                pass

    def kill(self):
        """SIGKILL every process in the child's group. Safe to call at any time."""
        if self.process is None:
            return
        try:
            os.killpg(self.process.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
