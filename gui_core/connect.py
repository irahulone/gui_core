from subprocess import Popen, PIPE, TimeoutExpired

import logging
logging.basicConfig(level=logging.WARNING)

RETRIES = 3


class Connect:
    def __init__(self):
        pass

    def handle_shell_request(self, cmd: str) -> str:
        """
        cmd needs to be a shell command that you type into a terminal
        or else there will be strange errors
        :return: decoded message as a string
        """
        with Popen(f'{cmd}', stdout=PIPE, shell=True) as proc:
            for retry in range(RETRIES):
                try:
                    stdout, errs = proc.communicate(timeout=5)
                except Exception as e:
                    logging.error(f"Popen Communicate Exception: {e}")
                else:
                    if errs:
                        logging.error(f"Shell error: {errs}")
                    else:
                        return stdout.decode("utf-8", "ignore").strip('\n')

    def handle_shell_ping(self, cmd: str) -> int:
        """
        :param cmd: command to ping an IP. Fundamentally different from shell request that returns an output
        :return: 0 if ping test was successful else 1 if failed
        """
        with Popen(f'{cmd}', shell=True) as proc:
            fail = 1
            for retry in range(RETRIES):
                proc.wait()
                fail = proc.poll()
                if fail:
                    continue
                else:
                    break

            return fail
