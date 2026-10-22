"""Socket ownership regressions; fake ports only, no GUI or robot connection."""
import io
import queue
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import tap_serial_control_gui as gui


class Port:
    def __init__(self):
        self.is_open = True

    def close(self):
        self.is_open = False


class ReconnectTest(unittest.TestCase):
    def test_failed_write_cannot_close_replacement_socket(self):
        link = gui.SerialLink(queue.Queue())
        link._tcp_target = ('fake', 8023)
        link.log_file = io.StringIO()
        replacement = Port()

        class FailingPort(Port):
            def write(self, data):
                # Deterministic version of a reconnect overtaking a failed write.
                link.port = replacement
                raise BrokenPipeError('old socket failed')

        old = FailingPort()
        link.port = old
        self.assertFalse(link.send('GET'))
        self.assertFalse(old.is_open)
        self.assertTrue(replacement.is_open)
        self.assertIn('link write error: old socket failed', link.log_file.getvalue())

    def test_close_during_connect_does_not_install_new_socket(self):
        link = gui.SerialLink(queue.Queue())
        link._tcp_target = ('fake', 8023)
        old, new = Port(), Port()
        link.port = old

        def connecting(*args):
            link.stop_event.set()
            return new

        with patch.object(link.stop_event, 'wait', return_value=False), \
                patch.object(gui, '_TcpPort', side_effect=connecting):
            self.assertFalse(link._reconnect())
        self.assertIs(link.port, old)
        self.assertFalse(new.is_open)


if __name__ == '__main__':
    unittest.main()
