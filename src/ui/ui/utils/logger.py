import logging
from datetime import datetime as d
from watchdog.events import FileSystemEventHandler, FileSystemEvent
from textual.widgets import RichLog
import __main__
import os
from rich.text import Text
from rich.style import Style
from watchdog.observers import ObserverType

LOG_LEVEL = {
    "Debug": logging.DEBUG,
    "Info": logging.INFO,
    "Warning": logging.WARNING,
    "Error": logging.ERROR,
    "Critical": logging.CRITICAL,
}


def map_level(level: str) -> int:
    """
    Maps the level string to the corresponding logging level.

    :param level: the level string
    :return: the logging level
    """
    return LOG_LEVEL.get(level, logging.DEBUG)


def set_log_event_function(callback: callable):
    """Set a callback function to be called when a log is done. The callback function must accept logRecord as the first argument and return True to be recorded or False to be ignored.
    Although it is not an intended way to use this as a event handler it is a simple and effective way to use it.

    Args:
        callback (callable): function to be called when a log is done
    """
    logging.root.addFilter(callback)


def configure_logger(level: str = "Debug"):
    """
    Configure the logger.

    :param level: the level to log at
    """
    lvl = map_level(level)
    print(f"Configuring logger {level}...")
    start_time = d.now().strftime("%Y-%m-%d.%H:%M:%S")
    filename = "ui." + __main__.__file__.split("/")[-1].split(".")[0]
    logging.basicConfig(
        filename=f".log/{start_time}.{filename}.{level}.log",
        format="%(filename)s: %(message)s",
        level=lvl,  # TODO: hook it to env or config file
        force=True,
    )


last_read_line = {}


def get_new_lines(path: str) -> list[str]:
    if path not in last_read_line:
        last_read_line[path] = 0
    unread_lines = []
    with open(path, "r") as f:
        f.seek(last_read_line[path])
        while True:
            buffer = f.read(1024)
            if not buffer:
                break
            lines = buffer.splitlines()
            for line in lines:
                unread_lines.append(line.strip())
            last_read_line[path] = f.tell()
    return unread_lines


class LogHandler(FileSystemEventHandler):
    observer: ObserverType
    logger: RichLog
    ID_style = Style(color="white", bold=True, bgcolor="orange_red1")
    regularStyle = Style(color="white", bgcolor="black")

    def __init__(self, logger: RichLog, observer: ObserverType):
        super().__init__()
        self.logger = logger
        self.observer = observer

    def on_created(self, event):
        path = event.src_path
        path_id = ".".join(path.split(".")[-3:-1]) + ":"
        content = Text("Program Log Started!", self.regularStyle)
        id = Text(path_id, self.ID_style)
        self.logger.write(id + content)

    def on_modified(self, event: FileSystemEvent):
        path = event.src_path
        path_id = ".".join(path.split(".")[-3:-1]) + ":"
        id = Text(path_id, self.ID_style)
        content = Text("\n".join(get_new_lines(path)), self.regularStyle)
        if content:
            self.logger.write(id + content)

    def on_closed(self, event: FileSystemEvent):
        path = event.src_path
        path_id = ".".join(path.split(".")[-3:-1]) + ":"
        content = Text("Program Log Closed!", style=self.regularStyle)
        id = Text(path_id, self.ID_style)
        self.logger.write(id + content)
