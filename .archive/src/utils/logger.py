import logging
import os
from logging.handlers import RotatingFileHandler
from datetime import datetime, timedelta

from src.utils.file_utils import FileUtils

def configure_root_logger(log_file=FileUtils.resolve_path(".logs/genai_paper_research.log"), level=logging.INFO, max_bytes=5 * 1024 * 1024, backup_count=4, days_to_keep=90, console_logging=True):
    """
    Set up a logger with both file and console handlers, with log rotation and cleanup.

    Args:
        name (str): Name of the logger.
        log_file (str): Path to the log file.
        level (int): Logging level (e.g., logging.INFO).
        max_bytes (int): Maximum size of a log file in bytes before rotation.
        backup_count (int): Number of rotated log files to keep.
        days_to_keep (int): Number of days to keep old logs.

    Returns:
        logging.Logger: Configured logger.
    """
    os.makedirs(os.path.dirname(log_file), exist_ok=True)

    # Cleanup old logs
    cleanup_logs(os.path.dirname(log_file), days_to_keep)

    # File handler with rotation
    file_handler = RotatingFileHandler(log_file, maxBytes=max_bytes, backupCount=backup_count)
    file_handler.setLevel(level)
    file_formatter = logging.Formatter("%(asctime)s - %(name)-22s - %(levelname)s - %(message)s")
    file_handler.setFormatter(file_formatter)

    logging.basicConfig(
        level=level,
        handlers=[file_handler],    # Add console_handler to the list if needed
    )

    if console_logging:
            console_handler = logging.StreamHandler()
            console_handler.setLevel(level)
            console_formatter = logging.Formatter(
                '%(name)-22s - %(levelname)s - %(message)s'
            )
            console_handler.setFormatter(console_formatter)
            logging.addHandler(console_handler)

def cleanup_logs(log_dir, days_to_keep):
    """
    Remove logs older than a certain number of days.

    Args:
        log_dir (str): Directory containing log files.
        days_to_keep (int): Number of days to keep old logs.
    """
    now = datetime.now()
    cutoff = now - timedelta(days=days_to_keep)

    for log_file in os.listdir(log_dir):
        log_path = os.path.join(log_dir, log_file)
        if os.path.isfile(log_path) and log_file.endswith(".log"):
            file_time = datetime.fromtimestamp(os.path.getmtime(log_path))
            if file_time < cutoff:
                os.remove(log_path)
                print(f"Deleted old log file: {log_path}")


def setup_logger(name: str) -> logging.Logger:
    logger = logging.getLogger(name)
    return logger