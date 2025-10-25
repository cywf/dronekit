"""
Logging configuration for DroneKit library.
"""

import logging
import sys


def setup_logger(name='dronekit', level=logging.INFO):
    """
    Set up a logger with the specified name and level.
    
    Args:
        name: Logger name
        level: Logging level (default: INFO)
    
    Returns:
        Configured logger instance
    """
    logger = logging.getLogger(name)
    
    # Only add handler if logger doesn't have one already
    if not logger.handlers:
        logger.setLevel(level)
        
        # Create console handler with formatting
        handler = logging.StreamHandler(sys.stdout)
        handler.setLevel(level)
        
        # Create formatter
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        handler.setFormatter(formatter)
        
        logger.addHandler(handler)
    
    return logger


# Default logger instance
logger = setup_logger()
