import logging

def setup_logger(
    level: int = logging.INFO,
) -> None:
    """
    Configure the logger.
    """
    logging.basicConfig(
        level=level,
        format=(
            "%(asctime)s | "
            "%(levelname)-8s | "
            "%(name)s | "
            "%(message)s"
        ),
        datefmt="%H:%M:%S",
    )