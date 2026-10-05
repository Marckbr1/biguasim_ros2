from pathlib import Path
import cv2
import logging

from .base_writer import BaseWriter

logger = logging.getLogger(__name__)
class ImageWriter(BaseWriter):
    def __init__(
            self,
            output_dir: Path,
            sensor_cfg,
            writer_cfg,
    ):
        super().__init__(
                output_dir, 
                sensor_cfg,
                writer_cfg,
        )
        logger.info("Starting ImageWriter")
        self.extension = writer_cfg.extension
        self.remove_alpha = writer_cfg.remove_alpha


        if not self.extension.startswith("."):
            self.extension = "." + self.extension

        self.output_dir.mkdir(
            parents=True, 
            exist_ok=True
        )

    def write(
            self,
            frame_id, 
            tick, 
            timestamp, 
            image,
    ):
        """Save an image to disk"""

        if image is None:
            return  # Skip if the image is None
        
        # Remove alpha channel if requested
        if (
            self.remove_alpha
            and image.ndim == 3
            and image.shape[2] == 4
        ):
            image = image[:, :, :3]  # Remove alpha channel if present

        # BiguaSim sensors typically output BGR images, but 
        # AnnotationComponent returns RGB, while
        # OpenCV expects BGR for imwrite(). Convert only when
        # explicitly requested by the writer configuration.
        if (
            (self.writer_cfg.channel_order or "").lower() == "rgb"
            and image.ndim == 3
            and image.shape[2] >= 3
        ):
            image = cv2.cvtColor(
                image,
                cv2.COLOR_RGB2BGR,
            )
                
        filename = (
            self.output_dir 
            / f"{frame_id:06d}_{timestamp:.6f}{self.extension}"
        )
        
        cv2.imwrite(
            str(filename), 
            image,
        )
