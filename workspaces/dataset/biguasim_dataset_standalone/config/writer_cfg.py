from dataclasses import dataclass, field, asdict

@dataclass
class WriterConfig:
    name: str
    type: str
    sensor: str
    output: str

    field_names: list[str] | None = None
    raw:dict = field(default_factory=dict)

    def to_dict(self,):
        return asdict(self)
    
    @property
    def field(self,):
        """
        Optional subfield of the sensor output.

        Examples
        --------
        None                  -> state['RGBCamera']

        'raw'                 -> state['ImagingSonar']['raw']

        'gt_intensity'        -> state['ImagingSonar']['gt_intensity']
        """
        return self.raw.get("field")

    @property
    def channel_order(self,):
        """Get the color space of the array"""
        return self.raw.get("channel_order", "native")

    @property
    def extension(self):
        """Get the file extension for the writer based on its type."""
        return self.raw.get("extension", None)
    
    @property
    def remove_alpha(self):
        """Check if the writer configuration specifies to remove alpha channel."""
        return self.raw.get("remove_alpha", False)

    @property
    def dedupe(self):
        """Whether Collector should skip a frame when the sensor handed back
        byte-for-byte the same reading as the previous written frame.

        Defaults True (the sub-Hz dedup described in Collector.__init__).
        Set `dedupe: false` for a sensor whose output legitimately repeats
        while still being a real, wanted sample -- e.g. an AnnotationComponent
        pointed at a STATIC scene (drone in `hold`): every semantic mask is
        pixel-identical, so value-dedup would collapse the whole run to 1-2
        files. RGBCamera dodges this only because water shimmer makes each
        RGB frame slightly different.
        """
        return self.raw.get("dedupe", True)
    
    @property
    def filename(self):
        """Get the filename for the writer based on its configuration."""
        return self.raw.get("filename", None)
    
    @property
    def use_rpy(self):
        """Check if the writer configuration specifies to use Roll, Pitch, Yaw (RPY)."""
        return self.raw.get("use_rpy", False)

    @property
    def kwargs(self):
        """Returns all writer-specific configuration parameters,
        excluding the common ones.
        """
        ignore = {
            "type",
            "sensor",
            "output",
            "field",
        }

        return {
            k: v
            for k, v in self.raw.items()
            if k not in ignore
        }