"""Camera availability policy for LiDAR-first replay."""


def camera_fusion_available(topic_types: dict[str, str], image_topic: str, mode: str) -> bool:
    """Return whether camera fusion should run, or reject an unmet requirement."""
    if mode not in {"auto", "required", "off"}:
        raise ValueError(f"Unsupported camera mode: {mode}")
    image_present = image_topic in topic_types
    if mode == "required" and not image_present:
        raise RuntimeError(f"Camera mode is required but bag lacks: {image_topic}")
    return image_present and mode != "off"
