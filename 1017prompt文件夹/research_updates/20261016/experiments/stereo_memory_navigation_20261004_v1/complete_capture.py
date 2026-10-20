"""A single state/history capture followed by a protected image transaction."""
from render_flush import fixed_render_flush
from capture_transaction import replace_step_images


def complete_capture(capture,render,clock,snapshot,images):
    step=capture()
    fixed_render_flush(render,clock,snapshot)
    return replace_step_images(step,*images())
