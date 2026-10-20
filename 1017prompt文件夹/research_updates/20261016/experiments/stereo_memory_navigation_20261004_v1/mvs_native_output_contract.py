"""Official inference decoder setting for frozen native640x480; no weight changes."""
def configure_native_output(model):
 model.depth_decoder.set_prediction_scale(1.0)
