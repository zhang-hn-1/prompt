"""Joint world model training package for the ball robot."""

from .ball_wm_dataset import BallWMDataset, SampleConfig, make_train_val_test_split

__all__ = ["BallWMDataset", "SampleConfig", "make_train_val_test_split"]
