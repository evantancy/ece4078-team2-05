# Milestone 3 (M3)
# Why we chose VGG16
Instead of using large receptive fields like AlexNet (11x11 with a stride of 4), VGG uses very small receptive fields (3x3 with a stride of 1). Because there are now three ReLU units instead of just one, the decision function is more discriminative. There are also fewer parameters (27 times the number of channels instead of AlexNet’s 49 times the number of channels).
# How to test our neural network
- Set `mode` in `master_config.yml` to `master`. This enables the use of `master_config.yml` instead of multiple training parameters