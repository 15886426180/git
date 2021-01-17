# import tensorflow as tf
#
# config = tf.ConfigProto()
# config.gpu_options.allow_growth = True
# sess = tf.Session(config=config)
#
# serialized = config.SerializeToString()
# print(list(map(hex, serialized)))

import tensorflow as tf
gpu_options = tf.GPUOptions(allow_growth=True)
config = tf.ConfigProto(gpu_options=gpu_options)
serialized = config.SerializeToString()
print(list(map(hex, serialized)))