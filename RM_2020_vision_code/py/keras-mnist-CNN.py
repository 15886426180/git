# coding:utf-8
# 解决GPU问题
import tensorflow as tf
import keras.backend.tensorflow_backend as KTF
config = tf.ConfigProto()
config.gpu_options.allow_growth = True
sess = tf.Session(config=config)
KTF.set_session(sess)
# 解决GPU问题
# 绘图函数
import matplotlib.pyplot as plot
def show_train_history(train_history, train, validation):
    plot.plot(train_history.history[train])
    plot.plot(train_history.history[validation])
    plot.title('Train History')
    plot.ylabel(train)
    plot.xlabel('Epoch')
    plot.legend(['train', 'validation'], loc='upper left') # 设置图例显示是左上角
    plot.show()
# 绘图函数

from keras.datasets import mnist
from keras.utils import np_utils
import numpy as np

(x_train_img, y_train_label),(x_test_img, y_test_label) = mnist.load_data()

# 特征转化为4维矩阵
x_train4D = x_train_img.reshape(x_train_img.shape[0], 28, 28, 1).astype('float32')
x_test4D = x_test_img.reshape(x_test_img.shape[0], 28, 28, 1).astype('float32')

# 特征值标准化
x_train4D_normalize = x_train4D / 255
x_test4D_normalize = x_test4D / 255

# label One-Hot Encoding 转换
y_train_OneHot = np_utils.to_categorical(y_train_label)
y_test_OneHot = np_utils.to_categorical(y_test_label)

# 建立模型
from keras.models import Sequential
from keras.layers import Dense,Dropout,Flatten,Conv2D,MaxPool2D
model = Sequential()

# 卷积层1和池化层1
# 卷积层
model.add(Conv2D(filters=16,    # 建立16个滤镜
                 kernel_size=(5,5), # 每个滤镜的尺寸
                 padding='same',    # 表示卷积后图像尺寸不变
                 input_shape=(28, 28, 1),   #图像维度
                 activation='relu'))    # 设置激活函数
# 池化层
model.add(MaxPool2D(pool_size=(2,2)))
# 第一次缩减采样 16个28x28 变为16个14x14

# 卷积层2和池化层2
# 卷积层将之前的16个图像转变为36个
model.add(Conv2D(filters=36,    # 建立36个滤镜
                 kernel_size=(5,5), # 每个滤镜的尺寸
                 padding='same',    # 表示卷积后图像尺寸不变
                 activation='relu'))    # 设置激活函数

model.add(MaxPool2D(pool_size=(2,2)))
# 第二次缩减采样 36个14x14图像变成36个7x7图像
# 加入 Dropout 避免过拟合
model.add(Dropout(0.25))
# 每次训练迭代会随机放弃25%的神经元防止过拟合
# 建立平坦层
model.add(Flatten())
# 建立隐藏层
model.add(Dense(128, activation='relu'))
# 再加入Dropout 避免过拟合 这次放弃50%
model.add(Dropout(0.5))

# 建立输出层
model.add(Dense(10, activation='softmax'))
#print(model.summary())

# 定义训练方式
model.compile(loss='categorical_crossentropy',
              optimizer='adam',
              metrics=['accuracy'])

train_history = model.fit(x= x_train4D_normalize,   # features数字特征值
                          y= y_train_OneHot,    # label数字真实值
                          validation_split=0.2, # 训练数据和测试数据的比例
                          epochs=10,    # 10个训练周期
                          batch_size=300,   # 每个周期训练300个数据
                          verbose=2)    # 显示训练过程

show_train_history(train_history,'acc','val_acc')
show_train_history(train_history,'loss','val_loss')
score = model.evaluate(x_test4D_normalize, y_test_OneHot)
print(score[1])

# model_save_path = ''
# model.save(model_save_path)
# prediction = model.predict_classes(x_test4D_normalize)
# print(prediction[:10])
