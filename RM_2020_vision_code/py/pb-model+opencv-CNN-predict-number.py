# coding:utf-8
# # 解决GPU问题
# import tensorflow as tf
# import keras.backend.tensorflow_backend as KTF
# config = tf.ConfigProto()
# config.gpu_options.allow_growth = True
# sess = tf.Session(config=config)
# KTF.set_session(sess)
# # 解决GPU问题
import cv2
import numpy as np
import tensorflow as tf

cap = cv2.VideoCapture(0)
pb_path = './CNN_model1.pb'

with tf.Graph().as_default():
    output_graph_def = tf.GraphDef()

with open(pb_path,'rb') as f:
    output_graph_def.ParseFromString(f.read())
    tensors = tf.import_graph_def(output_graph_def,name="")
    print("tensor",tensors)

config = tf.ConfigProto()
config.gpu_options.allow_growth = True
with tf.Session(config=config) as sess:
    init = tf.global_variables_initializer()
    sess.run(init)
    op = sess.graph.get_operations()
    input_x = sess.graph.get_tensor_by_name("conv2d_1_input:0")
    print("input_x: ",input_x)
    out_softmax = sess.graph.get_tensor_by_name("dense_2/Softmax:0")
    print("Output: ", out_softmax)

    while(1):
        ret_src, src_img = cap.read()
        gray_img = cv2.cvtColor(src_img,cv2.COLOR_BGR2GRAY)
        #ret_bin, bin_img = cv2.threshold(gray_img,40,255,cv2.THRESH_BINARY_INV)
        ret_bin, bin_img = cv2.threshold(gray_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        #bin_img = cv2.Canny(bin_img,100,200)
        bin_img = cv2.medianBlur(bin_img, 5)
        contours, hierarchy = cv2.findContours(bin_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        for i in range(0, len(contours)):
            x, y, w, h = cv2.boundingRect(contours[i])

            area = w * h
            if (22000 < area < 44000):
                cv2.rectangle(src_img, (x, y), (x + w, y + h), (153, 153, 0), 1)
                roi = bin_img[y+10:y + h-10, x+10:x + w-10]
                cv2.imshow('roi',roi)
                resize_img = cv2.resize(roi,(28,28),cv2.INTER_NEAREST)
                test_img = (resize_img.reshape(1,28,28,1)).astype('float32')/255
                img_out_softmax = sess.run(out_softmax, feed_dict={input_x: test_img})
                #print("img_out_softmax:", img_out_softmax)

                prediction = np.argmax(img_out_softmax, axis=1)

                print(prediction)

        cv2.namedWindow('src_img', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('src_img', src_img)
        cv2.namedWindow('bin_img', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('bin_img', bin_img)
        if cv2.waitKey(1) & 0xff == 27:
            cap.release()
            break

cv2.destroyAllWindows()