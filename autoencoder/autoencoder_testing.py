from keras.layers import Permute, Conv2D, BatchNormalization, Dropout, Reshape, Activation, Input
from keras.models import Sequential, Model
from keras.optimizers import Adam, SGD
from keras.callbacks import ModelCheckpoint
from keras.preprocessing.image import load_img, img_to_array, array_to_img
import numpy as np
from max_unpooling_layers import MaxPoolingWithArgmax2D, MaxUnpooling2D
from keras.utils.vis_utils import plot_model
import time
import tensorflow as tf
from keras.callbacks import TensorBoard
import matplotlib
import cv2

from matplotlib import pyplot as plt
import pandas as pd

img_w = 200
img_h = 400
n_labels = 2


def data_processor(split_range):
    prefix_list = ['um', 'umm', 'uu']
    data = []
    # for i in split_range:
    #     feature = img_to_array(
    #         load_img('images_ours/img_cloud_' + str(i) + '.png',
    #                  color_mode="grayscale"))
    #     data.append(feature)
    for prefix in prefix_list:
        for i in split_range:
            iterator = "%06d" % i
            try:
                feature = img_to_array(
                    load_img('/home/mechlab/hackathon/data/features/' + prefix + '_' + iterator + '_0.png',
                             color_mode="grayscale"))
                data.append(feature)
            except:
                print(str(iterator) + " not found!")
                pass

    data = np.array(data)
    return data


test_data = data_processor(range(38, 39))

inputs = Input(shape=(img_h, img_w, 1))
encoded1 = Conv2D(filters=32, kernel_size=(3, 3), activation='elu', strides=(1, 1), padding='same', )(inputs)
encoded1 = BatchNormalization()(encoded1)
encoded2 = Conv2D(filters=32, kernel_size=(3, 3), activation='elu', strides=(1, 1), padding='same')(encoded1)
encoded2 = BatchNormalization()(encoded2)
max_pool, mask_1 = MaxPoolingWithArgmax2D((2, 2))(encoded2)
cnt_mod1 = Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(1, 1))(max_pool)
cnt_mod1 = Dropout(rate=0.25)(cnt_mod1)
cnt_mod1 = BatchNormalization()(cnt_mod1)
cnt_mod2 = Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(1, 2))(cnt_mod1)
cnt_mod2 = Dropout(rate=0.25)(cnt_mod2)
cnt_mod2 = BatchNormalization()(cnt_mod2)
cnt_mod3 = Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(2, 4))(cnt_mod2)
cnt_mod3 = Dropout(rate=0.25)(cnt_mod3)
cnt_mod3 = BatchNormalization()(cnt_mod3)
cnt_mod4 = Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(4, 8))(cnt_mod3)
cnt_mod4 = Dropout(rate=0.25)(cnt_mod4)
cnt_mod4 = BatchNormalization()(cnt_mod4)
cnt_mod5 = Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(8, 16))(cnt_mod4)
cnt_mod5 = Dropout(rate=0.25)(cnt_mod5)
cnt_mod5 = BatchNormalization()(cnt_mod5)
cnt_mod6 = Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(16, 32))(cnt_mod5)
cnt_mod6 = Dropout(rate=0.25)(cnt_mod6)
cnt_mod6 = BatchNormalization()(cnt_mod6)
cnt_mod7 = Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(32, 64))(cnt_mod6)
cnt_mod7 = Dropout(rate=0.25)(cnt_mod7)
cnt_mod7 = BatchNormalization()(cnt_mod7)
cnt_mod8 = Conv2D(filters=32, kernel_size=(1, 1))(cnt_mod7)

decoder1 = MaxUnpooling2D((2, 2))([cnt_mod8, mask_1])
decoder2 = Conv2D(filters=32, kernel_size=(3, 3), activation='elu', strides=(1, 1), padding='same')(decoder1)
decoder2 = BatchNormalization()(decoder2)
decoder3 = Conv2D(filters=2, kernel_size=(3, 3), activation='elu', strides=(1, 1), padding='same')(decoder2)
decoder3 = BatchNormalization()(decoder3)
reshape = Reshape((2, 400 * 200))(decoder3)
permute = Permute(dims=(2, 1))(reshape)
outputs = Activation("softmax")(permute)

model = Model(inputs=inputs, outputs=outputs, name="LoDNN")
print(model.summary())

model.compile(loss='categorical_crossentropy', optimizer=SGD(), metrics=['accuracy'])
model.load_weights('weights-improvement-100-0.91.hdf5')

plot_model(model, to_file='model.png', show_shapes=True)

output = model.predict(test_data, batch_size=4, verbose=0)
print output.shape
output = output.reshape((output.shape[0], img_h, img_w, n_labels))

n = 3  # how many digits we will display
plt.figure(figsize=(20, 4))
for i in range(n):
    plt.imshow(test_data[i].reshape(400, 200), 'gray', interpolation='none')
    result = output[i].reshape(400, 200, 2)[:, :, 0]
    raw = result.copy()
    result = cv2.cvtColor(result, cv2.COLOR_GRAY2BGRA)
    print result.shape
    print result
    # compare = np.full((400, 200, 3), 0.9)
    # result[np.greater(result, compare)] = np.array([1, 0, 1])
    indices = np.where(result < 0.2)
    result.fill(0)
    result[indices[0], indices[1], :] = [0.9, 0.1, 0.7, 0.8]
    plt.axis('off')
    plt.imshow(result, interpolation='none')
    plt.savefig('results/result_kitti_train_praesi' + str(i) + '.png', bbox_inches='tight')
    # display original
    # ax = plt.subplot(2, n, i + 1)
    # plt.imshow(test_data[i].reshape(400, 200))
    # plt.gray()
    # ax.get_xaxis().set_visible(False)
    # ax.get_yaxis().set_visible(False)
    # plt.title(['um', 'umm', 'uu'][i % 3] + str(int(i / 3)))
    #
    # # display reconstruction
    # ax = plt.subplot(2, n, i + 1 + n)
    # plt.imshow(output[i].reshape(400, 200, 2)[:, :, 0])
    # plt.gray()
    # ax.get_xaxis().set_visible(False)
    # ax.get_yaxis().set_visible(False)
#plt.show()
