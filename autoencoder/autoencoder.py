from keras.layers import Permute, Softmax, Conv2D, Flatten, MaxPool2D, Dropout, Reshape, Activation, Input
from keras.models import Sequential, Model
from keras.optimizers import Adam
from keras.preprocessing.image import load_img, img_to_array
import numpy as np
from max_unpooling_layers import MaxPoolingWithArgmax2D, MaxUnpooling2D
import time
import tensorflow as tf
from keras.callbacks import TensorBoard


img_w = 200
img_h = 400
n_labels = 2


def label_map(labels):
    label_map = np.zeros([img_h, img_w, n_labels])
    for r in range(img_h):
        for c in range(img_w):
            label_map[r, c, int(labels[r][c])] = 1
    return label_map


def data_processor(split_range):
    prefix_list = ['um', 'umm', 'uu']
    data = []
    labels = []

    for theta in range(-30, 30, 5):
        for prefix in prefix_list:
            for i in split_range:
                iterator = "%06d" % i
                try:
                    feature = img_to_array(
                        load_img('../data/features/' + prefix + '_' + iterator + '_' + str(theta) + '.png',
                                 color_mode="grayscale"))
                    data.append(feature)
                    label = img_to_array(
                        load_img('../data/labels/ground_truth_transformed_' + prefix + '_' + iterator + '_' + str(
                            theta) + '.png', color_mode="grayscale")).reshape((img_h, img_w))
                    label = label_map(np.clip(label, 0, 1))
                    label = label.reshape((img_h * img_w, n_labels))
                    labels.append(label)
                except:
                    pass

    data = np.array(data)
    labels = np.array(labels)
    return data, labels


X_train, y_train = data_processor(range(0, 73))
X_test, y_test = data_processor(range(73, 89))

config = tf.ConfigProto()
config.gpu_options.allow_growth = True
session = tf.Session(config=config)


inputs = Input(shape=(img_h, img_w, 1))
encoded1 = Conv2D(filters=32, kernel_size=(3, 3), activation='elu', strides=(1, 1), padding='same')(inputs)
encoded2 = Conv2D(filters=32, kernel_size=(3, 3), activation='elu', strides=(1, 1), padding='same')(encoded1)
max_pool, mask_1 = MaxPoolingWithArgmax2D((2, 2))(encoded2)
cnt_mod1 = Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(1, 1))(max_pool)
cnt_mod1 = Dropout(rate=0.25)(cnt_mod1)
cnt_mod2 = Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(1, 2))(cnt_mod1)
cnt_mod2 = Dropout(rate=0.25)(cnt_mod2)
cnt_mod3 = Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(2, 4))(cnt_mod2)
cnt_mod3 = Dropout(rate=0.25)(cnt_mod3)
cnt_mod4 = Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(4, 8))(cnt_mod3)
cnt_mod4 = Dropout(rate=0.25)(cnt_mod4)
cnt_mod5 = Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(8, 16))(cnt_mod4)
cnt_mod5 = Dropout(rate=0.25)(cnt_mod5)
cnt_mod6 = Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(16, 32))(cnt_mod5)
cnt_mod6 = Dropout(rate=0.25)(cnt_mod6)
cnt_mod7 = Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(32, 64))(cnt_mod6)
cnt_mod7 = Dropout(rate=0.25)(cnt_mod7)
cnt_mod8 = Conv2D(filters=32, kernel_size=(1, 1))(cnt_mod7)

decoder1 = MaxUnpooling2D((2, 2))([cnt_mod8, mask_1])
decoder2 = Conv2D(filters=32, kernel_size=(3, 3), activation='elu', strides=(1, 1), padding='same')(decoder1)
decoder3 = Conv2D(filters=2, kernel_size=(3, 3), activation='elu', strides=(1, 1), padding='same')(decoder2)
reshape = Reshape((2, 400 * 200))(decoder3)
permute = Permute(dims=(2, 1))(reshape)
outputs = Activation("softmax")(permute)

model = Model(inputs=inputs, outputs=outputs, name="LoDNN")
print(model.summary())

model.compile(loss='categorical_crossentropy', optimizer=Adam(lr=0.01, decay=1e-2), metrics=['accuracy'])
model.fit(X_train, y_train, shuffle=True, batch_size=4, epochs=100, validation_data=(X_test, y_test))

model.save_weights("autoencoder" + str(time.time()) + ".hdf5")
