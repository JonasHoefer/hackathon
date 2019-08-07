from keras.layers import Permute, Softmax, Conv2D, Flatten, MaxPool2D, UpSampling2D
from keras.models import Sequential, Model
from keras.optimizers import Adam
import cv2
import numpy as np

X_train = np.expand_dims(cv2.imread('/home/mechlab/hackathon/data/features/um_000000.png'), 0)
y_train = np.expand_dims(cv2.imread('/home/mechlab/hackathon/data/labels_filtered/label_um_000000.png')[:, :, 0], 0)

autoencoder = Sequential()
# autoencoder.add(Input(shape=(400, 200, 1)))
autoencoder.add(Conv2D(filters=32, kernel_size=(3, 3), activation='elu', strides=(1, 1), padding='same'))
# autoencoder.add(Conv2D(filters=32, kernel_size=(3, 3), activation='elu', strides=(1, 1), padding='same'))
# autoencoder.add(MaxPool2D(pool_size=(2, 2), strides=(2, 2)))
# autoencoder.add(
#     Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(1, 1)))
# autoencoder.add(
#     Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(1, 2)))
# autoencoder.add(
#     Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(1, 2)))
# autoencoder.add(
#     Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(1, 2)))
# autoencoder.add(
#     Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(1, 2)))
# autoencoder.add(
#     Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(1, 2)))
# autoencoder.add(
#     Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(1, 2)))
# autoencoder.add(Conv2D(filters=32, kernel_size=(1, 1)))
# autoencoder.add(UpSampling2D(size=(2, 2)))
# autoencoder.add(Conv2D(filters=32, kernel_size=(3, 3), activation='elu', strides=(1, 1), padding='same'))
# autoencoder.add(Conv2D(filters=2, kernel_size=(3, 3), activation='elu', strides=(1, 1), padding='same'))
# autoencoder.add(Permute(dims=(1, 2, 0)))
# autoencoder.add(Softmax(axis=-1))

autoencoder.build(input_shape=(400, 200, 1))
print autoencoder.summary()

# autoencoder.compile(loss='categorical_crossentropy', optimizer=Adam(lr=0.01), metrics=['accuracy'])
# TODO: replace with testing data
# autoencoder.fit(X_train, y_train, batch_size=4, epochs=5, validation_data=(X_train, y_train))

# input_img = Input(shape=(400, 200, 1))
# encoded1 = Conv2D(filters=32, kernel_size=(3, 3), activation='elu', strides=(1, 1), padding='same')(input_img)
# encoded2 = Conv2D(filters=32, kernel_size=(3, 3), activation='elu', strides=(1, 1), padding='same')(encoded1)
# max_pool = MaxPool2D(pool_size=(2, 2), strides=(2, 2))(encoded2)
# cnt_mod1 = Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(1, 1))(max_pool)
# cnt_mod2 = Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(1, 2))(cnt_mod1)
# cnt_mod3 = Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(1, 2))(cnt_mod2)
# cnt_mod4 = Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(1, 2))(cnt_mod3)
# cnt_mod5 = Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(1, 2))(cnt_mod4)
# cnt_mod6 = Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(1, 2))(cnt_mod5)
# cnt_mod7 = Conv2D(filters=128, kernel_size=(3, 3), padding='same', activation='elu', dilation_rate=(1, 2))(cnt_mod6)
# cnt_mod8 = Conv2D(filters=32, kernel_size=(1, 1))(cnt_mod7)
#
# decoder1 = UpSampling2D(size=(2, 2))(cnt_mod8)
# decoder2 = Conv2D(filters=32, kernel_size=(3, 3), activation='elu', strides=(1, 1), padding='same')(decoder1)
# decoder3 = Conv2D(filters=2, kernel_size=(3, 3), activation='elu', strides=(1, 1), padding='same')(decoder2)
# softmax = Softmax(axis=-1)(decoder3)
